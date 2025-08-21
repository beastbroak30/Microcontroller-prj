// === Pins (same as yours) ===
#define ENA 6   // PWM Left (Motor A)
#define ENB 5   // PWM Right (Motor B)
#define IN1 10  // Left dir 1
#define IN2 9   // Left dir 2
#define IN3 8   // Right dir 1
#define IN4 7   // Right dir 2

// === Tunables ===
// Base trims (fix left=“smooth/weak”, right=“jerky”)
int baseLeft  = 170;   // tune for straight drive
int baseRight = 160;

// Speed caps (protect from fresh battery overshoot)
const int MAX_SPEED = 230;
const int MIN_SPEED = 0;

// Blind-forward window (keeps creeping before search)
const int BLIND_SPEED = 130;
const unsigned long BLIND_TIME = 300; // ms

// Search behavior (NEVER STOP): oscillate spins with growing sweep time
const int  SEARCH_SPIN_SPEED_L = 150;    // left motor PWM during spin
const int  SEARCH_SPIN_SPEED_R = 150;    // right motor PWM during spin
const int  SEARCH_SPIN_SPEED_BOOST = 20; // add bias so spin is decisive
const unsigned long SEARCH_PHASE_MIN = 250;  // ms initial sweep each side
const unsigned long SEARCH_PHASE_MAX = 900;  // ms cap per sweep
const unsigned long SEARCH_PHASE_STEP = 100; // how much longer each swap

// Sensor thresholds (use your values)
const int THRESH_ON  = 200;  // >200 = black
const int THRESH_OFF = 80;   // <80  = white

// PID gains (start here, then tune)
float Kp = 0.6;
float Ki = 0.00;  // start at 0
float Kd = 7.0;

// Anti-windup
const float I_MAX = 200.0;

// === Internals ===
int s[8];
const int w[8] = {-350, -250, -150, -50, 50, 150, 250, 350};

float lastError = 0.0;
float integral  = 0.0;
unsigned long lastUpdate = 0;
unsigned long lastSeen   = 0;

// Search state
bool inSearch = false;
int  searchDir = 1; // +1 spin right, -1 spin left
unsigned long searchPhaseDur = SEARCH_PHASE_MIN;
unsigned long searchPhaseStart = 0;

// ---------- Utils ----------
int readA(int apin) {
  int v = analogRead(apin);
  if (v > THRESH_ON)  return 1;
  if (v < THRESH_OFF) return 0;
  return 0;
}

void driveForwardPWM(int lPWM, int rPWM) {
  // Clamp
  if (lPWM > MAX_SPEED) lPWM = MAX_SPEED;
  if (rPWM > MAX_SPEED) rPWM = MAX_SPEED;
  if (lPWM < MIN_SPEED) lPWM = MIN_SPEED;
  if (rPWM < MIN_SPEED) rPWM = MIN_SPEED;

  // Forward both
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, lPWM);
  analogWrite(ENB, rPWM);
}

void spinInPlace(int dir, int speedL, int speedR) {
  // dir = +1 spin right (left backward, right forward)
  // dir = -1 spin left  (left forward,  right backward)
  if (dir > 0) {
    // left backward
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    // right forward
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    // left forward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    // right backward
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }
  // Clamp
  if (speedL > MAX_SPEED) speedL = MAX_SPEED;
  if (speedR > MAX_SPEED) speedR = MAX_SPEED;
  if (speedL < MIN_SPEED) speedL = MIN_SPEED;
  if (speedR < MIN_SPEED) speedR = MIN_SPEED;

  analogWrite(ENA, speedL);
  analogWrite(ENB, speedR);
}

bool anySeen() {
  for (int i = 0; i < 8; i++) if (s[i]) return true;
  return false;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  lastUpdate = millis();
  lastSeen   = millis();
  inSearch = false;
}

// ---------- Loop ----------
void loop() {
  // Read sensors A0..A7
  s[0] = readA(A0);
  s[1] = readA(A1);
  s[2] = readA(A2);
  s[3] = readA(A3);
  s[4] = readA(A4);
  s[5] = readA(A5);
  s[6] = readA(A6);
  s[7] = readA(A7);

  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastUpdate = now;

  // If any sensor sees the line -> TRACK (PID), exit search
  if (anySeen()) {
    inSearch = false;
    lastSeen = now;

    // Weighted position
    int count = 0; long sumW = 0;
    for (int i = 0; i < 8; i++) {
      if (s[i]) { count++; sumW += w[i]; }
    }
    float pos = (float)sumW / (float)count;   // 0 is center
    float err = -pos;                         // error (target 0)

    // PID
    integral += err * dt;
    if (integral >  I_MAX) integral =  I_MAX;
    if (integral < -I_MAX) integral = -I_MAX;
    float deriv = (err - lastError) / dt;
    lastError = err;

    float corr = Kp*err + Ki*integral + Kd*deriv;

    // Convert to motor PWMs (trim + correction)
    int lPWM = baseLeft  - (int)corr;
    int rPWM = baseRight + (int)corr;

    driveForwardPWM(lPWM, rPWM);
    return; // done this cycle
  }

  // No sensors see line: BLIND FORWARD then SEARCH (but never stop)
  if (!inSearch) {
    if (now - lastSeen < BLIND_TIME) {
      // creep forward, tiny bias helps re-acquire
      driveForwardPWM(BLIND_SPEED - 5, BLIND_SPEED);
    } else {
      // enter search mode
      inSearch = true;
      searchDir = 1; // start spinning right
      searchPhaseDur = SEARCH_PHASE_MIN;
      searchPhaseStart = now;
    }
    return;
  }

  // --- SEARCH MODE (always moving) ---
  // If a sweep phase expired, flip direction and lengthen phase (up to max)
  if (now - searchPhaseStart >= searchPhaseDur) {
    searchDir = -searchDir;
    searchPhaseStart = now;
    if (searchPhaseDur + SEARCH_PHASE_STEP <= SEARCH_PHASE_MAX) {
      searchPhaseDur += SEARCH_PHASE_STEP;
    }
  }

  // Spin decisively. Add small boost to the outer wheel so it pivots cleanly.
  if (searchDir > 0) {
    // spin right
    spinInPlace(+1,
      SEARCH_SPIN_SPEED_L + SEARCH_SPIN_SPEED_BOOST,
      SEARCH_SPIN_SPEED_R);
  } else {
    // spin left
    spinInPlace(-1,
      SEARCH_SPIN_SPEED_L,
      SEARCH_SPIN_SPEED_R + SEARCH_SPIN_SPEED_BOOST);
  }
}
