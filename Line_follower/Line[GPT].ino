// --- Pins (same as your setup) ---
#define ENA 6   // PWM Left (Motor A)
#define ENB 5   // PWM Right (Motor B)
#define IN1 10  // Left dir 1
#define IN2 9   // Left dir 2
#define IN3 8   // Right dir 1
#define IN4 7   // Right dir 2

// ---------- TUNABLES ----------
/* Motor base speeds (trim each side) */
int baseLeft  = 170;   // tune to make straight
int baseRight = 160;   // tune to make straight

/* Max speed caps (safety vs overshoot) */
const int MAX_SPEED = 230;  // never go above this
const int MIN_SPEED = 0;

/* Blind-forward behavior */
const int BLIND_SPEED = 130;     // creep speed when no line briefly
const unsigned long BLIND_TIME = 300; // ms to keep creeping before "lost"

/* Sensor thresholds (use yours) */
const int THRESH_ON  = 200;  // >200 = black line present
const int THRESH_OFF = 80;   // <80  = white

/* PID gains (start here, then tune) */
float Kp = 0.6;   // proportional
float Ki = 0.00;  // integral (start at 0)
float Kd = 7.0;   // derivative

/* Integral guard (anti-windup) */
const float I_MAX = 200.0;   // clamp integral term

// ---------- INTERNALS ----------
int s[8];                    // sensor binaries r1..r8
// Weights: left negative, right positive. Wider values = stronger correction.
const int w[8] = {-350, -250, -150, -50, 50, 150, 250, 350};

float lastError = 0.0;
float integral  = 0.0;
unsigned long lastUpdate = 0;
unsigned long lastSeen   = 0;

// ------------ UTILITIES ------------
int readA(int analogPin) {
  int value = analogRead(analogPin);
  // simple binary detection (use your thresholds)
  if (value > THRESH_ON)  return 1;
  if (value < THRESH_OFF) return 0;
  // between thresholds, keep as 0 (you can add hysteresis later)
  return 0;
}

void setMotorSpeeds(int leftPWM, int rightPWM) {
  // clamp
  if (leftPWM  > MAX_SPEED) leftPWM  = MAX_SPEED;
  if (rightPWM > MAX_SPEED) rightPWM = MAX_SPEED;
  if (leftPWM  < MIN_SPEED) leftPWM  = MIN_SPEED;
  if (rightPWM < MIN_SPEED) rightPWM = MIN_SPEED;

  // forward (both wheels forward)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, leftPWM);
  analogWrite(ENB, rightPWM);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ------------ SETUP ------------
void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  lastUpdate = millis();
  lastSeen   = millis();
}

// ------------ LOOP ------------
void loop() {
  // 1) Read sensors (A0..A7)
  s[0] = readA(A0);
  s[1] = readA(A1);
  s[2] = readA(A2);
  s[3] = readA(A3);
  s[4] = readA(A4);
  s[5] = readA(A5);
  s[6] = readA(A6);
  s[7] = readA(A7);

  // 2) Compute weighted position if any sensor sees the line
  int sumActive = 0;
  long weighted = 0;
  for (int i = 0; i < 8; i++) {
    if (s[i]) {
      sumActive += 1;
      weighted += w[i];
    }
  }

  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0; // seconds
  if (dt <= 0) dt = 0.001;
  lastUpdate = now;

  if (sumActive > 0) {
    // we saw the line; compute position and error
    float position = (float)weighted / (float)sumActive; // target is 0
    float error = 0.0 - position;

    // update "last seen"
    lastSeen = now;

    // 3) PID
    integral += error * dt;
    // anti-windup
    if (integral >  I_MAX) integral =  I_MAX;
    if (integral < -I_MAX) integral = -I_MAX;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float correction = Kp * error + Ki * integral + Kd * derivative;

    // 4) Convert correction to motor PWMs
    // If correction > 0 => line is to the left => speed up right / slow down left
    int leftPWM  = baseLeft  - (int)(correction);
    int rightPWM = baseRight + (int)(correction);

    setMotorSpeeds(leftPWM, rightPWM);
  } else {
    // No sensors see line: Blind-forward logic
    if (now - lastSeen < BLIND_TIME) {
      // creep forward with trims
      setMotorSpeeds(BLIND_SPEED - 5, BLIND_SPEED); // tiny bias helps re-capture
    } else {
      // really lost: stop (or you can spin-search here if you want)
      stopMotors();
      // Optional: quick search routine (uncomment to try)
      /*
      // simple wobble search
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); // left backward
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // right forward
      analogWrite(ENA, 150);
      analogWrite(ENB, 150);
      delay(120);
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // left forward
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // right backward
      analogWrite(ENA, 150);
      analogWrite(ENB, 150);
      delay(100);
      */
    }
  }
}
