#include <Arduino.h>

// L298N pin definitions for Arduino Nano
#define ENA 9    // PWM for Motor A speed
#define ENB 10   // PWM for Motor B speed
#define IN1 4    // Motor A direction 1
#define IN2 3    // Motor A direction 2
#define IN3 6    // Motor B direction 1
#define IN4 7    // Motor B direction 2

// 8-channel IR sensor array pin definitions
#define SENSOR1 A0  // Leftmost sensor
#define SENSOR2 A1
#define SENSOR3 A2
#define SENSOR4 A3
#define SENSOR5 A4
#define SENSOR6 A5
#define SENSOR7 A6
#define SENSOR8 A7

// Motor control variables
const int offsetA = 1;
const int offsetB = 1;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200; // Base speed for N20 200 RPM motors at 5V

// PID constants (tuned for 3mm sensing distance)
float Kp = 0.0;
float Kd = 0.0;
float Ki = 0.0;

int minValues[9], maxValues[9], threshold[9]; // Array size 9 to match 1-8 indexing

// Motor control function for L298N
void motorDrive(int motor, int speed, int offset) {
  if (motor == 1) { // Motor A
    if (speed * offset > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, abs(speed));
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, abs(speed));
    }
  } else { // Motor B
    if (speed * offset > 0) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, abs(speed));
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, abs(speed));
    }
  }
}

void setup() {
  Serial.begin(9600);
  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  // Initialize control pins
  pinMode(11, INPUT_PULLUP); // Calibration start
  pinMode(12, INPUT_PULLUP); // Run start
  // Sensor pins A0–A7 are inputs by default
}

void loop() {
  while (digitalRead(11)) {} // Wait for calibration trigger
  delay(1000);
  calibrate();
  while (digitalRead(12)) {} // Wait for run trigger
  delay(1000);

  while (1) {
    // Sharp turns based on outermost sensors (105mm span for 8 sensors at 15mm spacing)
    if (analogRead(SENSOR1) > threshold[1] && analogRead(SENSOR8) < threshold[8]) {
      lsp = 0;
      rsp = lfspeed;
      motorDrive(1, 0, offsetA);
      motorDrive(2, lfspeed, offsetB);
    }
    else if (analogRead(SENSOR8) > threshold[8] && analogRead(SENSOR1) < threshold[1]) {
      lsp = lfspeed;
      rsp = 0;
      motorDrive(1, lfspeed, offsetA);
      motorDrive(2, 0, offsetB);
    }
    else if (analogRead(SENSOR4) > threshold[4] || analogRead(SENSOR5) > threshold[5]) {
      // Dynamic PID tuning based on center sensors
      Kp = 0.0006 * (1000 - (analogRead(SENSOR4) + analogRead(SENSOR5)) / 2);
      Kd = 10 * Kp;
      linefollow();
    }
  }
}

void linefollow() {
  // Error calculation using weighted sensor readings for finer control
  int sensors[] = {SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5, SENSOR6, SENSOR7, SENSOR8};
  int weights[] = {-7, -5, -3, -1, 1, 3, 5, 7}; // Weights for 8 sensors (left to right)
  int sum = 0, weightedSum = 0;
  for (int i = 0; i < 8; i++) {
    int value = analogRead(sensors[i]);
    sum += value;
    weightedSum += value * weights[i];
  }
  error = weightedSum / (sum / 8); // Normalize to center line

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  // Constrain motor speeds (0–255 for PWM)
  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, 0, 255);

  motorDrive(1, lsp, offsetA);
  motorDrive(2, rsp, offsetB);
}

void calibrate() {
  // Initialize min/max values for 8 sensors (A0–A7)
  int sensors[] = {SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5, SENSOR6, SENSOR7, SENSOR8};
  for (int i = 1; i <= 8; i++) {
    minValues[i] = analogRead(sensors[i-1]);
    maxValues[i] = analogRead(sensors[i-1]);
  }
  
  // Calibration movement (slow rotation for N20 motors)
  for (int i = 0; i < 3000; i++) {
    motorDrive(1, 50, offsetA);
    motorDrive(2, -50, offsetB);

    for (int i = 1; i <= 8; i++) {
      int val = analogRead(sensors[i-1]);
      if (val < minValues[i]) {
        minValues[i] = val;
      }
      if (val > maxValues[i]) {
        maxValues[i] = val;
      }
    }
  }

  // Calculate thresholds
  for (int i = 1; i <= 8; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  // Stop motors
  motorDrive(1, 0, offsetA);
  motorDrive(2, 0, offsetB);
}
