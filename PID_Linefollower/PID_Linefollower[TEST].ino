#include <Arduino.h>

// L298N pin definitions for Arduino Nano
#define ENA 9    // PWM for Motor A speed
#define ENB 10   // PWM for Motor B speed
#define IN1 4    // Motor A direction 1
#define IN2 3    // Motor A direction 2
#define IN3 6    // Motor B direction 1
#define IN4 7    // Motor B direction 2

// 8-channel IR sensor array pin definitions (reversed for IR1 = rightmost)
#define SENSOR1 A0  // Rightmost sensor
#define SENSOR2 A1
#define SENSOR3 A2
#define SENSOR4 A3
#define SENSOR5 A4
#define SENSOR6 A5
#define SENSOR7 A6
#define SENSOR8 A7  // Leftmost sensor

// Motor control variables
const int offsetA = 1;    // Motor A direction offset
const int offsetB = 1;    // Motor B direction offset
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;        // Base speed for N20 200 RPM motors at 5V

// PID constants (initial fixed values for tuning)
float Kp = 0.1;           // Proportional gain
float Kd = 1.0;           // Derivative gain
float Ki = 0.0001;        // Integral gain (enabled)
const int I_MAX = 100;    // Max integral value to prevent windup

int minValues[9], maxValues[9], threshold[9]; // Array size 9 for 1-8 indexing

// Motor control function for L298N
void motorDrive(int motor, int speed, int offset) {
  speed = constrain(speed, 0, 255); // Ensure valid PWM range
  if (motor == 1) { // Motor A
    if (speed * offset > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, speed);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, speed);
    }
  } else { // Motor B
    if (speed * offset > 0) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, speed);
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, speed);
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
  pinMode(11, INPUT_PULLUP); // Calibration trigger
  pinMode(12, INPUT_PULLUP); // Run trigger
  // Sensor pins A0–A7 are inputs by default
  Serial.println("Robot initialized. Press pin 11 button to calibrate.");
}

void loop() {
  while (digitalRead(11)) {} // Wait for calibration trigger
  Serial.println("Starting calibration...");
  delay(1000);
  calibrate();
  Serial.println("Calibration complete. Press pin 12 to start.");
  while (digitalRead(12)) {} // Wait for run trigger
  delay(1000);
  Serial.println("Starting line following...");

  while (1) {
    // Sharp turns based on outermost sensors (105mm span, 15mm spacing)
    if (analogRead(SENSOR8) > threshold[8] && analogRead(SENSOR1) < threshold[1]) {
      lsp = 0;
      rsp = lfspeed;
      motorDrive(1, lsp, offsetA);
      motorDrive(2, rsp, offsetB);
      Serial.println("Sharp left turn");
    }
    else if (analogRead(SENSOR1) > threshold[1] && analogRead(SENSOR8) < threshold[8]) {
      lsp = lfspeed;
      rsp = 0;
      motorDrive(1, lsp, offsetA);
      motorDrive(2, rsp, offsetB);
      Serial.println("Sharp right turn");
    }
    else if (analogRead(SENSOR4) > threshold[4] || analogRead(SENSOR5) > threshold[5]) {
      linefollow();
    }
    // Debug output every 100ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      Serial.print("Error: "); Serial.print(error);
      Serial.print(" PID: "); Serial.print(PIDvalue);
      Serial.print(" Lsp: "); Serial.print(lsp);
      Serial.print(" Rsp: "); Serial.println(rsp);
      lastPrint = millis();
    }
  }
}

void linefollow() {
  // Error calculation using weighted sensor readings
  int sensors[] = {SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5, SENSOR6, SENSOR7, SENSOR8};
  int weights[] = {7, 5, 3, 1, -1, -3, -5, -7}; // Reversed weights for SENSOR1 = rightmost
  int sum = 0, weightedSum = 0;
  for (int i = 0; i < 8; i++) {
    int value = analogRead(sensors[i]);
    sum += value;
    weightedSum += value * weights[i];
  }
  error = (sum == 0) ? 0 : weightedSum / (sum / 8); // Avoid division by zero

  // PID calculations
  P = error;
  I = constrain(I + error, -I_MAX, I_MAX); // Limit integral windup
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // Adjust motor speeds
  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;
  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, 0, 255);

  motorDrive(1, lsp, offsetA);
  motorDrive(2, rsp, offsetB);
}

void calibrate() {
  // Initialize min/max values for 8 sensors
  int sensors[] = {SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5, SENSOR6, SENSOR7, SENSOR8};
  for (int i = 1; i <= 8; i++) {
    minValues[i] = analogRead(sensors[i-1]);
    maxValues[i] = minValues[i];
  }
  
  // Calibration movement (slow rotation for N20 motors)
  for (int i = 0; i < 3000; i++) {
    motorDrive(1, 50, offsetA);
    motorDrive(2, -50, offsetB);
    for (int j = 1; j <= 8; j++) {
      int val = analogRead(sensors[j-1]);
      if (val < minValues[j]) minValues[j] = val;
      if (val > maxValues[j]) maxValues[j] = val;
    }
  }

  // Calculate and print thresholds
  Serial.print("Thresholds: ");
  for (int i = 1; i <= 8; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  // Stop motors
  motorDrive(1, 0, offsetA);
  motorDrive(2, 0, offsetB);
}
