/*
 * Competition-Level PID Line Follower Robot
 * 
 * Hardware:
 * - Arduino Nano
 * - RLS-08 8-sensor array (TCRT5000 based)
 * - L298N motor driver
 * - 2x N20 geared motors
 * 
 * Author: Manus AI
 * Date: 2025
 * 
 * This code implements a high-performance PID-based line follower
 * optimized for international competition standards with advanced
 * turning capabilities and fast response times.
 */

// ===== PIN DEFINITIONS =====
// L298N Motor Driver Pins
#define ENA_PIN 9   // PWM pin for left motor speed control
#define IN1_PIN 8   // Direction pin 1 for left motor
#define IN2_PIN 7   // Direction pin 2 for left motor
#define ENB_PIN 10  // PWM pin for right motor speed control
#define IN3_PIN 11  // Direction pin 1 for right motor
#define IN4_PIN 12  // Direction pin 2 for right motor

// RLS-08 Sensor Array Analog Pins
#define SENSOR_0 A0  // Leftmost sensor
#define SENSOR_1 A1
#define SENSOR_2 A2
#define SENSOR_3 A3
#define SENSOR_4 A4
#define SENSOR_5 A5
#define SENSOR_6 A6
#define SENSOR_7 A7  // Rightmost sensor

// Additional pins
#define CALIBRATION_BUTTON 2  // Button for sensor calibration
#define STATUS_LED 13         // Built-in LED for status indication

// ===== GLOBAL VARIABLES =====
// Sensor readings and calibration
int sensorValues[8];
int sensorMin[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int sensorMax[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int calibratedSensorValues[8];

// PID variables
float Kp = 2.0;    // Proportional gain (tune this first)
float Ki = 0.0;    // Integral gain (tune after Kp)
float Kd = 1.0;    // Derivative gain (tune last)

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;

// Motor control variables
int baseSpeed = 150;        // Base motor speed (0-255)
int maxSpeed = 255;         // Maximum motor speed
int minSpeed = 0;           // Minimum motor speed
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// Advanced features
bool lineDetected = false;
unsigned long lastLineTime = 0;
unsigned long lineSearchTimeout = 2000;  // 2 seconds to search for line
bool isCalibrated = false;

// Competition features
int turnSpeed = 200;        // Speed for sharp turns
int searchSpeed = 120;      // Speed when searching for line
bool aggressiveMode = true; // Enable aggressive turning for competition

// Variable to track current robot mode (PID or manual control)
bool manualControlMode = false;

void setup() {
  Serial.begin(9600);
  
  // Initialize motor driver pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  
  // Initialize other pins
  pinMode(CALIBRATION_BUTTON, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  
  // Stop motors initially
  stopMotors();
  
  Serial.println("Competition PID Line Follower Initialized");
  Serial.println("Press calibration button to start calibration...");
  
  // Wait for calibration
  waitForCalibration();
}

void loop() {
  // Check for serial commands first
  serialTuning();

  if (!manualControlMode) {
    // Read sensors
    readSensors();
    
    // Calculate error for PID
    calculateError();
    
    // Calculate PID output
    calculatePID();
    
    // Apply motor control
    applyMotorControl();
    
    // Handle line loss scenarios
    handleLineLoss();
  }
  
  // Small delay for stability
  delay(5);
}

// ===== SENSOR FUNCTIONS =====
void readSensors() {
  sensorValues[0] = analogRead(SENSOR_0);
  sensorValues[1] = analogRead(SENSOR_1);
  sensorValues[2] = analogRead(SENSOR_2);
  sensorValues[3] = analogRead(SENSOR_3);
  sensorValues[4] = analogRead(SENSOR_4);
  sensorValues[5] = analogRead(SENSOR_5);
  sensorValues[6] = analogRead(SENSOR_6);
  sensorValues[7] = analogRead(SENSOR_7);
  
  // Calibrate sensor values
  for (int i = 0; i < 8; i++) {
    calibratedSensorValues[i] = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    calibratedSensorValues[i] = constrain(calibratedSensorValues[i], 0, 1000);
  }
}

void waitForCalibration() {
  while (digitalRead(CALIBRATION_BUTTON) == HIGH) {
    digitalWrite(STATUS_LED, millis() % 500 < 250); // Blink LED
    delay(10);
  }
  
  calibrateSensors();
}

void calibrateSensors() {
  Serial.println("Starting calibration...");
  Serial.println("Move robot over white and black surfaces for 10 seconds");
  
  digitalWrite(STATUS_LED, HIGH);
  
  unsigned long calibrationStart = millis();
  while (millis() - calibrationStart < 10000) { // 10 second calibration
    readRawSensors();
    
    for (int i = 0; i < 8; i++) {
      if (sensorValues[i] < sensorMin[i]) {
        sensorMin[i] = sensorValues[i];
      }
      if (sensorValues[i] > sensorMax[i]) {
        sensorMax[i] = sensorValues[i];
      }
    }
    
    // Blink LED during calibration
    digitalWrite(STATUS_LED, (millis() % 200) < 100);
    delay(10);
  }
  
  digitalWrite(STATUS_LED, LOW);
  isCalibrated = true;
  
  Serial.println("Calibration complete!");
  printCalibrationValues();
  
  delay(1000);
}

void readRawSensors() {
  sensorValues[0] = analogRead(SENSOR_0);
  sensorValues[1] = analogRead(SENSOR_1);
  sensorValues[2] = analogRead(SENSOR_2);
  sensorValues[3] = analogRead(SENSOR_3);
  sensorValues[4] = analogRead(SENSOR_4);
  sensorValues[5] = analogRead(SENSOR_5);
  sensorValues[6] = analogRead(SENSOR_6);
  sensorValues[7] = analogRead(SENSOR_7);
}

void printCalibrationValues() {
  Serial.println("Calibration values:");
  Serial.print("Min: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorMin[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.print("Max: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorMax[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// ===== PID CALCULATION =====
void calculateError() {
  // Weighted position calculation for 8 sensors
  // Sensor positions: -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5
  float weightedSum = 0;
  float totalActivation = 0;
  
  // Check if any sensor detects the line
  lineDetected = false;
  for (int i = 0; i < 8; i++) {
    if (calibratedSensorValues[i] > 300) { // Threshold for line detection
      lineDetected = true;
      break;
    }
  }
  
  if (lineDetected) {
    lastLineTime = millis();
    
    // Calculate weighted position
    float positions[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
    
    for (int i = 0; i < 8; i++) {
      weightedSum += calibratedSensorValues[i] * positions[i];
      totalActivation += calibratedSensorValues[i];
    }
    
    if (totalActivation > 0) {
      error = weightedSum / totalActivation;
    } else {
      error = 0;
    }
  } else {
    // Line lost - maintain last error direction for searching
    if (previousError > 0) {
      error = 3.5;  // Search right
    } else {
      error = -3.5; // Search left
    }
  }
}

void calculatePID() {
  // Proportional term
  float proportional = error;
  
  // Integral term (with windup protection)
  integral += error;
  integral = constrain(integral, -100, 100);
  
  // Derivative term
  derivative = error - previousError;
  
  // Calculate PID output
  pidOutput = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  
  // Store current error for next iteration
  previousError = error;
  
  // Constrain PID output
  pidOutput = constrain(pidOutput, -maxSpeed, maxSpeed);
}

// ===== MOTOR CONTROL =====
void applyMotorControl() {
  if (lineDetected) {
    // Normal line following mode
    if (aggressiveMode && abs(error) > 2.0) {
      // Aggressive turning for sharp curves
      if (error > 0) {
        // Sharp right turn
        turnRight(turnSpeed); 
      } else {
        // Sharp left turn
        turnLeft(turnSpeed); 
      }
    } else {
      // Normal PID control
      leftMotorSpeed = baseSpeed - pidOutput;
      rightMotorSpeed = baseSpeed + pidOutput;
      
      // Use setLeftMotor and setRightMotor for PID output
      setLeftMotor(leftMotorSpeed);
      setRightMotor(rightMotorSpeed);
    }
  } else {
    // Line search mode
    if (error > 0) {
      // Search right
      turnRight(searchSpeed); 
    } else {
      // Search left
      turnLeft(searchSpeed); 
    }
  }
}

void setLeftMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, speed);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, -speed);
  }
}

void setRightMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, speed);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, -speed);
  }
}

void stopMotors() {
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

// New direct motor control functions
void forward(int speed) {
  setLeftMotor(speed);
  setRightMotor(speed);
  Serial.print("Moving Forward at speed: ");
  Serial.println(speed);
}

void backward(int speed) {
  setLeftMotor(-speed);
  setRightMotor(-speed);
  Serial.print("Moving Backward at speed: ");
  Serial.println(speed);
}

void turnLeft(int speed) {
  setLeftMotor(-speed); // Left motor backward
  setRightMotor(speed); // Right motor forward
  Serial.print("Turning Left at speed: ");
  Serial.println(speed);
}

void turnRight(int speed) {
  setLeftMotor(speed);  // Left motor forward
  setRightMotor(-speed); // Right motor backward
  Serial.print("Turning Right at speed: ");
  Serial.println(speed);
}

// ===== ADVANCED FEATURES =====
void handleLineLoss() {
  if (!lineDetected && (millis() - lastLineTime > lineSearchTimeout)) {
    // Line lost for too long - stop and indicate error
    stopMotors();
    digitalWrite(STATUS_LED, HIGH);
    
    Serial.println("Line lost - stopping robot");
    
    // Wait for manual intervention or line detection
    while (!lineDetected) {
      readSensors();
      calculateError();
      delay(100);
    }
    
    digitalWrite(STATUS_LED, LOW);
    Serial.println("Line found - resuming");
  }
}

// ===== UTILITY FUNCTIONS =====
void printDebugInfo() {
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | PID: ");
  Serial.print(pidOutput);
  Serial.print(" | Left: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" | Right: ");
  Serial.print(rightMotorSpeed);
  Serial.print(" | Line: ");
  Serial.println(lineDetected ? "YES" : "NO");
}

// Function to tune PID values via Serial (for advanced users)
void serialTuning() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command.startsWith("kp")) {
      Kp = command.substring(2).toFloat();
      Serial.print("Kp set to: ");
      Serial.println(Kp);
      manualControlMode = false; // Exit manual control if PID param is set
    } else if (command.startsWith("ki")) {
      Ki = command.substring(2).toFloat();
      Serial.print("Ki set to: ");
      Serial.println(Ki);
      manualControlMode = false;
    } else if (command.startsWith("kd")) {
      Kd = command.substring(2).toFloat();
      Serial.print("Kd set to: ");
      Serial.println(Kd);
      manualControlMode = false;
    } else if (command.startsWith("speed")) {
      baseSpeed = command.substring(5).toInt();
      Serial.print("Base speed set to: ");
      Serial.println(baseSpeed);
      manualControlMode = false;
    } else if (command == "debug") {
      printDebugInfo();
      manualControlMode = false;
    } else if (command == "calibrate") {
      calibrateSensors();
      manualControlMode = false;
    } else if (command.startsWith("forward")) {
      int speed = command.substring(7).toInt();
      forward(speed);
      manualControlMode = true; // Enter manual control mode
    } else if (command.startsWith("backward")) {
      int speed = command.substring(8).toInt();
      backward(speed);
      manualControlMode = true;
    } else if (command.startsWith("left")) {
      int speed = command.substring(4).toInt();
      turnLeft(speed);
      manualControlMode = true;
    } else if (command.startsWith("right")) {
      int speed = command.substring(5).toInt();
      turnRight(speed);
      manualControlMode = true;
    } else if (command == "stop") {
      stopMotors();
      manualControlMode = true;
      Serial.println("Motors stopped. Manual control active.");
    } else if (command == "pid") {
      manualControlMode = false;
      Serial.println("Exiting manual control. Resuming PID line following.");
    }
  }
}


