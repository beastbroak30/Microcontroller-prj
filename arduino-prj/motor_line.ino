#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// IR receiver pin
const uint16_t RECV_PIN = D4; 
// L298N motor driver pins
const uint8_t ENA = D7;
const uint8_t IN1 = D1;
const uint8_t IN2 = D2;
const uint8_t ENB = D8;
const uint8_t IN3 = D6;
const uint8_t IN4 = D5;
// IR code for the specific button
const uint32_t BUTTON_CODE = 0xFF38C7;
const uint32_t BUTTON_CODE2 = 0xFF6897;

IRrecv irrecv(RECV_PIN);
decode_results results;

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver

  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set initial state of the motor to off
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(resultToHumanReadableBasic(&results)); // Print the result in human-readable form
    if (results.value == BUTTON_CODE) {
      moveMotor();
      Serial.println("curtain open");
    } else if (results.value == BUTTON_CODE2){
      moveMotor2();
      Serial.println("curtain close");
    }
    irrecv.resume(); // Receive the next value
  }
  delay(100);
}

void moveMotor() {
  // Move motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255); // Set motor speed (0-255)
  analogWrite(ENB, 255);
  delay(3000); // Run motor for 1 second (adjust as needed)

  // Stop motor
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveMotor2() {
  // Move motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255); // Set motor speed (0-255)
  analogWrite(ENB, 255);
  delay(1000); // Run motor for 1 second (adjust as needed)

  // Stop motor
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}