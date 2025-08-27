#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// RF24 radio setup
RF24 radio(9, 8); // CE, CSN pins
const byte address[6] = "00001";
int K_state = 0; // Global K_state (Forward = 1, Backward = 2, Left = 3, Right = 4)

// Pin definitions for motors
#define ENA 5   // Enable pin for motor A
#define ENB 6   // Enable pin for motor B
#define IN1 2   // Motor A input 1
#define IN2 3   // Motor A input 2
#define IN3 4   // Motor B input 1
#define IN4 7   // Motor B input 2

// Data structure to receive
struct DataPacket {
  byte jl_x;
  byte jl_y;
  byte jr_x;
  byte jr_y;
  byte pot;
} data;

// Function prototypes
void forward(int motorSpeed);
void backward(int motorSpeed);
void left(int motorSpeed);
void right(int motorSpeed);
void stop();

void setup() {
  // Initialize motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
  // Print header
  Serial.println("Receiving Joystick & Potentiometer Values:");
  Serial.println("JL_X\tJL_Y\tJR_X\tJR_Y\tPOT");
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(DataPacket));
    
    // Print received values
    Serial.print(data.jl_x);
    Serial.print("\t");
    Serial.print(data.jl_y);
    Serial.print("\t");
    Serial.print(data.jr_x);
    Serial.print("\t");
    Serial.print(data.jr_y);
    Serial.print("\t");
    Serial.println(data.pot);

    // Map potentiometer value (0-255) to motor speed with 30% reduction
    int motorSpeed = (data.pot); // 30% reduction, always positive int

    // Control motors based on left joystick (jl_x, jl_y) or right joystick (jr_x, jr_y)
    if (data.jl_x >= 110 && data.jl_x <= 115 && data.jl_y >= 110 && data.jl_y <= 115 &&
        data.jr_x >= 110 && data.jr_x <= 115 && data.jr_y >= 110 && data.jr_y <= 115) {
      // Neutral zone: stop motors and handle K_state
      if (K_state == 1) {
        forward(motorSpeed);
        delay(50);
        stop();
        K_state = 0;
      } else if (K_state == 2) {
        backward(motorSpeed);
        delay(50);
        stop();
        K_state = 0;
      } else if (K_state == 3) {
        right(motorSpeed);
        delay(50);
        stop();
        K_state = 0;
      } else if (K_state == 4) {
        left(motorSpeed);
        delay(50);
        stop();
        K_state = 0;
      } else {
        stop();
        K_state = 0;
      }
    } else {
      // Non-neutral zone: prioritize left joystick, then right joystick
      // X-axis for left/right, Y-axis for forward/backward
      if (data.jl_x < 110) {
        left(motorSpeed);
        K_state = 3;
      } else if (data.jl_x > 115) {
        right(motorSpeed);
        K_state = 4;
      } else if (data.jl_y < 110) {
        backward(motorSpeed);
        K_state = 2;
      } else if (data.jl_y > 115) {
        forward(motorSpeed);
        K_state = 1;
      } else if (data.jr_x < 110) {
        left(motorSpeed);
        K_state = 3;
      } else if (data.jr_x > 115) {
        right(motorSpeed);
        K_state = 4;
      } else if (data.jr_y < 110) {
        backward(motorSpeed);
        K_state = 2;
      } else if (data.jr_y > 115) {
        forward(motorSpeed);
        K_state = 1;
      }
    }
  }
}

// Motor control functions
void forward(int motorSpeed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void backward(int motorSpeed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void left(int motorSpeed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void right(int motorSpeed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}