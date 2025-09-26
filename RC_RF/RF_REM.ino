#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin assignments
const int JL_X = A1;  // Left joystick X-axis
const int JL_Y = A2;  // Left joystick Y-axis
const int JR_X = A3;  // Right joystick X-axis
const int JR_Y = A4;  // Right joystick Y-axis
const int POT = A0;   // Potentiometer 1
const int POTC = A7;  // Potentiometer 2
const int bt_pin = 3; // Tactile button

// Variables for button debouncing
int buttonState = HIGH;         // Current button state (HIGH = not pressed, LOW = pressed)
int lastButtonState = HIGH;     // Previous button state
unsigned long lastDebounceTime = 0;  // Last time the button state changed
const unsigned long debounceDelay = 1; // Debounce time in ms
int clawState = 0; // 0 = Claw Deactivated, 1 = Claw Activated

// RF24 radio setup
RF24 radio(9, 8); // CE, CSN pins
const byte address[6] = "00001";

// Data structure to send
struct DataPacket {
  byte jl_x;
  byte jl_y;
  byte jr_x;
  byte jr_y;
  byte pot;
  byte potc;
  byte claw; // Claw state (0 or 1)
} data;

void setup() {
  // Initialize Serial at 9600 baud
  Serial.begin(9600);
  
  // Set button pin as input with internal pull-up resistor
  pinMode(bt_pin, INPUT_PULLUP);
  
  // Initialize radio
  if (!radio.begin()) {
    Serial.println("Radio initialization failed!");
    while (1); // Halt if radio fails
  }
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  
  // Set ADC reference to 3.3V
  analogReference(DEFAULT);
  
  // Print header
  Serial.println("Transmitting Joystick, Potentiometer & Button Values:");
  Serial.println("JL_X\tJL_Y\tJR_X\tJR_Y\tPOT\tPOTC\tClaw");
  // Print initial claw state
  Serial.print("\t\t\t\t\t\t");
  Serial.println(clawState);
}

void loop() {
  // Read all analog inputs
  int jl_x_val = analogRead(JL_X);
  int jl_y_val = analogRead(JL_Y);
  int jr_x_val = analogRead(JR_X);
  int jr_y_val = analogRead(JR_Y);
  int pot_val = analogRead(POT);
  int potc_val = analogRead(POTC);
  
  // Read button state
  int reading = digitalRead(bt_pin);
  
  // Debounce button
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) { // Toggle on press
        clawState = !clawState; // Toggle between 0 and 1
      }
    }
  }
  lastButtonState = reading;
  
  // Map values to byte range (0-255)
  data.jl_x = map(jl_x_val, 0, 1023, 0, 255);
  data.jl_y = map(jl_y_val, 0, 1023, 0, 255);
  data.jr_x = map(jr_x_val, 0, 1023, 0, 255);
  data.jr_y = map(jr_y_val, 0, 1023, 0, 255);
  data.pot = map(pot_val, 0, 660, 0, 255); // Matches 2.13V limit
  data.potc = map(potc_val, 0, 660, 0, 255);
  data.claw = clawState;
  
  // Send data via RF
  radio.write(&data, sizeof(DataPacket));
  
  // Print values in tab-separated format continuously
  Serial.print(data.jl_x);
  Serial.print("\t");
  Serial.print(data.jl_y);
  Serial.print("\t");
  Serial.print(data.jr_x);
  Serial.print("\t");
  Serial.print(data.jr_y);
  Serial.print("\t");
  Serial.print(data.pot);
  Serial.print("\t");
  Serial.print(data.potc);
  Serial.print("\t");
  Serial.println(data.claw);
  
  // Delay for stability
  delay(10);
}