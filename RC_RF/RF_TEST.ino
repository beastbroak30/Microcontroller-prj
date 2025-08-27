#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin assignments
const int JL_X = A1;  // Left joystick X-axis
const int JL_Y = A2;  // Left joystick Y-axis
const int JR_X = A3;  // Right joystick X-axis
const int JR_Y = A4;  // Right joystick Y-axis
const int POT = A0;   // Potentiometer

// Variables to store readings
int jl_x_val, jl_y_val, jr_x_val, jr_y_val, pot_val;

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
} data;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  
  // Print header
  Serial.println("Transmitting Joystick & Potentiometer Values:");
  Serial.println("JL_X\tJL_Y\tJR_X\tJR_Y\tPOT");
}

void loop() {
  // Read all analog inputs
  jl_x_val = analogRead(JL_X);
  jl_y_val = analogRead(JL_Y);
  jr_x_val = analogRead(JR_X);
  jr_y_val = analogRead(JR_Y);
  pot_val = map(analogRead(POT), 0, 662, 0, 255);
  
  // Map values to byte range (0-255)
  data.jl_x = map(jl_x_val, 0, 1023, 0, 255);
  data.jl_y = map(jl_y_val, 0, 1023, 0, 255);
  data.jr_x = map(jr_x_val, 0, 1023, 0, 255);
  data.jr_y = map(jr_y_val, 0, 1023, 0, 255);
  data.pot = pot_val;
  
  // Send data
  radio.write(&data, sizeof(DataPacket));
  
  // Print values in a tab-separated format
  Serial.print(data.jl_x);
  Serial.print("\t");
  Serial.print(data.jl_y);
  Serial.print("\t");
  Serial.print(data.jr_x);
  Serial.print("\t");
  Serial.print(data.jr_y);
  Serial.print("\t");
  Serial.println(data.pot);
  
  // Small delay
  delay(100);
}