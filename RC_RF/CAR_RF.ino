#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// RF24 radio setup
RF24 radio(9, 8); // CE, CSN pins
const byte address[6] = "00001";

// Data structure to receive
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
  }
}