/*
   -- joystick --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.13 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.14.08 or later version;
     - for iOS 1.11.2 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__WIFI_POINT

#include <ESP8266WiFi.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "wallE2050"
#define REMOTEXY_WIFI_PASSWORD "103750ak"
#define REMOTEXY_SERVER_PORT 6377
#define REMOTEXY_ACCESS_PASSWORD "123456"


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 70 bytes
  { 255,5,0,0,0,63,0,18,0,0,0,25,2,106,200,200,84,1,1,4,
  0,4,78,69,7,86,177,5,13,68,0,103,28,5,205,33,143,143,12,19,
  60,60,1,103,26,31,4,82,59,7,86,89,7,12,77,32,77,26,4,60,
  33,7,157,96,68,76,14,160,77,26 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t ENABspeed; // from 0 to 100
  int8_t for_back; // from -100 to 100
  int8_t left_right; // from -100 to 100
  int8_t servo1; // from -100 to 100
  int8_t servo2; // from -100 to 100

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
// head control server 
WiFiServer server(80);
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////
// motor 
const int motorPin1 = D1;  // Replace with actual pin number
const int motorPin2 = D2;  // Replace with actual pin number
const int motorPin3 = D3;  // Replace with actual pin number
const int motorPin4 = D4;  // Replace with actual pin number
const int pwmPinA = D5;    // Replace with actual pin number
const int pwmPinB = D6;


void handleMovement() {
  int speed = map(RemoteXY.ENABspeed, 0, 100, 0, 255);  // Map speed from 0-100 to 0-255
  int for_back = RemoteXY.for_back;
  int left_right = RemoteXY.left_right;

  // Determine motor states based on joystick position
  if (for_back > 10) {  // RIGHT
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);

  } else if (for_back < -10) { 
    
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
     // LEFT


  } else if (left_right > 10) { 
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
     // FORWARD
  

    
  } else if (left_right < -10) { 
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
 // BACKWARD

   
  } else {
    // Stop all motors if joystick is in the center
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, LOW);
  }

  // Set motor speed based on slider value
  analogWrite(pwmPinA, speed);
  analogWrite(pwmPinB, speed);
}

void setup() 
{
  RemoteXY_Init ();
  server.begin();  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(pwmPinA, OUTPUT);
  pinMode(pwmPinB, OUTPUT);
  
  
  // TODO you setup code
  
}

void loop() 
{ 
  RemoteXY_Handler ();
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);   // Handle RemoteXY communication

  // Handle motor movement and speed
  handleMovement();
  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      // Send control data as simple text (e.g., "sl1:50,sl2:-30")
      String controlData = "sl1:" + String(RemoteXY.servo1) + ",sl2:" + String(RemoteXY.servo2);
      client.println(controlData);
  // Adjust as needed
    }
    client.stop();
  }
  
  
  // TODO you loop code
  // use the RemoteXY structure for data transfer
  // do not call delay(), use instead RemoteXY_delay() 


}