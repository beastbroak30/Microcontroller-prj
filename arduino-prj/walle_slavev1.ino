/*
   -- Wemos D1 Mini as Wi-Fi Client with Servo Control --
*/

#include <ESP8266WiFi.h>
#include <Servo.h>

const char* ssid = "wallE2050";     
const char* password = "103750ak";  

const char* serverIP = "192.168.4.1";  
const int serverPort = 80;       //server on 80

WiFiClient client;

Servo servo1;
Servo servo2;

#define SERVO1_PIN D5
#define SERVO2_PIN D6

void setup() {

  WiFi.begin(ssid, password);


  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }

  
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  

  servo1.write(90);  
  servo2.write(90);  
}

void loop() {
  if (client.connect(serverIP, serverPort)) {
    String line = client.readStringUntil('\n');  


    int sl1_start = line.indexOf("sl1:") + 4;
    int sl1_end = line.indexOf(",sl2:");
    int sl2_start = sl1_end + 5;

    if (sl1_start > 0 && sl1_end > 0 && sl2_start > 0) {
      int sl1_value = line.substring(sl1_start, sl1_end).toInt();
      int sl2_value = line.substring(sl2_start).toInt();

      int servo1_angle = map(sl1_value, -100, 100, 0, 180);
      int servo2_angle = map(sl2_value, -100, 100, 180, 0);

      // Set servo positions
      servo1.write(servo1_angle);
      servo2.write(servo2_angle);
    }
    client.stop();  
  }


}
