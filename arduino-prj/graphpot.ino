#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Potentiometer pins (ADC1)
int pot1Pin = 39; // Pin for Potentiometer 1 (ADC1_6)
int pot2Pin = 36; // Pin for Potentiometer 2 (ADC1_7)
int pot3Pin = 25; // Pin for Potentiometer 3 (ADC1_8)

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);

  // Initialize I2C Communication
  Wire.begin(5, 4);  // SDA = 5, SCL = 4

  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);  // Halt if display initialization fails
  }

  // Delay to ensure the display is ready
  delay(2000);

  // Clear the display buffer
  display.clearDisplay();
}

void loop() {
  // Read potentiometer values (0-4095 for ADC1)
  int pot1Value = analogRead(pot1Pin);
  int pot2Value = analogRead(pot2Pin);
  int pot3Value = analogRead(pot3Pin);

  // Clear the display before drawing new data
  display.clearDisplay();

  // Display potentiometer values on the OLED without mapping
  display.setTextSize(1); // Set text size
  display.setTextColor(SSD1306_WHITE); // Set text color
  display.setCursor(0, 0); // Set cursor to top left
  display.print("Pot1: ");
  display.println(pot1Value);
  
  display.setCursor(0, 10); // Move cursor down
  display.print("Pot2: ");
  display.println(pot2Value);

  display.setCursor(0, 20); // Move cursor further down
  display.print("Pot3: ");
  display.println(pot3Value);

  // Update the display with the new values
  display.display();

  // Output the potentiometer values to the Serial Monitor for plotting
  Serial.print("Pot1: ");
  Serial.print(pot1Value);
  Serial.print("\tPot2: ");
  Serial.print(pot2Value);
  Serial.print("\tPot3: ");
  Serial.println(pot3Value);

  // Small delay for better readability on display
  delay(500);
}