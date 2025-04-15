#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// DHT11 setup
#define DHTPIN D4         // GPIO2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// LCD setup (I2C address is usually 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2); // address, cols, rows

void setup() {
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  lcd.print("Checking sensor");
  // Initialize DHT11
  dht.begin();
  delay(1000);
  lcd.clear();
}

void loop() {
  // Read temperature
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity(); dht.

  // Check if reading was successful
  if (isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error    ");
  } else {
    Serial.print("Temp: ");
    Serial.print(temp);
    Serial.println(" *C");

    // Display temperature on LCD
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temp);
    lcd.print((char)223); // degree symbol
    lcd.print("C   ");
    lcd.setCursor(0,1);
    lcd.print("Humidity:");
    lcd.print(humidity);
    lcd.print("%");
  }

  delay(1000); //
}
