#include "esp_camera.h"
#include <WiFi.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "SD_MMC.h"

// Wi-Fi AP credentials
const char* ssid = "ESP32-CAM-AP";
const char* password = "12345678";

// Web server on port 80
AsyncWebServer server(80);

// Camera pin definitions for AI-Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Counter for JPEG filenames
int pictureNumber = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize camera
  if (!initCamera()) {
    Serial.println("Camera init failed");
    return;
  }

  // Initialize SD card
  if (!initSDCard()) {
    Serial.println("SD Card init failed");
  }

  // Start Wi-Fi as Access Point
  WiFi.softAP(ssid, password);
  Serial.println("AP Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", "<html><body><h1>ESP32-CAM Stream</h1><img src=\"/stream\" /></body></html>");
  });

  server.on("/stream", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponseStream("image/jpeg");
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      request->send(500, "text/plain", "Camera capture failed");
      return;
    }
    response->addHeader("Content-Disposition", "inline; filename=capture.jpg");
    response->write(fb->buf, fb->len);
    saveFrameToSD(fb); // Save the frame to SD
    esp_camera_fb_return(fb);
    request->send(response);
  });

  // Start server
  server.begin();
  Serial.println("Server started");
}

void loop() {
  delay(1000); // Adjust as needed
}

// Initialize the camera
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Set frame size and quality
  config.frame_size = FRAMESIZE_VGA; // 640x480
  config.jpeg_quality = 10; // 0-63, lower is higher quality
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }
  Serial.println("Camera initialized");
  return true;
}

// Initialize the SD card
bool initSDCard() {
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return false;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return false;
  }
  Serial.println("SD Card initialized");
  return true;
}

// Save frame to SD card
void saveFrameToSD(camera_fb_t * fb) {
  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("No SD Card available to save frame");
    return;
  }

  String path = "/picture" + String(pictureNumber) + ".jpg";
  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
  } else {
    file.write(fb->buf, fb->len);
    Serial.printf("Saved file to path: %s\n", path.c_str());
    file.close();
  }
  pictureNumber++;
}
