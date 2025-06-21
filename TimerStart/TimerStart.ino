#include "esp_camera.h"
//#include "FS.h"
#include "SD.h"
//#include "SPI.h"

#define LED_PIN 21

#define CAMERA_MODEL_XIAO_ESP32S3

#include "camera_pins.h"

bool camera_sign = false;          // Check camera status
bool sd_sign = false;              // Check sd status


void setup() {
  Serial.begin(115200);
  while(!Serial);

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
  config.frame_size = FRAMESIZE_QQVGA;
  config.pixel_format = PIXFORMAT_GRAYSCALE;  //for getting raw images not jpeg
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_DRAM;     //faster processing (CAMERA_FB_IN_PSRAM does not work and its because it is a lower processing speed)
  config.jpeg_quality = 12;
  config.fb_count = 2;                        //updated to 2 instead on 1

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  camera_sign = true;

  if(!SD.begin(21)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("SD Card initialized!");
  sd_sign = true;

  pinMode(LED_PIN, OUTPUT);
  
}

void loop() {
  static bool timerRunning = false;
  static unsigned long startTime = 0;
  static unsigned long endTime = 0;

  if (!(camera_sign && sd_sign)) return;

  //get first frame 
  camera_fb_t* fb1 = esp_camera_fb_get();
  if (!fb1) {
    Serial.println("Failed to capture frame 1.");
    delay(500);
    return;
  }

  delay(100);  //time between frames (to get a more significant change)

  //get second frame 
  camera_fb_t* fb2 = esp_camera_fb_get();
  if (!fb2) {
    Serial.println("Failed to capture frame 2.");
    esp_camera_fb_return(fb1);
    delay(500);
    return;
  }

  //comparing frames to get motion (subtracting pixels)
  int changedPixels = 0;
  int motionThreshold = 20;    //sensitivity    (from 0 - 255)
  int minChanged = 1000;       //how many pixels for change to be noticed 

  for (size_t i = 0; i < fb1->len && i < fb2->len; i++) {
    int diff = abs((int)fb1->buf[i] - (int)fb2->buf[i]);
    if (diff > motionThreshold){
      changedPixels++;
    }
  }

  if (changedPixels > minChanged) {
    Serial.println("Motion Detected!");
    if (!timerRunning) {
      //motion detected! start the timer
      startTime = millis();
      timerRunning = true;
      Serial.println("Timer started.");
      digitalWrite(LED_PIN, HIGH);
    } else {
      //motion detected! stop the timer and report
      endTime = millis();
      float elapsed_ms = endTime - startTime;
      float elapsed_s = elapsed_ms / 1000.0;
      float distance_m = 100.0;
      float speed = distance_m / elapsed_s;

      Serial.print("Timer stopped. Time elapsed: ");
      Serial.print(elapsed_ms);
      Serial.println(" ms");
      Serial.print("Speed: ");
      Serial.print(speed);
      Serial.println(" m/s");
      timerRunning = false;

      digitalWrite(LED_PIN, LOW);
    }

    delay(100); //delay if motion is detected so it doesnt double detect
  } else {
    Serial.println("No significant motion.");
  }

  esp_camera_fb_return(fb1);
  esp_camera_fb_return(fb2);

  delay(1); //decrease number for more set of frames 
}
