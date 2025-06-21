#include <esp_camera.h>

#define LED_PIN 21 //visual (on when timer is running)

//for handeling the two FreeRTOS tasks and a queue for inter task communication
TaskHandle_t cameraTaskHandle;    //"pointer" reference to a running thread
TaskHandle_t timerTaskHandle;
QueueHandle_t motionQueue;

bool timerRunning = false;    //flag
unsigned long startTime = 0;  

#define CAMERA_MODEL_XIAO_ESP32S3
#include "camera_pins.h"

void initCamera() {
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
  config.pixel_format = PIXFORMAT_GRAYSCALE;  //for getting raw images not jpeg
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_DRAM;     //faster processing (CAMERA_FB_IN_PSRAM does not work and its because it is a lower processing speed)
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    while (true);
  }
}

//get a new frame from camera every 30 ms
void CameraTask(void *pvParameters) {
  while (true) {
    
    camera_fb_t* fb1 = esp_camera_fb_get();
    if (!fb1) {
      Serial.println("Failed to capture frame 1.");
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    camera_fb_t* fb2 = esp_camera_fb_get();
    if (!fb2) {
      Serial.println("Failed to capture frame 2.");
      esp_camera_fb_return(fb1);
    }

    //comparing frames to get motion (subtracting pixels)
    int changedPixels = 0;
    int motionThreshold = 20;    //sensitivity    (from 0 - 255)
    int minChanged = 1000;       //how many pixels for change to be noticed 

    //calculate pixel-wise difference
    for (size_t i = 0; i < fb1->len && i < fb2->len; i++) {
    int diff = abs((int)fb1->buf[i] - (int)fb2->buf[i]);
      if (diff > motionThreshold){
        changedPixels++;
      }
  }

    if (changedPixels > minChanged) {
      Serial.println("Motion Detected!");
      int signal = 1;
      xQueueSend(motionQueue, &signal, portMAX_DELAY);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
   
    esp_camera_fb_return(fb1);
    esp_camera_fb_return(fb2);

    //vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

//waits for "motion detected"
void TimerTask(void *pvParameters) {
  int signal;
  while (true) {
    if (xQueueReceive(motionQueue, &signal, portMAX_DELAY) == pdTRUE) {  //(queue, buffer, tickstoW8)
      if (!timerRunning) {
        startTime = millis();
        timerRunning = true;
        digitalWrite(LED_PIN, LOW);
        Serial.println("Timer started");
      } else {
        unsigned long elapsed = millis() - startTime;
        timerRunning = false;
        digitalWrite(LED_PIN, HIGH);
        Serial.printf("Elapsed time: %lu ms\n", elapsed);
      }
    }
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  initCamera();

  motionQueue = xQueueCreate(5, sizeof(int));
  Serial.println("motion queue created");

  xTaskCreatePinnedToCore(CameraTask, "CameraTask", 4096, NULL, 1, &cameraTaskHandle, 1); //(function, name, size (16kb), parameters, proority, pointer, core)
  xTaskCreatePinnedToCore(TimerTask, "TimerTask", 2048, NULL, 1, &timerTaskHandle, 0);
}

void loop() {
}
