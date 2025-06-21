# ESP32-S3 Motion Detection

A motion detection system using the ESP32-S3 Xiao-Sense. It compares grayscale frames to detect motion and trigger actions.

## Two Versions

- [`FreeRTOS_Integration`](https://github.com/cllonn/esp32s3-motionDetection/tree/master/FreeRTOS_Integration): uses FreeRTOS tasks for better multitasking.
- [`TimerStart`](https://github.com/cllonn/esp32s3-motionDetection/tree/master/TimerStart): simple Arduino-style code without RTOS.

## ⚙Features

- Frame differencing in grayscale
- Adjustable motion threshold
- Optional timer or LED trigger

## Tools

- ESP32-S3 + camera (e.g. OV2640)
- Arduino or PlatformIO
- `esp_camera.h`

## Author

Sara Almarzooqi  
