#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H

// ESP32-S3-DevKitC-1 (Freenove ESP32-S3-WROOM CAM)
// Custom Pinout provided by user

#define PWDN_GPIO_NUM   5    // 0-normal 1-sleep
#define RESET_GPIO_NUM  -1   // 0-reset 1-normal
#define XCLK_GPIO_NUM   41
#define SIOD_GPIO_NUM   1
#define SIOC_GPIO_NUM   2

#define Y9_GPIO_NUM     42
#define Y8_GPIO_NUM     47
#define Y7_GPIO_NUM     48
#define Y6_GPIO_NUM     20
#define Y5_GPIO_NUM     18
#define Y4_GPIO_NUM     16
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     19

#define VSYNC_GPIO_NUM  46
#define HREF_GPIO_NUM   45
#define PCLK_GPIO_NUM   21

#endif
