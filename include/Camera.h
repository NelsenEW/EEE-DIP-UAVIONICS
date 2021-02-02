/**
 * Camera for ESP32
 * @author  Nelsen Edbert Winata
 * @version Feb 2020
*/

#ifndef __camera_H__
#define __camera_H__

#include "Arduino.h"
#include "esp_camera.h"
#include <ESPAsyncWebServer.h>

// Select camera model
//#define CAMERA_MODEL_AI_THINKER
#define CAMERA_MODEL_NH_DRONE

#if defined(CAMERA_MODEL_AI_THINKER)
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

#elif defined(CAMERA_MODEL_NH_DRONE)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     12
#define SIOC_GPIO_NUM     14

#define Y9_GPIO_NUM       34
#define Y8_GPIO_NUM       39  // SENSOR_VN
#define Y7_GPIO_NUM       36  // SENSOR_VP
#define Y6_GPIO_NUM       15
#define Y5_GPIO_NUM        4
#define Y4_GPIO_NUM       17
#define Y3_GPIO_NUM       16
#define Y2_GPIO_NUM        2
#define VSYNC_GPIO_NUM    27
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM      5

#else
#error "Camera model not selected"
#endif

// Stream boundary
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: %s\r\nContent-Length: %u\r\n\r\n";
static const char * JPG_CONTENT_TYPE = "image/jpeg";

// Camera parameters
typedef struct {
  camera_fb_t * fb;
  size_t index;
} camera_frame_t;

class AsyncJpegStreamResponse: public AsyncAbstractResponse {
  private:
    camera_frame_t _frame;
    size_t _index;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    uint64_t lastAsyncRequest;
  public:
    AsyncJpegStreamResponse();
    ~AsyncJpegStreamResponse();
    bool _sourceValid() const;
    virtual size_t _fillBuffer(uint8_t *buf, size_t maxLen);
    size_t _content(uint8_t *buffer, size_t maxLen, size_t index);
};

void streamJpg(AsyncWebServerRequest *request);
void cameraInit();

#endif
