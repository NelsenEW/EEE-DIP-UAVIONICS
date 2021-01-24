/**
 * Camera ESP32 Asynchronous Web Server
 * @author  Nelsen Edbert Winata
 * @version Jan 2020
 * @brief   Demonstrate asynchronous camera streaming with TCP stack on ESP32-CAM AI thinker board.
*/

#include "Camera.h"
#include "Arduino.h"

/**
 * @brief  Constructor for AsyncJpegStreamResponse
 */
AsyncJpegStreamResponse::AsyncJpegStreamResponse() {
  _callback = nullptr;
  _code = 200;
  _contentLength = 0;
  _contentType = STREAM_CONTENT_TYPE;
  _sendContentLength = false;
  _chunked = true;
  _index = 0;
  _jpg_buf_len = 0;
  _jpg_buf = NULL;
  lastAsyncRequest = 0;
  memset(&_frame, 0, sizeof(camera_frame_t));
}

/**
 * @brief  Destructor for AsyncJpegStreamResponse
 */
AsyncJpegStreamResponse::~AsyncJpegStreamResponse() {
  if (_frame.fb) {
    if (_frame.fb->format != PIXFORMAT_JPEG) {
      free(_jpg_buf);
    }
    esp_camera_fb_return(_frame.fb);
  }
}

bool AsyncJpegStreamResponse::_sourceValid() const {
  return true;
}

/**
 * @brief  This function return the size of the buffer and update the index
 * @note   The method overrides the method from the parent class, AsyncAbstractResponse
 * @param  buf: buffer for image
 * @param  maxLen: maximum length for header and image
 * @retval size of the content in bytes
 */
size_t AsyncJpegStreamResponse::_fillBuffer(uint8_t *buf, size_t maxLen) {
  size_t ret = _content(buf, maxLen, _index);
  if (ret != RESPONSE_TRY_AGAIN) {
    _index += ret;
  }
  return ret;
}

/**
 * @brief  This function returns the size of the content
 * @param  buffer: buffer for image
 * @param  maxLen: maximum length for header and image
 * @param  index : index of frame
 * @retval size of the content in bytes
 */
size_t AsyncJpegStreamResponse::_content(uint8_t *buffer, size_t maxLen, size_t index) {
  if (!_frame.fb || _frame.index == _jpg_buf_len) {
    if (index && _frame.fb) {
      uint64_t end = (uint64_t)micros();
      int fp = (end - lastAsyncRequest) / 1000;
      log_printf("Size: %uKB, Time: %ums \n", _jpg_buf_len / 1024, fp);
      lastAsyncRequest = end;
      if (_frame.fb->format != PIXFORMAT_JPEG) {
        free(_jpg_buf);
      }
      esp_camera_fb_return(_frame.fb);
      _frame.fb = NULL;
      _jpg_buf_len = 0;
      _jpg_buf = NULL;
    }
    if (maxLen < (strlen(STREAM_BOUNDARY) + strlen(STREAM_PART) + strlen(JPG_CONTENT_TYPE) + 8)) {
      //log_w("Not enough space for headers");
      return RESPONSE_TRY_AGAIN;
    }
    //get frame
    _frame.index = 0;

    _frame.fb = esp_camera_fb_get();
    if (_frame.fb == NULL) {
      log_e("Camera frame failed");
      return 0;
    }

    if (_frame.fb->format != PIXFORMAT_JPEG) {
      unsigned long st = millis();
      bool jpeg_converted = frame2jpg(_frame.fb, 80, &_jpg_buf, &_jpg_buf_len);
      if (!jpeg_converted) {
        log_e("JPEG compression failed");
        esp_camera_fb_return(_frame.fb);
        _frame.fb = NULL;
        _jpg_buf_len = 0;
        _jpg_buf = NULL;
        return 0;
      }
      log_i("JPEG: %lums, %uB", millis() - st, _jpg_buf_len);
    } else {
      _jpg_buf_len = _frame.fb->len;
      _jpg_buf = _frame.fb->buf;
    }

    //send boundary
    size_t blen = 0;
    if (index) {
      blen = strlen(STREAM_BOUNDARY);
      memcpy(buffer, STREAM_BOUNDARY, blen);
      buffer += blen;
    }
    //send header
    size_t hlen = sprintf((char *)buffer, STREAM_PART, JPG_CONTENT_TYPE, _jpg_buf_len);
    buffer += hlen;
    //send frame
    hlen = maxLen - hlen - blen;
    if (hlen > _jpg_buf_len) {
      maxLen -= hlen - _jpg_buf_len;
      hlen = _jpg_buf_len;
    }
    memcpy(buffer, _jpg_buf, hlen);
    _frame.index += hlen;
    return maxLen;
  }

  size_t available = _jpg_buf_len - _frame.index;
  if (maxLen > available) {
    maxLen = available;
  }
  memcpy(buffer, _jpg_buf + _frame.index, maxLen);
  _frame.index += maxLen;

  return maxLen;
}

/**
 * @brief   This function streams JPG to web server
 * @note    Create Async response object to control JPG streaming
 * @param   request: Request from Asynchronous web server
 */
void streamJpg(AsyncWebServerRequest *request) {
  AsyncJpegStreamResponse *response = new AsyncJpegStreamResponse();
  if (!response) {
    request->send(501);
    return;
  }
  response->addHeader("Access-Control-Allow-Origin", "*");
  request->send(response);
}

/**
 * @brief  This function is used to initialize camera
 * @note   Camera pin definition is defined on Camera.h
 */
void cameraInit() {
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
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();

  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);
}
