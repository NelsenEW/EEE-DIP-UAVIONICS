#include "Camera.h"
#include <WiFi.h>
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include <ESPAsyncWebServer.h>

const char* ssid = "espcam";
const char* password = "12345678";
//
IPAddress apIP = IPAddress(192, 168, 1, 1);

AsyncWebServer server(80);

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  IPAddress ip;
  cameraInit();
  sensor_t * s = esp_camera_sensor_get();
  
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  
  WiFi.mode(WIFI_AP);
  
  bool result = WiFi.softAP(ssid, password, 1, 0);
  delay(2000);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

  if (!result)
  {
    Serial.println("AP Config failed.");
    return;
  }
  else
  {
    Serial.println("AP Config Success.");
    Serial.print("AP MAC: ");
    Serial.println(WiFi.softAPmacAddress());

    ip = WiFi.softAPIP();
  }

  server.on("/stream.jpg", HTTP_GET, streamJpg);
  server.begin();


  Serial.print("Camera Ready! Use 'http://");
  Serial.print(ip);
  Serial.println("' to connect");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000);
}
