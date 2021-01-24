#include "Camera.h"
#include <WiFi.h>
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include <ESPAsyncWebServer.h>

// WIFI Station vs Software Access Point
#define SOFTAP_MODE // Enable onboard ESP AP

#ifdef SOFTAP_MODE
const char *ssid = "espcam";                // Put your SSID here
const char *password = "12345678";          // Put your PASSWORD here
IPAddress apIP = IPAddress(192, 168, 1, 1); // IP address
#else
const char *ssid = "********";              // Put your SSID here
const char *password = "********";          // Put your PASSWORD here
// IP address of ESP32 is automatically configured with DHCP from the Wifi router
#endif

AsyncWebServer server(80);

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  cameraInit();
  sensor_t * s = esp_camera_sensor_get();
  
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  // WiFi setup
  IPAddress ip;
#ifdef SOFTAP_MODE
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
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  ip = WiFi.localIP();
  Serial.println("");
  Serial.println("WiFi connected");
#endif
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(ip);
  Serial.println("/stream.jpg' to connect");
  server.on("/stream.jpg", HTTP_GET, streamJpg);
  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000);
}
