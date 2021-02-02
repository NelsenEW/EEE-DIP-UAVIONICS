/**
 * Asynchronous Camera streamer for ESP32 NH board
 * @author  Nelsen Edbert Winata
 * @version Feb 2020
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "Camera.h"

/******************** NH Drone Pinout ***********************/
// LED pin. NOTE - HIGH means the led is off, LOW means led is on.
#define blueLed 13
#define redLed 33

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
  // Turn on red led, and turn off blue led as running mode indicator
  pinMode(redLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  digitalWrite(redLed, LOW);
  digitalWrite(blueLed, HIGH);
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
