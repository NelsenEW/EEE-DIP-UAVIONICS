/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor
  This example shows how to take Sensor Events instead of direct readings
  
  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried& Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BMP280.h"

#define I2C_SDA 14
#define I2C_SCL 15

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

sensors_event_t temp_event, pressure_event;
float avg_pressure = 0;
float new_pressure, iir_pressure = 0;
const float alpha = 0.03;

void setup() {
  Serial.begin(115200);
  Serial.println(F("BMP280 Sensor event test"));
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  bmp_temp->printSensorDetails();

  // Get average value.
  for (int i = 0; i < 500; i++){
    bmp_pressure->getEvent(&pressure_event);
    avg_pressure += pressure_event.pressure;
    delay(20);
  }
  avg_pressure /= 500;
}

void loop() {
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  Serial.println();

  delay(2000);

  new_pressure = 100*(pressure_event.pressure - avg_pressure);
  iir_pressure = (1-alpha)*iir_pressure + alpha*new_pressure;
  Serial.print( new_pressure );
  Serial.print(" ");
  Serial.println( iir_pressure );

  delay(50);
}
