/**
   Asynchronous Sensor Visualization
   @author Nelsen Edbert Winata
   @version Jan 2020
*/

#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Ticker.h>
#include "MPU9250.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_Sensor.h"
#include <Arduino_JSON.h>
#include "SPIFFS.h"

#define DEBUG 1

// I2C Pin definition
#define I2C_SDA 14
#define I2C_SCL 15

// WIFI Station vs Software Access Point
#define SOFTAP_MODE // Enable onboard ESP AP

#ifdef SOFTAP_MODE
const char *ssid = "espcam";                // Put your SSID here
const char *password = "12345678";          // Put your PASSWORD here
IPAddress apIP = IPAddress(192, 168, 1, 1); // IP address
#else
const char *ssid = "**********";              // Put your SSID here
const char *password = "**********";          // Put your PASSWORD here
// IP address of ESP32 is automatically configured with DHCP from the Wifi router
#endif

// BMP280 Initialization
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temperature = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
float bmpTemperature = 0, bmpPressure = 0, avgPressure = 0, height = 0; // Measurement from ESP32
sensors_event_t temperature_event, pressure_event;

// MPU9250 Configuration
// Specify sensor full scale
/* Choices are:
    Gscale: GFS_250 == 250 dps, GFS_500 DPS == 500 dps, GFS_1000 == 1000 dps, and GFS_2000DPS == 2000 degrees per second gyro full scale
    Ascale: AFS_2G == 2 g, AFS_4G == 4 g, AFS_8G == 8 g, and AFS_16G == 16 g accelerometer full scale
    Mscale: MFS_14BITS == 0.6 mG per LSB and MFS_16BITS == 0.15 mG per LSB
    Mmode: Mmode == M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
    (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so
    sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
*/
const uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_8Hz, sampleRate = 0x04;
const uint8_t samplingRate = Mmode == M_8Hz ? 8 : 100;
const float pi = 3.1415926535897f;
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float motion = 0; // check on linear acceleration to determine motion
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
const float GyroMeasError = pi * (0.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 0 deg/s)
const float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
const float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
const float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
const float Kp = 2.0f * 5.0f; // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
const float Ki = 0.0f;

int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float   magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float   temperature;    // Stores the MPU9250 internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// These can be measured once and entered here or can be calculated each time the device is powered on
float   gyroBias[3] = { -23.56, 9.22, -5.31}, accelBias[3] = {1.66, -2.37, 0.53};
float   magBias[3] = {67.47, 472.31, -103.46}, magScale[3]  = {1.17, 1.30, 0.73}; // Bias corrections for gyro and accelerometer

uint32_t delt_t;// used to control display output rate
uint32_t count = 0, sumCount = 0;         // used to control display output rate
float pitch, yaw, roll; // absolute orientation
float pitchBias = 0, yawBias = 0, rollBias = 0;

float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0;                  // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float rate;                               // rate of the sensor update

#define CALIBRATING             // Uncomment when calibration is required
#define MADGWICK_WITH_MAGNETO  // Uncomment Madgwick sensor fusion implementation with 9 DOF instead of 6 DOF

MPU9250 MPU9250; // instantiate MPU9250 class

// Sampling MPU9250 every 10ms period (100 Hz)
Ticker sampler;
const float samplingPeriod = 0.1;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

#ifdef MADGWICK_WITH_MAGNETO
__attribute__((optimize("O3"))) void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

#else
__attribute__((optimize("O3"))) void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  gbiasx += gerrx * deltat * zeta;
  gbiasy += gerry * deltat * zeta;
  gbiasz += gerrz * deltat * zeta;
  gx -= gbiasx;
  gy -= gbiasy;
  gz -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 - (beta * hatDot1)) * deltat;
  q2 += (qDot2 - (beta * hatDot2)) * deltat;
  q3 += (qDot3 - (beta * hatDot3)) * deltat;
  q4 += (qDot4 - (beta * hatDot4)) * deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
#endif


void samplingMPU9250() {
  bmp_pressure->getEvent(&pressure_event);
  bmp_temperature->getEvent(&temperature_event);
  MPU9250.readMPU9250Data(MPU, MPU9250Data);

  // Now we'll calculate the accleration value into actual g's
  ax = (float)MPU9250Data[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
  ay = (float)MPU9250Data[1] * aRes - accelBias[1];
  az = (float)MPU9250Data[2] * aRes - accelBias[2];

  // Calculate the gyro value into actual degrees per second
  gx = (float)MPU9250Data[4] * gRes; // get actual gyro value, this depends on scale being set
  gy = (float)MPU9250Data[5] * gRes;
  gz = (float)MPU9250Data[6] * gRes;

  MPU9250.readMagData(MPU, magCount);  // Read the x/y/z adc values

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
  my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];
  mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];
  mx *= magScale[0];
  my *= magScale[1];
  mz *= magScale[2];


  for (uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;
#ifdef MADGWICK_WITH_MAGNETO
    MadgwickQuaternionUpdate(-ax, +ay, +az, gx * pi / 180.0f, -gy * pi / 180.0f, -gz * pi / 180.0f,  my,  -mx, mz);
#else
    MadgwickQuaternionUpdate(-ax, +ay, +az, gx * pi / 180.0f, -gy * pi / 180.0f, -gz * pi / 180.0f);
#endif
  }

  if (DEBUG) {
    Serial.print("ax = "); Serial.print((int)1000 * ax);
    Serial.print(" ay = "); Serial.print((int)1000 * ay);
    Serial.print(" az = "); Serial.print((int)1000 * az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2);
    Serial.print(" gy = "); Serial.print( gy, 2);
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)mx );
    Serial.print(" my = "); Serial.print( (int)my );
    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
    //    Serial.print("q0 = "); Serial.print(q[0]);
    //    Serial.print(" qx = "); Serial.print(q[1]);
    //    Serial.print(" qy = "); Serial.print(q[2]);
    //    Serial.print(" qz = "); Serial.println(q[3]);
    //
    //    temperature = ((float) MPU9250Data[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade
    //    // Print temperature in degrees Centigrade
    //    Serial.print("MPU temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
  }

  a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
  a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
  a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
  a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  pitch = -asinf(a32);
  roll  = atan2f(a31, a33);
  yaw   = atan2f(a12, a22);
  pitch = pitch * (180.0f / pi) - pitchBias;
  yaw   = yaw * (180.0f / pi) - yawBias;
  //  yaw   += 0.13f; // Declination at Singapore is 13 degrees
  if (yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
  roll  = roll * (180.0f / pi) - rollBias;
  lin_ax = ax + a31;
  lin_ay = ay + a32;
  lin_az = az - a33;

  if (DEBUG) {
    Serial.print("MPU9250 Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    //    Serial.print("Grav_x, Grav_y, Grav_z: ");
    //    Serial.print(-a31 * 1000.0f, 2);
    //    Serial.print(", ");
    //    Serial.print(-a32 * 1000.0f, 2);
    //    Serial.print(", ");
    //    Serial.print(a33 * 1000.0f, 2);  Serial.println(" mg");
    //    Serial.print("Lin_ax, Lin_ay, Lin_az: ");
    //    Serial.print(lin_ax * 1000.0f, 2);
    //    Serial.print(", ");
    //    Serial.print(lin_ay * 1000.0f, 2);
    //    Serial.print(", ");
    //    Serial.print(lin_az * 1000.0f, 2);  Serial.println(" mg");
    rate = (float)sumCount / sum;
    Serial.print("rate = "); Serial.print(rate, 2); Serial.println(" Hz");
    sumCount = 0;
    sum = 0;
  }

  bmpPressure = pressure_event.pressure;
  bmpTemperature = temperature_event.temperature;
  height = - 8.6 * (bmpPressure - avgPressure);

  if (DEBUG) {
    Serial.print("BMP280 Temperature, Pressure, Height: ");
    Serial.print(bmpTemperature, 2);
    Serial.print(", ");
    Serial.print(bmpPressure, 2); // in HPa rather than Pa
    Serial.print(", ");
    Serial.println(height);
  }
  events.send(getAllReadings().c_str(), "all_readings", millis());
}

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

String readSensor() {
  String sensor = String(bmp.readTemperature()) + "\n" + String(bmp.readPressure() / 100.0F) + "\n" + String(height)
                  + "\n" + String((int)(ax * 1000)) + "\n" + String((int)(ay * 1000)) + "\n" + String((int)(az * 1000)) + "\n" + String(gx) + "\n"
                  + String(gy) + "\n" + String(gz) + "\n" + String(mx) + "\n" + String(my) + "\n" + String(mz)
                  + "\n" + String(pitch) + "\n" + String(roll) + "\n" + String(yaw) + "\n" + String(rate);
  return sensor;
}

String getAllReadings() {
  readings["accX"] = String((int)1000 * ax);
  readings["accY"] = String((int)1000 * ay);
  readings["accZ"] = String((int)1000 * az);
  readings["gyroX"] = String(round(gx * 100) / 100.0);
  readings["gyroY"] = String(round(gy * 100) / 100.0);
  readings["gyroZ"] = String(round(gz * 100) / 100.0);
  readings["magX"] = String((int) mx);
  readings["magY"] = String((int) my);
  readings["magZ"] = String((int) mz);
  readings["roll"] = String(round(roll * 100) / 100.0);
  readings["pitch"] = String(round(pitch * 100) / 100.0);
  readings["yaw"] = String(round(yaw * 100) / 100.0);
  readings["temp"] = String(round(bmpTemperature * 100) / 100.0);
  readings["pres"] = String(round(bmpPressure * 100) / 100.0);

  String allString = JSON.stringify (readings);
  return allString;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  /*********************** Serial Initialization ***********************/
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  /*********************** SPIFFS Initialization ***********************/
  initSPIFFS();

  /*********************** WiFi Setup ***********************/
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

  /*********************** Sensor Initialization ***********************/
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  //MPU9250.I2Cscan(); // should detect BMP280 at 0x76, MPU9250 at 0x71
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  delay(500);

  Serial.println("MPU9250 9-axis motion sensor...");
  uint8_t addr = MPU9250.getMPU9250ID(MPU);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(addr, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  delay(1000);

  // Connection
  if (addr == 0x71) // WHO_AM_I should always be 0x71 for MPU9250
  {
    Serial.println("MPU9250 is online...");

    MPU9250.resetMPU9250(MPU); // start by resetting MPU9250

    MPU9250.SelfTest(MPU, SelfTest); // Start by performing self test and reporting values
    Serial.println("Self Test for MPU9250 #1:");
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");
    delay(1000);

    // get sensor resolutions, only need to do this once, same for both MPU9250s for now
    aRes = MPU9250.getAres(Ascale);
    gRes = MPU9250.getGres(Gscale);
    mRes = MPU9250.getMres(Mscale);

#ifdef CALIBRATING
    MPU9250.calibrateMPU9250(MPU, gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    Serial.println("MPU accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
    Serial.println("MPU gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
    delay(1000);
#endif

    MPU9250.initMPU9250(MPU, Ascale, Gscale, sampleRate);
    Serial.println("MPU9250 is initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    addr = MPU9250.getAK8963CID(MPU);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 1 "); Serial.print("I AM "); Serial.print(addr, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    MPU9250.initAK8963Slave(MPU, Mscale, Mmode, magCalibration); Serial.println("AK8963 is initialized for active data mode...."); // Initialize device 1 for active mode read of magnetometer
    Serial.println("Calibration values for mag: ");
    Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);

#ifdef CALIBRATING
    MPU9250.magcalMPU9250(MPU, magBias, magScale);
    Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
    Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]);
    delay(1000); // add delay to see results before serial spew of data
#endif
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x"); Serial.println(addr, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  delay(3000);

  Serial.println("Pressure averaging, do not move or touch the sensors");

  // Get average value.
  for (int i = 0; i < 500; i++) {
    bmp_pressure->getEvent(&pressure_event);
    avgPressure += pressure_event.pressure;
    delay(20);
  }
  avgPressure /= 500;

  Serial.print("Average Pressure as bias: ");   Serial.println(avgPressure);


  Serial.print("Visualization Ready! Use 'http://");
  Serial.print(ip);
  Serial.println("' to read");
  Serial.print("Sensor Ready! Use 'http://");
  Serial.print(ip);
  Serial.println("/sensor' to read");
  delay(2000);
  /*********************** Ticker Sampler Initialization ***********************/
  sampler.attach(samplingPeriod, samplingMPU9250);

  /*********************** Server Initialization ***********************/

  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest * request) {
    rollBias += roll;
    pitchBias += pitch;
    yawBias += yaw;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest * request) {
    rollBias += roll;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest * request) {
    pitchBias += pitch;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest * request) {
    yawBias += yaw;
    request->send(200, "text/plain", "OK");
  });

  server.on("/sensor", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readSensor().c_str());
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient * client) {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "Connected!", id current millis
    // and set reconnect delay to 1 second
    client->send("Connected!", NULL, millis(), 10000);
  });

  server.addHandler(&events);
  server.begin();
}

void loop() { // Implement your control algorithm here
  delay(10000);
}
