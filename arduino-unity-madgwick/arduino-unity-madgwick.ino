#include <TaskScheduler.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Wire.h>

// #include "MadgwickAHRS.h"
#include "Adafruit_AHRS.h"
#include "Adafruit_Sensor_Calibration.h"

// for Madgwick setup
// Madgwick filter;
Adafruit_Madgwick filter;
const float sensorRate = 104.00;


// params for the sensor data acquisition
unsigned int GyroFS_dps = 2000;
unsigned int AccFS_g = 16;
unsigned int MagFS_uT = 400;
unsigned int GyroFS_dps_BuitIn = 2000;
unsigned int AccFS_g_BuitIn = 16;
unsigned int MagFS_uT_BuitIn = 400;

// Calibration offsets - from sensor_calibration_read (Adafruit)
// used in applyCalibration() function below
/**! XYZ vector of offsets for zero-g, in m/s^2 */
float accel_zerog[3] = {-0.15, -0.2, 0.2};
/**! XYZ vector of offsets for zero-rate, in rad/s */
float gyro_zerorate[3] = {0.0104, 0.0098, 0.0000};
/**! XYZ vector of offsets for hard iron calibration (in uT) */
float mag_hardiron[3] = {-1633.83, 53.19, -590.69};
/**! The 3x3 matrix for soft-iron calibration (unitless) */
float mag_softiron[9] = {0.28361, -0.00277, -0.00111, -0.00277, 0.27285, 0.00045, -0.00111, 0.00045, 0.24917};
/**! The magnetic field magnitude in uTesla */
float mag_field = 50;
////

// the I2C addresses of the sensors when correct PCA port selected
// PCA 1: Adr0 is left ear, Adr 1 is right ear
// PCA2: Adr 0 is chest, Adr 1 is (left) leg
uint8_t j = 0;
uint8_t I2C_Adr0 = 0X6B; //0X6A
uint8_t I2C_Adr1 = 0X6B;
uint8_t I2C_MagAdr0 = 0X1E; //0X1C
uint8_t I2C_MagAdr1 = 0X1E;

// declare the sensors (acc/gyr and mag for each of the 4)
// A = left ear, B = right ear, C = chest, D = leg
Adafruit_LSM6DSOX soxA;
Adafruit_LIS3MDL lis3mdlA;
float gx, gy, gz;

// I2C TwoWire to allow read access to data
TwoWire I2C_0 = TwoWire(0);
#define I2C_Freq 100000
#define SDA_0 23
#define SCL_0 22

// for mux scanning *******
#define PCAADDR 0x70

void pcaselect(uint8_t i) {
  if (i > 3) return;
 
  I2C_0.beginTransmission(PCAADDR);
  I2C_0.write(1 << i);
  I2C_0.endTransmission();  
}
//*************************

// Callback methods prototypes
void t2Callback();

//Tasks (delay_ms,times,func)
Task t2(1, TASK_FOREVER, &t2Callback);

Scheduler runner;
int t2_Counter = 0;
unsigned long FrameCounter = 0x00000000;   //Data frame number
float EEG_A0[5], EEG_A1[5];
float Accx, Accy, Accz, Gyrox, Gyroy, Gyroz, Magx, Magy, Magz;
uint8_t Send_buff[152];     // 声明一个用来存储 4 字节的字节数组


void t2Callback() 
{
  FrameCounter ++;
  /* Get a new normalized sensor event */
  sensors_event_t accelA;
  sensors_event_t gyroA;
  sensors_event_t tempA;
  float roll, pitch, yaw;
  soxA.getEvent(&accelA, &gyroA, &tempA);
  lis3mdlA.read();      // get mag X Y and Z data at once

  // apply calibrations
  applyCalibration(&accelA);
  applyCalibration(&gyroA);
  applyCalibration(&accelA);

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyroA.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyroA.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyroA.gyro.z * SENSORS_RADS_TO_DPS;

  // filter.updateIMU(gyroA.gyro.x, gyroA.gyro.y, gyroA.gyro.z, accelA.acceleration.x, accelA.acceleration.y, accelA.acceleration.z);
  filter.update(gx, gy, gz, accelA.acceleration.x, accelA.acceleration.y, accelA.acceleration.z, lis3mdlA.x, lis3mdlA.y, lis3mdlA.z);
  
  // // troubleshooting ***************************************
  // // Serial.println("IMU A:");
  // Serial.print(accelA.acceleration.x);
  // Serial.print(","); Serial.print(accelA.acceleration.y);
  // Serial.print(","); Serial.print(accelA.acceleration.z);
  // Serial.print(",");

  // Serial.print(gyroA.gyro.x);
  // Serial.print(","); Serial.print(gyroA.gyro.y);
  // Serial.print(","); Serial.print(gyroA.gyro.z);
  // Serial.println();

  // // Then print out the raw mag data
  // Serial.print(","); Serial.print(lis3mdlA.x); 
  // Serial.print(","); Serial.print(lis3mdlA.y); 
  // Serial.print(","); Serial.println(lis3mdlA.z); 
  // // *******************************************************
  // print orientation created from Madgwick
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
  Serial.print("Orientation: ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);
  // print quarternions on the line
  // Serial.print(filter.q0);
  // Serial.print(filter.q1);
  // Serial.print(filter.q2);
  // Serial.println(filter.q3);


}


void setup () 
{
//    Serial.begin(1000000);
  Serial1.begin(1000000);
  
  // change the ADC resolution to 12 bits
  analogReadResolution(12);
  byte error, address;
  Serial.begin(115200);
  I2C_0.begin(SDA_0 , SCL_0 , I2C_Freq);
  while (!Serial) delay(10); // will pause Zero, Leonardo, etc until serial console opens

// setup the pca ports
  for(address = 1; address < 127; address++ )
  {
    I2C_0.beginTransmission(address);
    error = I2C_0.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
    }
  }
  
  Serial.println("Adafruit LSM6DSOX test!");
  if (!soxA.begin_I2C(0x6B, &I2C_0)) { //0x6A
    while (1) {
      delay(10);
    }
  }
  if (!lis3mdlA.begin_I2C(0x1E, &I2C_0)) { //0x1C         // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("IMU Found!");

  // set the data rate for the IMU
  // Serial.println(soxA.getGyroDataRate());
  // Serial.println(soxA.getAccelDataRate());
  soxA.setAccelDataRate(LSM6DS_RATE_104_HZ);
  soxA.setGyroDataRate(LSM6DS_RATE_104_HZ);

  filter.setBeta(0.5);
  Serial.println(filter.getBeta());
  // while (1) { delay(10); }
  // setup Madgwick filter
  filter.begin(sensorRate);

  delay(1000);
  
  runner.init();
  runner.addTask(t2);
  t2.enable();
}


void loop () 
{
  runner.execute();
  // delay(1000);
}

bool applyCalibration(sensors_event_t &event)
{
  if (event.type == SENSOR_TYPE_MAGNETIC_FIELD) {
    // hard iron cal
    float mx = event.magnetic.x - mag_hardiron[0];
    float my = event.magnetic.y - mag_hardiron[1];
    float mz = event.magnetic.z - mag_hardiron[2];
    // soft iron cal
    event.magnetic.x =
        mx * mag_softiron[0] + my * mag_softiron[1] + mz * mag_softiron[2];
    event.magnetic.y =
        mx * mag_softiron[3] + my * mag_softiron[4] + mz * mag_softiron[5];
    event.magnetic.z =
        mx * mag_softiron[6] + my * mag_softiron[7] + mz * mag_softiron[8];
  } else if (event.type == SENSOR_TYPE_GYROSCOPE) {
    event.gyro.x -= gyro_zerorate[0];
    event.gyro.y -= gyro_zerorate[1];
    event.gyro.z -= gyro_zerorate[2];
  } else if (event.type == SENSOR_TYPE_ACCELEROMETER) {
    event.acceleration.x -= accel_zerog[0];
    event.acceleration.y -= accel_zerog[1];
    event.acceleration.z -= accel_zerog[2];
  } else {
    return false;
  }
  return true;
}