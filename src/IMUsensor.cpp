#include "../include/IMUsensor.hpp"

#define MPU_addr 0x68
#define FS_SEL 131


IMUsensor::IMUsensor()
{
  raw_gyro_x = 0; //raw gyroscope data coming from the IMU
  raw_gyro_y = 0;
  raw_gyro_z = 0;

  offset_gyro_x = 0; //offset for the calibration of the gyroscope
  offset_gyro_y = 0;
  offset_gyro_z = 0;

  raw_accel_x = 0; //raw accelerometer data coming from the IMU
  raw_accel_y = 0;
  raw_accel_z = 0;

  last_loop_gyro_x = 0; //degrees per second since the last loop
  last_loop_gyro_y = 0;
  last_loop_gyro_z = 0;

  accel_x = 0; //absolute orientation from accelerometer
  accel_y = 0;
  accel_z = 0;

  gyro_x = 0; //absolute orientation from accelerometer
  gyro_y = 0;
  gyro_z = 0;



  orientation_x = 0; //absolute orientation
  orientation_y = 0;
  orientation_z = 0;

  raw_temperature = 0;

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("end constructor");
}

bool IMUsensor::calcAbsoluteOrientation( float complementary_rate )
{

  last_loop_gyro_x = raw_gyro_x / FS_SEL;
  last_loop_gyro_y = raw_gyro_y / FS_SEL;
  last_loop_gyro_z = raw_gyro_z / FS_SEL;

  gyro_x = last_loop_gyro_x * time_loop + orientation_x;
  gyro_y = last_loop_gyro_y * time_loop + orientation_y;
  gyro_z = last_loop_gyro_z * time_loop + orientation_z;


  accel_x = atan(raw_accel_y/sqrt(pow(raw_accel_x, 2) + pow(raw_accel_z, 2))) * (180/3.14159);    //converts the raw accelerometer values to an absolute orientation (in degrees)
  accel_y = atan(-1*raw_accel_x/sqrt(pow(raw_accel_y, 2) + pow(raw_accel_z, 2))) * (180/3.14159);

  orientation_x = (complementary_rate * gyro_x) + ((1 - complementary_rate) * accel_x);
  orientation_y = (complementary_rate * gyro_y) + ((1 - complementary_rate) * accel_y);
  orientation_z = gyro_z;

  return true;
}


bool IMUsensor::actualizeSensorData()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);


  raw_accel_x=Wire.read()<<8|Wire.read();
  raw_accel_y=Wire.read()<<8|Wire.read();
  raw_accel_z=Wire.read()<<8|Wire.read();

  raw_temperature=Wire.read()<<8|Wire.read();

  raw_gyro_x=Wire.read()<<8|Wire.read();
  raw_gyro_y=Wire.read()<<8|Wire.read();
  raw_gyro_z=Wire.read()<<8|Wire.read();

  raw_gyro_x -= offset_gyro_x;
  raw_gyro_y -= offset_gyro_y;
  raw_gyro_z -= offset_gyro_z;


  time_loop = millis() / 1000.0 - last_data_refresh / 1000.0;
  last_data_refresh = millis();

  return true;

}

bool IMUsensor::calibrateSensors()
{
  #define NUM_SAMPLES 100

  for (int i = 0 ; i < NUM_SAMPLES ; i++) //assumes that the quadcopter has no motio during the calibration
  {
    actualizeSensorData();
    offset_gyro_x += raw_accel_x;
    offset_gyro_y += raw_accel_y;
    offset_gyro_z += raw_accel_z;
    delay(10);
  }

  offset_gyro_x /= NUM_SAMPLES;
  offset_gyro_y /= NUM_SAMPLES;
  offset_gyro_z /= NUM_SAMPLES;

  return true;
}
