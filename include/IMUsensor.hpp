#ifndef DEF_IMUSENSOR
#define DEF_IMUSENSOR

#include <Arduino.h>
#include <Wire.h>


class IMUsensor
{
public:
  IMUsensor();

  bool calibrateSensors();
  bool actualizeSensorData();

  bool calcAbsoluteOrientation( float complementary_rate );


  //accesseurs

  float getAngularSpeedX() { return last_loop_gyro_x ; }
  float getAngularSpeedY() { return last_loop_gyro_y ; }
  float getAngularSpeedZ() { return last_loop_gyro_z ; }

  float getX() { return orientation_x - 0.5 ; }
  float getY() { return orientation_y + 1	; }
  float getZ() { return orientation_z ; }



private:
  int16_t raw_gyro_x; //raw gyroscope data coming from the IMU
  int16_t raw_gyro_y;
  int16_t raw_gyro_z;

  int16_t offset_gyro_x; //offset from the calibration of the IMU
  int16_t offset_gyro_y;
  int16_t offset_gyro_z;

  int16_t raw_accel_x; //raw accelerometer data coming from the IMU
  int16_t raw_accel_y;
  int16_t raw_accel_z;

  float last_loop_gyro_x; //degrees per second since the last loop
  float last_loop_gyro_y;
  float last_loop_gyro_z;

  float accel_x; //absolute orientation from accelerometer
  float accel_y;
  float accel_z;

  float gyro_x; //absolute orientation from accelerometer
  float gyro_y;
  float gyro_z;



  float orientation_x; //absolute orientation
  float orientation_y;
  float orientation_z;

  int16_t raw_temperature;

  bool gyro_angle_set;

  unsigned long last_data_refresh;
  float time_loop;

};

#endif
