#ifndef DEF_MOTOR
#define DEF_MOTOR

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>


class MotorManager
{
public:
  MotorManager();

  void command(float command_x, float command_y, float command_z, float command_h);
  float getMotorValue(int motor_id);
  void startMotors();
  void stop();

private:
  float motor_value[4];
  Servo motor[4];

  float gain_x;
  float gain_y;
  float gain_z;
  float gain_h;

  bool stop_motor;

};

#endif
