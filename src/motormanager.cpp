#include "motormanager.hpp"

MotorManager::MotorManager()
{
  for(int i = 0; i <= 3 ; i++)
  {
    motor_value[i] = 0;
  }

  motor[0].attach(23);
  motor[1].attach(22);
  motor[2].attach(20);
  motor[3].attach(21);

  gain_x = 1.5;
  gain_y = 1.5;
  gain_z = 1;
  gain_h = 1;

  stop_motor = 1;

}
void MotorManager::setOn()
{
  stop_motor = 0;
}


void MotorManager::setOff()
{
  for(unsigned int i = 0 ; i < 4 ; i++)
  {
    motor_value[i] = 0;
    motor[i].write(55);
  }

  stop_motor = 1;

}

void MotorManager::command(float command_x, float command_y, float command_z, float command_h)
{

  motor_value[0] = gain_y * ( 1 * command_y) + gain_x * (-1 * command_x) /*+ ( 1 * command_z)*/ + gain_h * command_h;
  motor_value[1] = gain_y * ( 1 * command_y) + gain_x * ( 1 * command_x) /*+ (-1 * command_z)*/ + gain_h * command_h;
  motor_value[2] = gain_y * (-1 * command_y) + gain_x * ( 1 * command_x) /*+ ( 1 * command_z)*/ + gain_h * command_h;
  motor_value[3] = gain_y * (-1 * command_y) + gain_x * (-1 * command_x) /*+ (-1 * command_z)*/ + gain_h * command_h;



  for(unsigned int i = 0 ; i < 4 ; i++)
  {
    motor_value[i] += (160-55);
    motor_value[i] = (motor_value[i] <= 55) ? 55 : motor_value[i]; //just to make sure that the motor values are always between 55 (stop of the motors) and 160 (max value)
    motor_value[i] = (motor_value[i] > 160) ? 160 : motor_value[i];
  }

  for(int i = 0; i <= 3 ; i++)
  {
    if(! stop_motor)
    {
      motor[i].write(motor_value[i]);
    }
    else
    {
      motor_value[i] = 0;
      motor[i].write(55);
    }
  }

}

void MotorManager::startMotors()    //needed to "arm" the ESC, without that, they wont start.
{
  delay(100);
  for(int i = 0; i <= 3 ; i++)
  {
    motor[i].write(10);
  }
  delay(100);

  stop_motor = 0;
}

float MotorManager::getMotorValue(int motor_id)
{
  if(motor_id < 4 && motor_id >= 0)
  {
    return motor_value[motor_id];
  }

  return 0;
}
