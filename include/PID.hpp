#ifndef DEF_PID
#define DEF_PID

#include <Arduino.h>
#include <Wire.h>

class PID
{
public:
  PID();
  void reset();

  void calcCommand( float orientation_x,
                  float orientation_y,
                  float orientation_z,
                  float height,
                  float vertical_speed,
                  float angular_speed_x,
                  float angular_speed_y,
                  float angular_speed_z,
                  float order_x,
                  float order_y,
                  float order_z,
                  float order_h );

  float getCommandX() { return command_x ; }
  float getCommandY() { return command_y ; }
  float getCommandZ() { return command_z ; }
  float getCommandH() { return command_h ; }


  float getSumErrorX() { return sum_error_x ; }
  float getSumErrorY() { return sum_error_y ; }
  float getSumErrorZ() { return sum_error_z ; }
  float getSumErrorH() { return sum_error_h ; }

	float getProportionalCorrectionX(float order_x, float orientation_x) { return (( order_x - orientation_x ) * gain_p_x) ;  }
	float getDerivateCorrectionX(float order_x, float orientation_x) { return ( ( order_x - orientation_x ) - last_error_x ) / time_loop  * gain_d_x ; }

private:
	float last_height;
  float gain_p_x; //gain for the proportional correction
  float gain_p_y;
  float gain_p_z;
  float gain_p_h;


  float gain_i_x; //gain for the integral correction
  float gain_i_y;
  float gain_i_z;
  float gain_i_h;


  float gain_d_x; //gain for the derivation correction
  float gain_d_y;
  float gain_d_z;
  float gain_d_h;


  float command_x; //final command
  float command_y;
  float command_z;
  float command_h;

  float gain_command_x; //gain for the final command
  float gain_command_y;
  float gain_command_z;
  float gain_command_h;

  float sum_error_x; //sum of the errors, used for the integral correction
  float sum_error_y;
  float sum_error_z;
  float sum_error_h;

  float last_error_x;
  float last_error_y;
  float last_error_z;
  float last_error_h;

  unsigned long last_pid_calc;
  float time_loop;
};

#endif
