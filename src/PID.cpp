#include "PID.hpp"

#define gain_P 0.9
#define gain_I 0.3
#define gain_D 0.20
#define weight_H 1

#define GAIN_COMMAND_X 0.65
#define GAIN_COMMAND_Y 0.65
#define GAIN_COMMAND_Z 1
#define GAIN_COMMAND_H 1

PID::PID()
{
  command_x = 0;
  command_y = 0;
  command_z = 0;

  sum_error_x = 0;
  sum_error_y = 0;
  sum_error_z = 0;
	sum_error_h = 0;

  gain_p_x = gain_P; //gain for the proportional correction
  gain_p_y = gain_P;
  gain_p_z = 1;
  gain_p_h = 1;

  gain_i_x = gain_I; //gain for the integral correction
  gain_i_y = gain_I;
  gain_i_z = 0.025;
  gain_i_h = 0.5;

  gain_d_x = gain_D; //gain for the derivation correction
  gain_d_y = gain_D;
  gain_d_z = 0.20;
  gain_d_h = 0.010;

  gain_command_x = GAIN_COMMAND_X;
  gain_command_y = GAIN_COMMAND_Y;
  gain_command_z = GAIN_COMMAND_Z;
  gain_command_h = GAIN_COMMAND_H;


  last_pid_calc = millis();

}

void PID::reset()
{
  command_x = 0;
  command_y = 0;
  command_z = 0;
  command_h = 0;

  sum_error_x = 0;
  sum_error_y = 0;
  sum_error_z = 0;
  sum_error_h = 0;

  last_pid_calc = millis();


}

void PID::calcCommand( float orientation_x,
                float orientation_y,        //orientation de l'IMU
                float orientation_z,
                float height,               //hauteur
                float vertical_speed,				//vitesse verticale
                float angular_speed_x,      //vitesse angulaire
                float angular_speed_y,
                float angular_speed_z,
                float order_x,              //aka consigne
                float order_y,
                float order_z,
                float order_h )
{
  sum_error_x += (orientation_x - order_x) * time_loop * 1;
  sum_error_y += (orientation_y - order_y) * time_loop * 1;
  sum_error_z += (orientation_z - order_z) * time_loop * 1;
  sum_error_h += (height - order_h) * time_loop;


  float p_x = (order_x - orientation_x) * gain_p_x;
  float p_y = (order_y - orientation_y) * gain_p_y;
  float p_z = (order_z - orientation_z) * gain_p_z;
  float p_h = (order_h - height) * gain_p_h;


  float i_x = (sum_error_x * -1) * gain_i_x;
  float i_y = (sum_error_y * -1) * gain_i_y;
  float i_z = (sum_error_z * -1) * gain_i_z;
  float i_h = (sum_error_h * -1) * gain_i_h;


  float d_x = (angular_speed_x - order_x - orientation_x) * -1 * gain_d_x;
  float d_y = (angular_speed_y - order_y - orientation_y) * -1 * gain_d_y;
  float d_z = (angular_speed_z - order_z - orientation_z) * -1 * gain_d_z;
  float d_h = (vertical_speed - order_h - height) * -1 * gain_d_h;

  command_x = (p_x + i_x + d_x) * gain_command_x;
  command_y = (p_y + i_y + d_y) * gain_command_x;
  command_z = (p_z + i_z + d_z) * gain_command_x;
  command_h = ((p_h + i_h + d_h) * weight_H) * gain_command_x;

  time_loop = millis() / 1000.0 - last_pid_calc / 1000.0;
  last_pid_calc = millis();
}
