#include "PID.hpp"

#define gain_P 2
#define gain_I 0.000
#define gain_D 1
#define weight_H 0.7

#define GAIN_COMMAND_X 1
#define GAIN_COMMAND_Y 1
#define GAIN_COMMAND_Z 0.7
#define GAIN_COMMAND_H 1

#define DEBUG 0



PID::PID()
{
	last_height = 0;

  command_x = 0;
  command_y = 0;
  command_z = 0;

  sum_error_x = 0;
  sum_error_y = 0;
  sum_error_z = 0;
	sum_error_h = 0;

  last_error_x = 0;
  last_error_y = 0;
  last_error_z = 0;
  last_error_h = 0;

  gain_p_x = gain_P; //gain for the proportional correction
  gain_p_y = gain_P;
  gain_p_z = 1;
  gain_p_h = 0.3;

  gain_i_x = gain_I; //gain for the integral correction
  gain_i_y = gain_I;
  gain_i_z = 0.025;
  gain_i_h = 0.2;

  gain_d_x = gain_D; //gain for the derivation correction
  gain_d_y = gain_D;
  gain_d_z = 0.20;
  gain_d_h = 0.15;


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
  time_loop = millis() / 1000.0 - last_pid_calc / 1000.0;
  last_pid_calc = millis();

	if( height == 0 ) //if the ultrasonic sensors returns shit, we use the last good measurement he gave us
		height = last_height;
 	else
		last_height = height;

	if( order_h - height > 5) 	order_h = height + 5; //here we just
	if( order_h - height < -5)	order_h = height - 5;


  float error_x = orientation_x - order_x;
  float error_y = orientation_y - order_y;
  float error_z = orientation_z - order_z;
  float error_h = height - order_h;

  sum_error_x += error_x * time_loop;
  sum_error_y += error_y * time_loop;
  sum_error_z += error_z * time_loop;
  sum_error_h += error_h * time_loop;


  float p_x = error_x * gain_p_x;
  float p_y = error_y  * gain_p_y;
  float p_z = error_z  * gain_p_z;
  float p_h = error_h  * gain_p_h;


  float i_x = (sum_error_x * -1) * gain_i_x;
  float i_y = (sum_error_y * -1) * gain_i_y;
  float i_z = (sum_error_z * -1) * gain_i_z;
  float i_h = (sum_error_h * -1) * gain_i_h;


  float d_x = ( error_x - last_error_x ) / time_loop      * gain_d_x;
  float d_y = ( error_x - last_error_x ) / time_loop      * gain_d_y;
  float d_z = ( error_x - last_error_x ) / time_loop * -1 * gain_d_z;
  float d_h = ( error_x - last_error_x ) / time_loop * -1 * gain_d_h;



  command_x = (p_x + i_x + d_x);
  command_y = (p_y + i_y + d_y);
  command_z = (p_z + i_z + d_z);
  command_h = ((p_h + i_h + d_h) * weight_H);

  last_error_x = error_x;
  last_error_y = error_y;
  last_error_z = error_z;
  last_error_h = error_h;


	if(DEBUG)
	{
		Serial.print( p_x , 2);  Serial.print("\t");
		Serial.print( i_x , 2);  Serial.print("\t");
		Serial.print( d_x , 2);  Serial.print("\t");
		Serial.print( command_x , 2);  Serial.print("\n");
	}



}
