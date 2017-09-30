#include "PID.hpp"

#define gain_P 0.8
#define gain_I 0.05
#define gain_D 0.005
#define weight_H 0

#define GAIN_COMMAND_X 1
#define GAIN_COMMAND_Y 1
#define GAIN_COMMAND_Z 0
#define GAIN_COMMAND_H 1

#define DEBUG 0



PID::PID()
{
	last_height = 0;

  command.x = 0;
  command.y = 0;
  command.z = 0;
	command.h = 0;

  sum_error.x = 0;
  sum_error.y = 0;
  sum_error.z = 0;
	sum_error.h = 0;

	gain_x.p = gain_P;
	gain_y.p = gain_P;
	gain_z.p = 1.9;
	gain_h.p = 0.4;

	gain_x.i = gain_I;
	gain_y.i = gain_I;
	gain_z.i = 0.2;
	gain_h.i = 0.2;

	gain_x.d = gain_D;
	gain_y.d = gain_D;
	gain_z.d = 0.1;
	gain_h.d = 0.15;


	for(unsigned int i = 0 ; i < DERIVATE_BUFFER_SIZE ; i++)
	{
		last_errors[i].x = 0;
		last_errors[i].y = 0;
		last_errors[i].z = 0;
		last_errors[i].h = 0;
	}


  last_pid_calc = millis();
	serial_index = 0;
}

void PID::reset()
{
  command.x = 0;
  command.y = 0;
  command.z = 0;
  command.h = 0;

  sum_error.x = 0;
  sum_error.y = 0;
  sum_error.z = 0;
  sum_error.h = 0;

  last_pid_calc = millis();

	for(unsigned int i = 0 ; i < DERIVATE_BUFFER_SIZE ; i++)
	{
		last_errors[i].x = 0;
		last_errors[i].y = 0;
		last_errors[i].z = 0;
		last_errors[i].h = 0;
	}


}

void PID::calcCommand( vec4f position, vec4f consigne )
{
  time_loop = millis() / 1000.0 - last_pid_calc / 1000.0;
  last_pid_calc = millis();
/*
	if( position.h == 0 ) //if the ultrasonic sensors returns shit, we use the last good measurement he gave us
		position.h = last_height;
 	else
		last_height = position.h;
*/
	if( consigne.h - position.h > 5) 	consigne.h = position.h + 5;
	if( consigne.h - position.h < -5)	consigne.h = position.h - 5;


  float error_x = consigne.x - position.x;
  float error_y = consigne.y - position.y;
  float error_z = consigne.z - position.z;
  float error_h = consigne.h - position.h;

	last_errors[DERIVATE_BUFFER_SIZE - 1].x = error_x;
	last_errors[DERIVATE_BUFFER_SIZE - 1].y = error_y;
	last_errors[DERIVATE_BUFFER_SIZE - 1].z = error_z;
	last_errors[DERIVATE_BUFFER_SIZE - 1].h = error_h;

	for(unsigned int i = 0 ; i < DERIVATE_BUFFER_SIZE - 1 ; i++)
	{
		last_errors[i].x = last_errors[i + 1].x;
		last_errors[i].y = last_errors[i + 1].y;
		last_errors[i].z = last_errors[i + 1].z;
		last_errors[i].h = last_errors[i + 1].h;
	}

	float average_last_error_x = 0;
	float average_last_error_y = 0;
	float average_last_error_z = 0;
	float average_last_error_h = 0;


	for(unsigned int i = 0 ; i < DERIVATE_BUFFER_SIZE  ; i++)
	{
		average_last_error_x += last_errors[i].x;
		average_last_error_y += last_errors[i].y;
		average_last_error_z += last_errors[i].z;
		average_last_error_h += last_errors[i].h;
	}

	average_last_error_x /= DERIVATE_BUFFER_SIZE ;
	average_last_error_y /= DERIVATE_BUFFER_SIZE ;
	average_last_error_z /= DERIVATE_BUFFER_SIZE ;
	average_last_error_h /= DERIVATE_BUFFER_SIZE ;

	sum_error.x += error_x * time_loop;
  sum_error.y += error_y * time_loop;
  sum_error.z += error_z * time_loop;
  sum_error.h += error_h * time_loop;


  float p_x = error_x * gain_x.p;
  float p_y = error_y * gain_y.p;
  float p_z = error_z * gain_z.p;
  float p_h = error_h * gain_h.p;


  float i_x = (sum_error.x ) * gain_x.i;
  float i_y = (sum_error.y ) * gain_y.i;
  float i_z = (sum_error.z ) * gain_z.i;
  float i_h = (sum_error.h ) * gain_h.i;


  float d_x = ( error_x - average_last_error_x ) / time_loop  * gain_x.d;
  float d_y = ( error_y - average_last_error_y ) / time_loop  * gain_y.d;
  float d_z = ( error_z - average_last_error_z ) / time_loop  * gain_z.d;
  float d_h = ( error_h - average_last_error_h ) / time_loop  * gain_h.d;


  command.x = (p_x + i_x + d_x);
  command.y = (p_y + i_y + d_y);
  command.z = (p_z + i_z + d_z);
  command.h = ((p_h + i_h + d_h) * weight_H);



	if(DEBUG)
	{

  //  Serial.print( average_last_error_h , 2); Serial.print("\t");
  //  Serial.print( error_h , 2); Serial.print("\n");

    Serial.print( position.x , 2); Serial.print("\t");
		Serial.print( p_h , 2);  Serial.print("\t");
		Serial.print( i_h , 2);  Serial.print("\t");
		Serial.print( d_h , 2);  Serial.print("\t");
		Serial.print( command.h , 2);  Serial.print("\n");

	}

	serial_index++;

}
