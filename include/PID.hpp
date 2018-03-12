#ifndef DEF_PID
#define DEF_PID

#include <Arduino.h>
#include "datastructs.hpp"



class PID
{
public:
  PID();
  PID(gain3f gains);
  void setGains(gain3f gains) { this->gains = gains; }
  void reset()                { error_sum = 0; last_error = 0; }
  float getCorrection(float instruction, float situation, uint16_t delta_time);
  float getDerivateCorrection()     { return derivative; }
  float getProportionalCorrection() { return proportional; }
  float getIntegralCorrection()     { return integral; }

private:
  gain3f gains;
  float proportional;
  float derivative;
  float integral;
  float error_sum;
  float last_error;

/*
  struct vec3f
  {
    float x;
    float y;
    float z;
  };

  struct vec4f
  {
    float x;
    float y;
    float z;
    float h;
  };

  struct gain3f
  {
    float p;
    float i;
    float d;
  };


	void calcCommand( vec4f position, vec4f consigne );

	vec4f getCommand() { return command ; }
	vec4f getProportionalCorrection() { return proportional ; }
	vec4f getDerivateCorrection() { return derivate ; }
	vec4f getIntegralCorrection() { return integral ; }

	void setGainX( gain3f new_gain ) { this->gain_x = new_gain; }
  void setGainY( gain3f new_gain ) { this->gain_y = new_gain; }
  void setGainZ( gain3f new_gain ) { this->gain_z = new_gain; }
  void setGainH( gain3f new_gain ) { this->gain_h = new_gain; }



private:
	float last_height;
  gain3f gain_x;
  gain3f gain_y;
  gain3f gain_z;
  gain3f gain_h;

	vec4f proportional;
	vec4f derivate;
	vec4f integral;
  vec4f command;
  vec4f gain_command;
  vec4f sum_error;
	vec4f last_errors[DERIVATE_BUFFER_SIZE];


  unsigned long last_pid_calc;
  float time_loop;
	unsigned int serial_index;
  */
};

#endif
