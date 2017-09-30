#ifndef DEF_PID
#define DEF_PID

#include <Arduino.h>
#include <Wire.h>

#define DERIVATE_BUFFER_SIZE 10

class PID
{
public:
  PID();
  void reset();

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

  vec4f command;
  vec4f gain_command;
  vec4f sum_error;
	vec4f last_errors[DERIVATE_BUFFER_SIZE];


  unsigned long last_pid_calc;
  float time_loop;
	unsigned int serial_index;
};

#endif
