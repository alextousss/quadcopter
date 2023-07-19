#ifndef DATASTRUCTS
#define DATASTRUCTS
#include <Arduino.h>

struct vec3int16 {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct gain3f
{
  float p;
  float i;
  float d;
};

struct Gains4
{
  gain3f x;
  gain3f y;
  gain3f z;
  gain3f h;
};

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

#endif
