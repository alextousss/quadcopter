#ifndef DEF_IMUSENSOR
#define DEF_IMUSENSOR

#include <Arduino.h>
#include "datastructs.hpp"
#include <Wire.h>


class IMUsensor
{
    public:
        IMUsensor();
        void resetOrientation() { gyro_angle_set = false ;}

        bool calibrateSensors();
        bool actualizeSensorData();

        bool calcAbsoluteOrientation( float complementary_rate );


        //accesseurs
        vec3f getRotation() { return last_loop_gyro; };
        vec3f getOrientation() { return orientation; };

    private:
        vec3int16 raw_gyro;
        vec3int16 offset_gyro;
        vec3int16 raw_accel;
        vec3f last_loop_gyro;
        vec3f accel;
        vec3f gyro;
        vec3f orientation;
        int16_t raw_temperature;
        bool gyro_angle_set;
        unsigned long last_data_refresh;
        float time_loop;
};

#endif
