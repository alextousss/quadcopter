#ifndef DEF_MOTOR
#define DEF_MOTOR

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "datastructs.hpp"


class MotorManager
{
    public:
        MotorManager();

        void command(float command_x, float command_y, float command_z, float command_h);
        float getMotorValue(int motor_id);
        void startMotors();
        void setOn();
        void setOff();
        vec4f getMotorValues() {
            return {motor_value[0], motor_value[1], motor_value[2], motor_value[3]};
        }
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
