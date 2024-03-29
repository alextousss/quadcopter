#include "motormanager.hpp"

MotorManager::MotorManager()
{
    for(int i = 0; i <= 3 ; i++)
    {
        motor_value[i] = 0;
    }

    motor[0].attach(6);//23
    motor[1].attach(7);//22
    motor[2].attach(9);//20
    motor[3].attach(8);//21

    gain_x = 1;
    gain_y = 1;
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
    float commands[3] = {command_x, command_y, command_z};

    motor_value[0] = (1 * commands[1]) + (1 * commands[0]) + ( 1 * commands[2]) + command_h;
    motor_value[1] = ( -1 * commands[1]) + (1 * commands[0]) + (-1 * commands[2]) + command_h;
    motor_value[2] = ( -1 * commands[1]) + (-1 * commands[0]) + ( 1 * commands[2]) + command_h;
    motor_value[3] = (1 * commands[1]) + ( -1 * commands[0]) + (-1 * commands[2]) + command_h;
/*
    motor_value[0] = gain_y * ( 1 * command_y) + gain_x * (-1 * command_x) + (-1 * command_z) + gain_h * command_h;
    motor_value[1] = gain_y * ( 1 * command_y) + gain_x * ( 1 * command_x) + ( 1 * command_z) + gain_h * command_h;
    motor_value[2] = gain_y * (-1 * command_y) + gain_x * ( 1 * command_x) + (-1 * command_z) + gain_h * command_h;
    motor_value[3] = gain_y * (-1 * command_y) + gain_x * (-1 * command_x) + ( 1 * command_z) + gain_h * command_h;
*/

    for(int i = 0; i < 4 ; i++)
    {
        motor_value[i] = min(motor_value[i], 0.7f);
        motor_value[i] = max(motor_value[i], 0);
        if(! stop_motor)
        {
            motor[i].writeMicroseconds(1000 + motor_value[i]*1000); // moteurs calibrés pour être à zéro pour 1000 et au max à 2000
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
    for(int i = 0; i <= 3 ; i++)
    {
        motor[i].write(55);
    }

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
