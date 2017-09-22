#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>
#include <VirtualWire.h>

#include "IMUsensor.hpp"
#include "PID.hpp"
#include "motormanager.hpp"
#include <VirtualWire.h>

#define PERCENTAGE_RADIO_SENT 10
#define PRINT_PERIOD 30
#define MOTOR_MAX_DURATION 4000
#define MAX_SAMPLE_BUFFER_SIZE  2000


struct Sample
{
  float x;
  float y;
  float z;
  float command_x;
  float command_y;
  float command_z;
};

Sample samples[MAX_SAMPLE_BUFFER_SIZE];

bool safe_mode = 1;             //si activé, les moteurs se coupent automatiquement après 3 secondes d'allumage
bool wait_serial = 0;                // et ce afin d'éviter une perte de contrôle du quadricoptère sur le banc de test
bool serial_debug = 1;
bool radio_debug = 0;
bool sd_debug = 1;

bool motor_started = 0;

unsigned long millis_at_motor_start = 0;
unsigned long millis_at_last_print = 0;
unsigned long millis_at_last_loop = 0;
unsigned long millis_at_last_max_time_loop = 0;
unsigned long time_loop = 0;
unsigned long max_time_loop = 0;

float desired_height = 15;
float sonar_height = 0;
float sonar_speed = 0;

float last_sonar_height = 0;
unsigned long time_at_last_sonar_height = 0;

unsigned int sample_num = 0;
unsigned int sample_id = 0;

NewPing sonar(6,5, 500);

IMUsensor mpu;                  //objet pour récupérer les valeurs de l'IMU et calculer une orientation absolue
PID pid;                        //objet qui gère le calcul des directives pour les moteurs
MotorManager motors;            //objet qui gère le calcul des valeurs par moteur, et s'occupe de les contrôler

void setup()
{
  pinMode(9, INPUT_PULLUP); //on configure les entrées pour pouvoir utiliser le bouton
  Serial.begin(115200);
  vw_setup(1200);
  mpu.calibrateSensors();

  if(wait_serial)
    while(!Serial); //on attends que le port série soit ouvert pour commencer les calculs

  motors.startMotors();

}


void loop()
{
  time_loop = millis() - millis_at_last_loop;
  millis_at_last_loop = millis();

  if( time_loop > max_time_loop  || millis() - millis_at_last_max_time_loop > 1000) //calcul du loop ayant prit le plus de temps dans la dernière seconde
  {
    max_time_loop = time_loop;
    millis_at_last_max_time_loop = millis();
  }


  if( millis() - time_at_last_sonar_height > 50 ) //ici on utilise le capteur ultrason pour connaître l'altitude du quadricoptere toutes les 50ms
  {
    last_sonar_height = sonar_height;

    sonar_height = sonar.ping_cm(80);
    sonar_speed = ( sonar_height - last_sonar_height ) / ( ( millis() - time_at_last_sonar_height ) / 400.0f );
    time_at_last_sonar_height = millis();
  }


  mpu.actualizeSensorData(); //on actualise les capteurs et on calcule l'orientation
  mpu.calcAbsoluteOrientation(0.99);

  motor_started = 1;
  millis_at_motor_start = millis();
  motors.setOn();


    //calcul du PID avec les valeurs de l'IMU
    pid.calcCommand(mpu.getX(), mpu.getY(), mpu.getZ(), sonar_height , sonar_speed, mpu.getAngularSpeedX(), mpu.getAngularSpeedY(), mpu.getAngularSpeedZ(), 0, 0, 0, desired_height);

    motors.command( pid.getCommandX(), pid.getCommandY(), pid.getCommandZ(), pid.getCommandH() ); //commande des moteurs avec les valeurs données par le PID

    if( sample_id % 10 == 0 )
    {
      samples[sample_num] = { mpu.getX(), mpu.getY(), mpu.getZ(), pid.getCommandX(), pid.getCommandY(), pid.getCommandZ() };
      sample_num++;
    }
    sample_id++;

    if( sd_debug && sample_num >= MAX_SAMPLE_BUFFER_SIZE )
    {

    }

    if ( ( safe_mode && motor_started  && millis() - millis_at_motor_start > MOTOR_MAX_DURATION ) )
    {
      motors.setOff();
    	if( millis() - millis_at_last_print > PRINT_PERIOD)
    	{
    		Serial.println("stop bc of millis > MOTOR_MAX_DURATION");
    	}
    }



    if(millis() - millis_at_last_print > PRINT_PERIOD)
    {
      if(serial_debug)
      {
        Serial.print( motors.getMotorValue(0) );  Serial.print("\t");
        Serial.print( motors.getMotorValue(1) );  Serial.print("\t");
        Serial.print( motors.getMotorValue(2) );  Serial.print("\t");
        Serial.print( motors.getMotorValue(3) ); Serial.print("\t|\t");
        Serial.print( mpu.getX(), 2 ); Serial.print("\t");
        Serial.print( mpu.getY(), 2 ); Serial.print("\t");
        Serial.print( mpu.getZ(), 2 ); Serial.print("\t|\t");
        Serial.print( sonar_height, 2 ); Serial.print("\t");
        Serial.print( max_time_loop ); Serial.print("\t");
        Serial.print( sample_num ); Serial.print("\n");
      }
      millis_at_last_print = millis();
    }

}
