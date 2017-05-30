#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>

#include "IMUsensor.hpp"
#include "PID.hpp"
#include "motormanager.hpp"

bool safe_mode = 1;             //si activé, les moteurs se coupent automatiquement après 3 secondes d'allumage
bool debug = 1;                // et ce afin d'éviter une perte de contrôle du quadricoptère sur le banc de test

void setup()
{
  pinMode(9, INPUT_PULLUP); //on configure les entrées pour pouvoir utiliser le bouton
  Serial.begin(9600);
}


void loop()
{
  unsigned long millis_at_last_print = millis();

  if(debug)
  {
    while(!Serial); //on attends que le port série soit ouvert pour commencer les calculs
  }

  bool motor_started = 0;
  unsigned long millis_at_motor_start = 0;

//  Ultrasonic ultrasonic(6,5);      //objet pour contrôler le capteur ultrason
  NewPing sonar(6,5, 500);
  IMUsensor mpu;                  //objet pour récupérer les valeurs de l'IMU et calculer une orientation absolue
  PID pid;                        //objet qui gère le calcul des directives pour les moteurs
  MotorManager motors;            //objet qui gère le calcul des valeurs par moteur, et s'occupe de les contrôler

  mpu.calibrateSensors();
  motors.startMotors();

  delay(1000);

  while(true)
  {
    mpu.calcAbsoluteOrientation(0.99);
    mpu.actualizeSensorData();

    if( !digitalRead(9) == LOW )
    {
      pid.reset();
      motors.stop();
      if(!safe_mode)
        motor_started = 0;
    }
    else
    {
      if(!motor_started)
      {
        motor_started = 1;
        millis_at_motor_start = millis();
      }

      //calcul du PID avec les valeurs de l'IMU
      pid.calcCommand(mpu.getX(), mpu.getY(), mpu.getZ(), sonar.ping_cm() - 5, mpu.getAngularSpeedX(), mpu.getAngularSpeedY(), mpu.getAngularSpeedZ(), 0, 0, 0, 30);
      float command_h = (pid.getCommandH() > 20) ? 20 : pid.getCommandH();
      motors.command( pid.getCommandX(), pid.getCommandY(), pid.getCommandZ(), command_h ); //commande des moteurs avec les valeurs données par le PID

    }

    if(millis() - millis_at_last_print > 30)
    {
      Serial.print( motors.getMotorValue(0) );  Serial.print("\t");
      Serial.print( motors.getMotorValue(1) );  Serial.print("\t");
      Serial.print( motors.getMotorValue(2) );  Serial.print("\t");
      Serial.print( motors.getMotorValue(3) ); Serial.print("\t|\t");
      Serial.print( mpu.getX(), 2); Serial.print("\t");
      Serial.print( mpu.getY(), 2); Serial.print("\t");
      Serial.print( mpu.getZ(), 2 ); Serial.print("\t | \t");
      Serial.print( pid.getCommandH() ); Serial.print("\n");


      millis_at_last_print = millis();
    }

    if ( safe_mode && motor_started  && millis() - millis_at_motor_start > 3000 )
    {
      motors.stop();
      Serial.println("stop bc of millis > 3000");
    }
    delay(10);
  }
}
