#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Ultrasonic.h>

#include "../include/IMUsensor.hpp"
#include "../include/PID.hpp"
#include "../include/motormanager.hpp"

bool debug = 1;

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


  Ultrasonic ultrasonic(6,5);      //objet pour contrôler le capteur ultrason
  IMUsensor mpu;                  //objet pour récupérer les valeurs de l'IMU et calculer une orientation absolue
  PID pid;                        //objet qui gère le calcul des directives pour les moteurs
  MotorManager motors;            //objet qui gère le calcul des valeurs par moteur, et s'occupe de les contrôler

  mpu.calibrateSensors();
  delay(100);
  motors.startMotors();

  while(true)
  {

    mpu.calcAbsoluteOrientation(0.98);
    mpu.actualizeSensorData();

    //calcul du PID avec les valeurs de l'IMU
    pid.calcCommand(mpu.getX(), mpu.getY(), mpu.getZ(), ultrasonic.Ranging(CM) - 5, mpu.getAngularSpeedX(), mpu.getAngularSpeedY(), mpu.getAngularSpeedZ(), 0, 0, 0, 20);

    motors.command( pid.getCommandX(), pid.getCommandY(), pid.getCommandZ(), pid.getCommandH() ); //commande des moteurs avec les valeurs données par le PID

    if(millis() - millis_at_last_print > 60)
    {
      Serial.print( motors.getMotorValue(0) );  Serial.print("\t");
      Serial.print( motors.getMotorValue(1) );  Serial.print("\t");
      Serial.print( motors.getMotorValue(2) );  Serial.print("\t");
      Serial.print( motors.getMotorValue(3) ); Serial.print("\t|\t");
      Serial.print( mpu.getX(), 2); Serial.print("\t");
      Serial.print( mpu.getY(), 2); Serial.print("\t");
      Serial.print( mpu.getZ(), 2 ); Serial.print("\t\n");



    //  Serial.print("command H : ") ; Serial.print( pid.getCommandH(), 2 ) ; Serial.print("\t\tsum error h :\t"); Serial.println( pid.getSumErrorH(), 2 );
      millis_at_last_print = millis();
    }
  }
}
