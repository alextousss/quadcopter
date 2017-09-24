#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>
#include <SD.h>

#include "IMUsensor.hpp"
#include "PID.hpp"
#include "motormanager.hpp"

#define PERCENTAGE_RADIO_SENT 10
#define PRINT_PERIOD 30
#define MOTOR_MAX_DURATION 4000
#define MAX_SAMPLE_BUFFER_SIZE  1000


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

float desired_height = 70;
float sonar_height = 0;
float sonar_speed = 0;

float last_sonar_height = 0;
unsigned long time_at_last_sonar_height = 0;

unsigned int sample_num = 0;
unsigned int sample_id = 0;

//NewPing sonar(6,5, 500);

const unsigned short chip_select = 10;

PID pid;                        //objet qui gère le calcul des directives pour les moteurs
MotorManager motors;            //objet qui gère le calcul des valeurs par moteur, et s'occupe de les contrôler
IMUsensor mpu;                  //objet pour récupérer les valeurs de l'IMU et calculer une orientation absolue

void setup()
{
  pinMode(5, OUTPUT);

  Serial.begin(115200);
  if(wait_serial)
    while(!Serial); //on attends que le port série soit ouvert pour commencer les calculs


//  pinMode(9, INPUT_PULLUP); //on configure les entrées pour pouvoir utiliser le bouton
  mpu.calibrateSensors();

  Serial.print("Initialisation de la carte SD ...");

  //On regarde si la carte est présente
  if (!SD.begin(chip_select))
  {
    while(true)
    {
      Serial.println("Pas possible d'initialiser la carte SD");
    }
  }

  Serial.println("Carte initialisée.");

  motors.startMotors();

  motor_started = 1;
  millis_at_motor_start = millis();

  motors.setOn();
  digitalWrite(5, LOW);
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

    sonar_height = 0;//sonar.ping_cm(80);
    sonar_speed = ( sonar_height - last_sonar_height ) / ( ( millis() - time_at_last_sonar_height ) / 400.0f );
    time_at_last_sonar_height = millis();
  }


  mpu.actualizeSensorData(); //on actualise les capteurs et on calcule l'orientation
  mpu.calcAbsoluteOrientation(0.99);


  //calcul du PID avec les valeurs de l'IMU
  pid.calcCommand( { mpu.getX(), mpu.getY(), mpu.getZ(), sonar_height } , { 0, 0, 0, desired_height } );

  motors.command( pid.getCommand().x, pid.getCommand().y, pid.getCommand().z, pid.getCommand().h ); //commande des moteurs avec les valeurs données par le PID

  if( sample_id % 10 == 0 )
  {
    samples[sample_num] = { mpu.getX(), mpu.getY(), mpu.getZ(), pid.getCommand().x, pid.getCommand().y, pid.getCommand().z };
    sample_num++;
  }

  sample_id++;


  if ( ( safe_mode && motor_started  && millis() - millis_at_motor_start > MOTOR_MAX_DURATION ) || (sd_debug && sample_num >= MAX_SAMPLE_BUFFER_SIZE ) )
  {
    motors.setOff();
  	if( millis() - millis_at_last_print > PRINT_PERIOD)
  	{
  		Serial.println("stop bc of millis > MOTOR_MAX_DURATION");
  	}

    if(sd_debug)
    {
      unsigned int log_count = 0;
      while ( SD.exists( (String("log") + String(log_count)).c_str() ) )
        log_count++;
      File data_file = SD.open((String("log") + String(log_count)).c_str(), FILE_WRITE);
      Serial.println((String("log") + String(log_count)).c_str());
      if(data_file)
      {
        digitalWrite(5, HIGH);
        Serial.println("open achieved");
        String stream = "";
        for( unsigned int i = 0 ; i < sample_num ; i++ )
        {
          stream += String(i);
          stream += String("\t");
          stream += String(samples[i].x);
          stream += String("\t");
          stream += String(samples[i].y);
          stream += String("\t");
          stream += String(samples[i].z);
          stream += String("\t");
          stream += String(samples[i].command_x);
          stream += String("\t");
          stream += String(samples[i].command_y);
          stream += String("\t");
          stream += String(samples[i].command_z);
          stream += String("\n");
        }
        data_file.print(stream);
        delay(100);
        data_file.close();
        Serial.println(stream);
      }
      while(true);
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
