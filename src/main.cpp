#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>
#include <SD.h>

#include "datastructs.hpp"
#include "IMUsensor.hpp"
#include "stabilitycontrol.hpp"
#include "motormanager.hpp"

#define PRINT_PERIOD 20
#define MOTOR_MAX_DURATION 10000
#define PAUSE_BETWEEN_TESTS 7000
#define MAX_SAMPLE_BUFFER_SIZE  100
#define SAMPLE_PERIOD 10



struct Sample
{
  float x;
	float command_x;
	float proportional_x;
	float integral_x;
	float derivate_x;
};

Sample samples[MAX_SAMPLE_BUFFER_SIZE];


bool safe_mode = 1;             //si activé, les moteurs se coupent automatiquement après 3 secondes d'allumage
bool wait_serial = 0;                // et ce afin d'éviter une perte de contrôle du quadricoptère sur le banc de test
bool serial_debug = 1;
bool radio_debug = 0;
bool sd_debug = 0;


unsigned long millis_at_motor_start = 0;
unsigned long millis_at_last_print = 0;
unsigned long millis_at_last_loop = 0;
unsigned long millis_at_last_max_time_loop = 0;
unsigned long millis_at_last_test_end = 0;
unsigned long time_loop = 0;
unsigned long max_time_loop = 0;

float desired_height = 0;
float sonar_height = 0;
float sonar_speed = 0;

float last_sonar_height = 0;
unsigned long time_at_last_sonar_height = 0;

unsigned int sample_num = 0;
unsigned int sample_id = 0;
unsigned int test_id = 0;
unsigned int folder_count = 0;


const unsigned short chip_select = 10;

StabilityControl controller;        // objet qui gère le calcul des directives pour les moteurs
MotorManager motors;            // objet qui gère le calcul des valeurs par moteur, et s'occupe de les contrôler
IMUsensor mpu;                  // objet pour récupérer les valeurs de l'IMU et calculer une orientation absolue

void setup()
{
  pinMode(5, OUTPUT);

  Serial.begin(115200);
  if(wait_serial)
    while(!Serial); //on attends que le port série soit ouvert pour commencer les calculs


//  pinMode(9, INPUT_PULLUP); //on configure les entrées pour pouvoir utiliser le bouton
  motors.startMotors();
  mpu.calibrateSensors();


  if(sd_debug)
  {
    Serial.print("Initialisation de la carte SD ...");

    //On regarde si la carte est présente
    if (!SD.begin(chip_select))
    {
      Serial.println("Pas possible d'initialiser la carte SD");
      while(true);
    }

    Serial.println("Carte SD initialisée.");


    while ( SD.exists( (String("test") + String(folder_count)).c_str() ) )
      folder_count++;
    SD.mkdir ( (String("test") + String(folder_count) ).c_str() );
  }
  digitalWrite(5, LOW);
}


void loop()
{
  digitalWrite(5,LOW);

  millis_at_motor_start = millis();
  motors.setOn();
  mpu.resetOrientation();

  controller.reset();
	controller.setGainX( {20.0f, 0.0f, 0.0f} );
	controller.setGainY( {20.0f, 0.0f, 0.0f} );

  while( !(safe_mode && millis() - millis_at_motor_start > MOTOR_MAX_DURATION) || !(sd_debug && sample_num >= MAX_SAMPLE_BUFFER_SIZE) )
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
    mpu.calcAbsoluteOrientation(0.995);


    //calcul du PID avec les valeurs de l'IMU
    vec4f command = controller.getCommand({ mpu.getX(), mpu.getY(), mpu.getZ(), sonar_height } , { 0, 0, 0, desired_height }, time_loop);


    motors.command( command.x, command.y, command.z, command.h ); //commande des moteurs avec les valeurs données par le PID

    if( sample_id % SAMPLE_PERIOD == 0 )
    {
      samples[sample_num] = { mpu.getX(), command.x, controller.getProportionalCorrection().x, controller.getIntegralCorrection().x, controller.getDerivateCorrection().x };
      sample_num++;
    }

    sample_id++;

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
        Serial.print( command.x, 2 ); Serial.print("\t");
        Serial.print( controller.getProportionalCorrection().x, 2 ); Serial.print("\t");
        Serial.print( controller.getIntegralCorrection().x, 2 ); Serial.print("\t");
        Serial.print( controller.getDerivateCorrection().x, 2 ); Serial.print("\t|\t");
        Serial.print( sonar_height, 2 ); Serial.print("\t");
        Serial.print( max_time_loop ); Serial.print("\n");
      }
      millis_at_last_print = millis();
    }
  }

  Serial.println("Stop of the test !");
  Serial.println("Next test starts in 20 seconds !");

  motors.setOff();

  millis_at_last_test_end = millis();

  if(sd_debug)
  {

    unsigned int file_count = 0;
    while (SD.exists((String("test") + String(folder_count) + String("/") + String("log") + String(file_count)).c_str()))
      file_count++;
    File data_file = SD.open((String("test") + String(folder_count) + String("/") + String("log") + String(file_count) ).c_str(), FILE_WRITE);

    if(data_file)
    {
      digitalWrite(5, HIGH);
      Serial.println("open achieved");
      String stream = "----TEST n°" + String(test_id) + "----\n";
      for( unsigned int i = 0 ; i < sample_num ; i++ )
      {
        stream += String(i);
        stream += String("\t");
        stream += String(samples[i].x);
        stream += String("\t");
        stream += String(samples[i].command_x);
        stream += String("\t");
        stream += String(samples[i].proportional_x);
        stream += String("\t");
        stream += String(samples[i].integral_x);
        stream += String("\t");
        stream += String(samples[i].derivate_x);
        stream += String("\n");
      }
      data_file.print(stream);
      delay(100);
      data_file.close();
      Serial.println(stream);
    }
  }

}
