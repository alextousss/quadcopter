#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>
#include <SD.h>

#include "datastructs.hpp"
#include "IMUsensor.hpp"
#include "stabilitycontrol.hpp"
#include "motormanager.hpp"


#define NUM_STATES 500
#define PIN_CS 10

typedef struct State {
    vec3f ori;
    vec3f rot;
    vec3f rot_filtered;
    vec3f cmd;
    vec3f p;
    vec3f d;
    //float height;
    unsigned long t; // microseconds
} State;



float X_K_P = 15.0f*0.2f;
float X_K_D = 15.0f*0.2f*0.076f;
#define Ku 0.0125f
#define Tu 0.240f // 240ms
#define K_P (0.6f*Ku)
#define K_D (0.075f*Ku*Tu)

void compute_commands(State *states, int idx){
    if(idx == 0) {
        states[idx].cmd = {0.0f, 0.0f, 0.0f};
        return;
    }
    float dt = (states[idx].t-states[idx-1].t)*1e-6f;
    vec3f ncsigne;
    ncsigne.x = X_K_P*-states[idx].ori.x - X_K_D*states[idx].rot_filtered.x;
    ncsigne.y = X_K_P*-states[idx].ori.y - X_K_D*states[idx].rot_filtered.y;

    vec3f ocsigne;
    ocsigne.x = X_K_P*-states[idx-1].ori.x - X_K_D*states[idx].rot_filtered.x;
    ocsigne.y = X_K_P*-states[idx-1].ori.y - X_K_D*states[idx].rot_filtered.y;


    states[idx].p.x = K_P*(ncsigne.x-states[idx].rot_filtered.x);
    states[idx].p.y = K_P*(ncsigne.y-states[idx].rot_filtered.y);
    states[idx].d.x = K_D*((ncsigne.x-states[idx].rot_filtered.x)-(ocsigne.x-states[idx-1].rot_filtered.x))/dt;
    states[idx].d.y = K_D*((ncsigne.y-states[idx].rot_filtered.y)-(ocsigne.y-states[idx-1].rot_filtered.y))/dt;

    states[idx].cmd.x =  states[idx].p.x + states[idx].d.x;
    states[idx].cmd.y =  states[idx].p.y + states[idx].d.y;
    states[idx].cmd.z = 0.0f;
}




bool wait_serial = 1;
bool serial_debug = 1;
bool radio_debug = 0;
bool sd_debug = 0;


State states[NUM_STATES];

float desired_height = 0;

unsigned long sonar_timer = 0;



MotorManager motors;
NewPing sonar(6,5, 500);
IMUsensor mpu;

void setup()
{
    sonar_timer = millis();
    pinMode(5, OUTPUT);

    Serial.begin(115200);
    if(wait_serial)
        while(!Serial); //on attends que le port série soit ouvert pour commencer les calculs


    //  pinMode(9, INPUT_PULLUP); //on configure les entrées pour pouvoir utiliser le bouton
    motors.startMotors();
    mpu.calibrateSensors();


    if(sd_debug) {
        Serial.print("Initialisation de la carte SD ...");
        //On regarde si la carte est présente
        if (!SD.begin(PIN_CS))  {
            Serial.println("Pas possible d'initialiser la carte SD");
            while(true);
        }
        Serial.println("Carte SD initialisée.");
    }
    digitalWrite(5, LOW);
}


void loop()
{
    digitalWrite(5,LOW);
    motors.setOn();
    mpu.resetOrientation();
    unsigned long millis_at_start = millis();
    for(unsigned int zero = 0; zero < 20 ; zero++) {
        if(zero > 0) {
            states[0] = states[NUM_STATES-1];
        }
        for(unsigned int idx = 1; idx < NUM_STATES ; idx++) {
            while(micros()-states[idx-1].t<2000);
            states[idx].t = micros();
            mpu.actualizeSensorData(); //on actualise les capteurs et on calcule l'orientation
            mpu.calcAbsoluteOrientation(0.995);
            states[idx].ori = mpu.getOrientation();
            states[idx].rot = mpu.getRotation();

            float dt = (states[idx].t-states[idx-1].t)*1e-6f;
            const float beta = exp(-2.0f*3.14f*25.0f*dt); // filtrage passe-bas 200hz
            states[idx].rot_filtered.x = beta*states[idx-1].rot_filtered.x + (1.0f-beta)*states[idx].rot.x;

            states[idx].rot_filtered.y = beta*states[idx-1].rot_filtered.y + (1.0f-beta)*states[idx].rot.y;

            compute_commands(states, idx);
            motors.command( states[idx].cmd.x, states[idx].cmd.y, states[idx].cmd.z, 0.3f ); //commande des moteurs avec les valeurs données par le PID
            vec4f motor_value = motors.getMotorValues();
            //if(idx%1==0) {
            Serial.print(states[idx].ori.x, 4) + Serial.print("\t");
            Serial.print(states[idx].ori.y, 4) + Serial.print("\t");
            Serial.print(states[idx].rot.x, 4) + Serial.print("\t");
            Serial.print(states[idx].rot_filtered.x, 4) + Serial.print("\t");
            Serial.print(states[idx].cmd.x, 4) + Serial.print("\t");
            Serial.print(states[idx].p.x, 4) + Serial.print("\t");
            Serial.print(states[idx].d.x, 4) + Serial.print("\t");
            Serial.print(motor_value.x, 4) + Serial.print("\t");
            Serial.print(motor_value.z, 4) + Serial.print("\t");
            Serial.println("");
            //}
        }
    }
    motors.setOff();
/*
    String stream = "t (micros)\tori_x\trot_x\tcmd_x\tm_0\tm_1\tm_2\tm_3\n";
    for( unsigned int i = 0 ; i < NUM_STATES; i++ ) {
        stream += String(states[i].t) + String("\t");
        stream += String(states[i].ori.x) + String("\t");
        stream += String(states[i].rot.x) + String("\t");
        stream += String(states[i].cmd.x) + String("\t");
        stream += String(states[i].motor_value.x) + String("\t");
        stream += String(states[i].motor_value.y) + String("\t");
        stream += String(states[i].motor_value.z) + String("\t");
        stream += String(states[i].motor_value.h) + String("\t");

        stream += String("\n");

        if(serial_debug) {
            Serial.print(stream);
        }
        stream = "";
    }
    */
    /*
       if(sd_debug) {
       unsigned int file_count = 0;
       while (SD.exists((String("test") + String(file_count)).c_str()))
       file_count++;
       File data_file = SD.open((String("test") + String(file_count) ).c_str(), FILE_WRITE);

       if(!data_file) {
       Serial.println("erreur en ouvrant le fichier!");
       } else {
       digitalWrite(5, HIGH);
       Serial.println("open achieved");

       data_file.print(stream);
       delay(100);
       data_file.close();
       Serial.println(stream);
       }
       }
       */
    while(true); // test effectué c'était énorme on s'arrête maintenant
}
