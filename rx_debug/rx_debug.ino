#include <VirtualWire.h>


struct Sample
{
  float x;
  float y;
  float z;
  float command_x;
  float command_y;
  float command_z;
};

Sample sample;
byte taille_message = sizeof(Sample);

unsigned int id = 0;

void setup() 
{
  id = 0;
  Serial.begin(9600); 
  Serial.println("#Start listening for radio debug"); 

  vw_setup(1200);
  vw_rx_start(); 
}

void loop() 
{
  if (vw_get_message( (byte *)&sample, &taille_message  )) 
  {
    Serial.print(id); Serial.print("\t");
    Serial.print(sample.x, 2); Serial.print("\t");
    Serial.print(sample.y, 2); Serial.print("\t");
    Serial.print(sample.z, 2); Serial.print("\t");
    Serial.print(sample.command_x, 2); Serial.print("\t");
    Serial.print(sample.command_y, 2); Serial.print("\t");
    Serial.print(sample.command_z, 2); Serial.print("\n");
    id++;

  }
}
