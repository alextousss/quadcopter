#include <VirtualWire.h>

int nombre_recu = 0;
int pos = 0;
char Message[VW_MAX_MESSAGE_LEN];


void setup() // Fonction setup()
{
    Serial.begin(9600); // Initialisation du port série pour avoir un retour sur le serial monitor
    Serial.println("Tuto VirtualWire"); // Petit message de bienvenue

    vw_setup(1200); // initialisation de la librairie VirtualWire à 2000 bauds (note: je n'utilise pas la broche PTT)
    vw_rx_start();  // Activation de la partie réception de la librairie VirtualWire
}

void loop() // Fonction loop()
{
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;

  if (vw_get_message(buf, &buflen)) // Non-blocking
  {
    int i;
    Serial.print("Quadricoptère : ");
    for (i = 0; i < buflen; i++)
    {
      if(i == 4 || i == 5)
      {
        Serial.print(buf[i] - 127, DEC);

      }
      else
      {
        Serial.print(buf[i], DEC);
      }

      Serial.print('\t');
    }
    Serial.println();
  }
}
