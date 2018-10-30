// I²C ECHO
// Recoit un paquet de données (16 octets maximum)
// Stocke le paquet puis le retourne à la demande
// PBA 2017-02-10

#include <EEPROM.h>
#include<Wire.h>
#define DATASIZE 16 // Trames de 16 octets maximum

byte data[DATASIZE]; // stockage des données reçues
byte dataSize;
word cmd;

// -------------------------------------------------------

void serviceMode(byte* data,byte inSize)
// Le mode service permet de piloter la configuration par les outils DiCiCino-multi
// Toute commande commençant par 00 est considérée comme étant en mode service
// Les deux commandes disponibles sont les suivantes :
// 00 FF 00 FF : retourne l'identifiant du circuit sur 4 caractères
// 00 adr FF /adr : change l'adresse I²C du circuit pour l'adresse adr (/adr = complément à un de adr)
{
  // Retourne l'identifiant
  if((inSize==3)&&(data[0]==0xFF)&&(data[1]==0)&&(data[2]==0xFF))
  {
    cmd=0x100;
    return;
  }
  // Changement d'adresse
  if((inSize==3)&&(data[1]==0xFF)&&(data[0]=~data[2]))
  {
    // L'adresse doit être spécifiée au format 7 bits
    byte addr=data[0];
    if((addr<0x10)||(addr>0x7E)) return; // Adresses I²C interdites
    EEPROM.write(0, addr);
    Wire.begin(addr); // On change l'adresse du périphérique I²C au vol
  } 
}

void wireReceiveEventFunction()
// Fonction de réception
// Lit la commande passée en paramètre et aiguille vers le bon traitement
{
  byte i;
  byte serviceData[4];
  if(!Wire.available()) return; // Pas de données à lire
  digitalWrite(LED_BUILTIN, HIGH); // LED allumée = réception
  cmd = Wire.read(); // Lecture de la commande
  switch(cmd)
  {
    case 0x00 : // Mode service : commun à tous les circuits
      i=0;
      while(Wire.available()) // Tant que des données sont disponibles
      {
        serviceData[i++]=Wire.read(); // Lecture des données
        if(i==sizeof(serviceData)) break;
      }
      serviceMode(serviceData,i);
      break;
    case 0x11 : // lecture
      if(!Wire.available()) // Commande incomplète
        return;
      dataSize = Wire.read(); // lecture de la taille des données
      break; // Rien de plus à faire : le reste est dans wireRequestEventFunction
    case 0x22 : // écriture
      if(!Wire.available()) // Commande incomplète
        return;
      dataSize = Wire.read(); // lecture de la taille des données
      i=0;
      while(Wire.available()) // Tant que des données sont disponibles
      {
        if(i<dataSize)
          data[i++]=Wire.read(); // Lecture des données
      }
      dataSize=i;
      break; 
    default :
      cmd = 0; // commande inconnue
      while(Wire.available()) // Lecture des données pour purger le buffer
        Wire.read();
  }
}

void wireRequestEventFunction()
// Fonction d'émission
// Retourne des données en fonction de la commande active
{
  digitalWrite(LED_BUILTIN, LOW); // LED éteinte = émission
  switch(cmd)
  {
    case 0x11 : // lecture
      Wire.write(data,dataSize);
      break;
    case 0x100 :
      Wire.write("ECHO"); // Mode service : retourne l'identifiant du circuit
      break;
    default :
      Wire.write(0);
  }
  cmd=0;
}

void ledDisplay(byte data)
// Affiche l'octet passé en paramètre sous forme de clignotements de la LED
{
  for(int i=0; i<8; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(data&0x80?400:100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    data<<=1;
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  byte addr=EEPROM.read(0); // Lit l'adresse programmée dans la NVRAM
  if((addr&0xF0==0)||(addr&0x80)) addr=0x70; // Adresses interdites -> Adresse par défaut
  Wire.begin(addr); // Initialisation en mode esclave
  Wire.onReceive(wireReceiveEventFunction); // Événement réception
  Wire.onRequest(wireRequestEventFunction); // Événement émission
  pinMode(LED_BUILTIN, OUTPUT);
  ledDisplay(addr); // Affiche l'adresse active en faisant clignoter la LED
}

void loop()
{
  delay(1000); // Rien à faire à part attendre les événements
}


