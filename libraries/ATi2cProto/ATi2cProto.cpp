// AT-I2C-PROTO
// PBA 2021-06-06
// Librairie d'interface pour les périphériques I2C

#include "Arduino.h"
#include "ATi2cProto.h"

#include <EEPROM.h>
#include <Wire.h>

#define I2C_ID 0
#define I2C_ADDR 4
#define CMD_SERVICE 0x00 // Mode service
#define CMD_SERVICE_ID 0xFF // Lecture de l'identifiant du circuit

void ATi2cProto::dump(void)
{
  Serial.print(length);
  Serial.print(" :");
  for(byte i=0; i<length; i++)
  {
    Serial.print(" ");
    char hex[3];
    sprintf(hex,"%02x",data[i]);
    Serial.print(hex);
  }
  Serial.println();
}

byte ATi2cProto::getAddr(void)
// Retourne l'adresse I²C active
{
	return addr;
}

void ATi2cProto::receive(void)
// réception de données : lit la commande passée en paramètre
{
  if(!Wire.available()) return; // Pas de données à lire
  command=0;
  length=0;

  while(Wire.available()&&(length<sizeof(data)))
    data[length++]=Wire.read(); // Lecture des données dans le buffer
  while(Wire.available())
    Wire.read(); // Lecture des données en excès

  if(data[0]==CMD_SERVICE)
  {
    // Mode service : configuration du circuit
    // Les deux commandes disponibles sont les suivantes :
    // 00 FF 00 FF : retourne l'identifiant du circuit sur 4 caractères
    // 00 adr FF /adr : change l'adresse I2C du circuit pour l'adresse adr (/adr = complément à un de adr)
    if((length==4)&&(data[1]==0xFF)&&(data[2]==0)&&(data[3]==0xFF))
    {
      // Retourne l'identifiant lors de la prochaine lecture de données
      command=CMD_SERVICE_ID;
      return;
    }
    if((length==4)&&(data[2]==0xFF)&&(data[1]=~data[3]))
    {
      // Changement d'adresse
      // L'adresse doit être spécifiée au format 7 bits
      byte newAddr=data[1];
      if((newAddr&0xF0==0)||(newAddr&0x80)) return; // Adresses interdites
      addr=newAddr;
      EEPROM.write(I2C_ADDR, addr);
      Wire.begin(addr); // On change l'adresse du périphérique I2C au vol
    } 
  }
  else
  {
    atReceive(data,length);
  }
}

void ATi2cProto::request(void)
{
  switch(command)
  {
    case CMD_SERVICE_ID :
      Wire.write(id,sizeof(id)); // Mode service : retourne l'identifiant du circuit
      break;
    default :
      byte out[32];
       byte size=atRequest(data,length,out);
      if(size)
        Wire.write(out,size);
  }
  command=0;
}

ATi2cProto* ATi2cProto_this;

static void ATi2cProto::receiveStatic(void)
{
  ATi2cProto_this->receive();
}

static void ATi2cProto::requestStatic(void)
{
  ATi2cProto_this->request();
}

ATi2cProto::ATi2cProto(char _id[4],byte defAddr,void (*init)(),void(*rcv)(byte* data, byte length), byte(*req)(byte* data, byte length, byte* out))
{
	addr=0;
  memcpy(id,_id,sizeof(id));

  if((EEPROM.read(I2C_ID)!=id[0])||(EEPROM.read(I2C_ID+1)!=id[1])||(EEPROM.read(I2C_ID+2)!=id[2])||(EEPROM.read(I2C_ID+3)!=id[3]))
  {
    // L'EEPROM n'est pas encore initialisée
    addr=defAddr;
    EEPROM.put(I2C_ID,id);
    EEPROM.write(I2C_ADDR,addr);
    if(init) init(); // Initialisation du contenu de l'EEPROM
  }
  else
  {
    addr=EEPROM.read(4);
  }
  if((addr&0xF0==0)||(addr&0x80)) return; // Adresses interdites

  atReceive=rcv;
  atRequest=req;
  Wire.begin(addr); // Initialisation en mode esclave
  ATi2cProto_this=this; // pointeur sur l'objet créé pour permettre l'accès depuis les méthodes statiques
  Wire.onReceive(receiveStatic); // Événement réception
  Wire.onRequest(requestStatic); // Événement émission
}
