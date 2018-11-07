// I²C AIGUILLAGE ET ELECTRO-AIMANTS
// Commande d'aiguillage par protocole I²C
// PBA 2018-04-09

// Ce programme peut fonctionner avec des sorties multiplexées ou non multiplexées
// En mode non multiplexé, on peut adresser 8x8 = 64 bobines -> 32 aiguillages
// En mode multiplexé, on peut installer 2, 3 ou 4 multiplexeurs selon les besoins
// 2 multiplexeurs : 8x8 = 64 bobines -> 32 aiguillages
// 3 multiplexeurs : 8x16 = 128 bobines -> 64 aiguillages
// 4 multiplexeurs : 16x16 = 256 bobines -> 128 aiguillages
// Supprimer le commentaire devant le define qui suit pour activer le mode multiplexé

// #define MUXMODE // Active le mode multiplexé

#include <Wire.h>
#include <EEPROM.h>

#define I2C_ADDRESS 0x71 // Adresse par défaut du périphérique
#define DATASIZE 4 // Taille maximum des données à recevoir
#define NUM_AIG 32 // Nombre d'aiguillages à gérer
#define EA_PULSE 300 // Durée de l'impulsion envoyée aux électro-aimants
#define STACK_SIZE 256 // Taille de la pile de stockage
#define EEPROM_ADDR 256 // Emplacement de l'adresse I²C dans la NVRAM

// assignation de la matrice aux sorties de l'Arduino
#ifdef MUXMODE
// Mode multiplexé
byte eaRows[4]={2,3,4,5};
byte eaRowEnable=6;
byte eaCols[4]={7,8,9,10};
byte eaColEnable=11;
#else
// Mode non multiplexé
byte eaRows[8]={2,3,4,5,6,7,8,9};
byte eaCols[8]={10,11,12,13,A0,A1,A2,A3};
#endif


typedef struct _progStruct
{
  byte dir_col; // Voie directe
  byte dir_row;
  byte dev_col; // Voie déviée
  byte dev_row;
} progStruct;

progStruct progTable[NUM_AIG]; // Table de programmation des aiguillages
byte aigPos[NUM_AIG]; // Position des aiguillages
byte aigStack[STACK_SIZE]; // Stockage des aiguilles à commander
byte dataSize; // Taille des données reçues
byte stackIn,stackOut; // Indexes de bas et sommet de pile
word cmd; // Commande en cours

#ifdef MUXMODE
void eaSwitch(byte rowCol)
// Bascule un aiguillage (mode multiplexé)
{
  byte row = rowCol>>4;
  byte col = rowCol&0xF;
  digitalWrite(eaRows[0],(bool)(row&1));
  digitalWrite(eaRows[1],(bool)(row&2));
  digitalWrite(eaRows[2],(bool)(row&4));
  digitalWrite(eaRows[3],(bool)(row&8));
  digitalWrite(eaCols[0],(bool)(col&1));
  digitalWrite(eaCols[1],(bool)(col&2));
  digitalWrite(eaCols[2],(bool)(col&4));
  digitalWrite(eaCols[3],(bool)(col&8));

  digitalWrite(eaRowEnable,HIGH); // Activation d'une ligne
  digitalWrite(eaColEnable,HIGH); // et d'une colonne
  delay(EA_PULSE);
  digitalWrite(eaRowEnable,LOW);  // Désactivation
  digitalWrite(eaColEnable,LOW);
}
#else
void eaSwitch(byte rowCol)
// Bascule un aiguillage (mode non multiplexé)
{
  byte row = rowCol>>4;
  byte col = rowCol&0xF;
  for(byte i=0; i<8; i++) // Mise à zéro des sorties
  {
    digitalWrite(eaRows[i],LOW);
    digitalWrite(eaCols[i],LOW);
  }
  
  digitalWrite(eaRows[row],HIGH); // Activation d'une ligne
  digitalWrite(eaCols[col],HIGH); // et d'une colonne
  delay(EA_PULSE);
  digitalWrite(eaRows[row],LOW);  // Désactivation
  digitalWrite(eaCols[col],LOW);
}
#endif

void storePos(byte rowCol) // Stocke un ordre dans la pile
{
  aigStack[stackIn++]=rowCol;
// On ne se donne pas la peine de tester la saturation de la pile
// de toute façon des données seront perdues
}

void setPos(byte aigNum,byte dirDev) // Convertit d'adresse de l'aiguillage et stocke l'ordre
{
  byte row,col;
  if(dirDev) // voie déviée
  {
    row = progTable[aigNum].dev_row;
    col = progTable[aigNum].dev_col;
    aigPos[aigNum]=1;
  }
  else // voie directe
  {
    row = progTable[aigNum].dir_row;
    col = progTable[aigNum].dir_col;
    aigPos[aigNum]=0;
  }
  if(!(row&0xF8)&&!(col&0xF8)) // teste si l'aiguillage est programmé
    storePos((row<<4)|col);
}

byte getPos(word param=0x100) // Retourne la position de l'aiguillage
{
  static byte aigNum; // Passage de paramètre entre la requête et la réponse
  if(param==0x100)
    return aigPos[aigNum];
  aigNum=param; // stockage du paramètre pour la réponse
  return 0;
}

void setProg(byte aigNum, byte dir, byte dev) // Change la programmation de l'aiguillage
{
  progTable[aigNum].dev_row = dev>>4;
  progTable[aigNum].dev_col = dev&0xF;
  progTable[aigNum].dir_row = dir>>4;
  progTable[aigNum].dir_col = dir&0xF;

  byte* tablePtr = (byte*)&progTable[aigNum]; // Pointeur sur un aiguillage dans la table
  int tableIndex = aigNum*sizeof(progStruct);
  byte i;
  for(i=0; i<sizeof(progStruct); i++)
  {
    EEPROM.write(tableIndex++,*tablePtr++); // Copie directe via le pointeur
  }
}

byte getProg(word param) // Retourne la programmation de l'aiguillage
{
  static byte aigNum; // Passage de paramètre entre la requête et la réponse
  byte row,col;
  if(param==0x100) // voie déviée
  {
    row = progTable[aigNum].dev_row;
    col = progTable[aigNum].dev_col;
    return (row<<4)|col;
  }
  else if(param==0x200) // voie directe
  {
    row = progTable[aigNum].dir_row;
    col = progTable[aigNum].dir_col;
    return (row<<4)|col;
  }
  aigNum=param; // stockage du paramètre pour la réponse
  return 0;
}

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
    EEPROM.write(EEPROM_ADDR, addr);
    Wire.begin(addr); // On change l'adresse du périphérique I²C au vol
  } 
}

byte readData(byte* data,byte dataSize)
// Lit des données sur l'entrée I²C
{
  byte i;
  for(i=0; (i<dataSize)&&Wire.available(); i++) // Tant que des données sont disponibles
    *data++=Wire.read(); // Lit les données
  return i; // Nombre d'octets lus
}

void wireReceiveEventFunction()
// Fonction de réception
// Lit la commande passée en paramètre et aiguille vers le bon traitement
{
  byte data[DATASIZE];
  byte dataSize;

  if(!Wire.available()) return; // Pas de données à lire
  digitalWrite(LED_BUILTIN, HIGH); // LED allumée = réception
  cmd = Wire.read(); // Lecture de la commande
  dataSize=readData(data,DATASIZE);
  switch(cmd)
  {
    case 0x00 : // Mode service (commun à tous les circuits)
      if(dataSize>0) serviceMode(data,dataSize);
      break;
    case 0x30 : // Déplace une aiguille en voie directe : identifiant de l'aiguillage
      if(dataSize==1) setPos(data[0],0);
      break;
    case 0x31 : // Déplace une aiguille en voie déviée : identifiant de l'aiguillage
      if(dataSize==1) setPos(data[0],1);
      break;
    case 0x32 : // Déplace une aiguille : identifiant de l'aiguillage (le bit de poids fort indique la direction)
      if(dataSize==1) setPos(data[0]&0x7F,!!data[0]&80);
      break;
    case 0x33 : // Retourne la position actuelle de l'aiguillage (sauf manip manuelle) : identifiant de l'aiguillage
      if(dataSize==1) getPos(data[0]);
      break;
    case 0x34 : // Programmation de la configuration : identifiant de l'aiguillage + Row/Col voie directe + Row/Col voie déviée
      if(dataSize==3) setProg(data[0],data[1],data[2]);
      break;
    case 0x35 : // Demande de lecture de la configuration : identifiant de l'aiguillage
      if(dataSize==2) getProg(data[0]);
      break;
    case 0x36 : // Adressage direct : RC = Row/Col
      if(dataSize==1) storePos(data[0]); // Stockage direct dans la pile
      break;
    default :
      cmd = 0; // commande inconnue
      while(Wire.available()) Wire.read(); // Lecture des données pour purger le buffer
  }
}

void wireRequestEventFunction()
// Fonction d'émission
// Retourne des données en fonction de la commande active
{
  digitalWrite(LED_BUILTIN, LOW); // LED éteinte = émission
  switch(cmd)
  {
    case 0x100 :
      Wire.write("AIEA"); // Identifiant AIEA : AIguillage Electro Aimant
      break;
    case 0x33 : // retourne la position
      Wire.write(getPos());
      break;
    case 0x35 : // retourne la configuration
      Wire.write(getProg(0x100));
      Wire.write(getProg(0x200));
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
  byte addr=EEPROM.read(EEPROM_ADDR); // Lit l'adresse programmée dans la NVRAM
  if((addr&0xF0==0)||(addr&0x80)) addr=I2C_ADDRESS; // Adresses interdites -> Adresse par défaut
  Wire.begin(addr); // Initialisation en mode esclave
  Wire.onReceive(wireReceiveEventFunction); // Événement réception
  Wire.onRequest(wireRequestEventFunction); // Événement émission
  pinMode(LED_BUILTIN, OUTPUT);
  ledDisplay(addr); // Affiche l'adresse active en faisant clignoter la LED

  byte i;
  for(i=0; i<8; i++) // Initialisation des sorties
  {
    pinMode(eaRows[i],OUTPUT); // Toutes les lignes de la matrice en sortie
    digitalWrite(eaRows[i],LOW);
    pinMode(eaCols[i],OUTPUT);
    digitalWrite(eaCols[i],LOW);
  }

  byte* tablePtr = (byte*)progTable; // Pointeur sur la table
  for(i=0; i<sizeof(progTable); i++)
  {
    *tablePtr++=EEPROM.read(i); // Copie directe via le pointeur
  }
  stackIn=stackOut=0;

  for(i=0; i<NUM_AIG; i++)  // Initialise tous les aiguillages
    setPos(i,0);

  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  if(stackIn!=stackOut) // Il y a un aiguillage à manoeuvrer
    eaSwitch(aigStack[stackOut++]);
  else
    delay(100); // Rien à faire
}

