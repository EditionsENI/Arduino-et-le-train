// I²C AIGUILLAGE ET SERVOMOTEURS
// Commande des servomoteurs par un circuit PCA9685
// PBA 2018-11-10

// L'arduino fait l'interface entre deux bus I²C
// ce qui permet d'effectuer l'abstraction du pilotage des servomoteurs
// et de commander tous les types d'aiguillages de la même façon

// D'un côté les ordres envoyés par le PCC/TCO
// Bus I²C esclave (géré en hardware)
// SDA = A4
// SCL = A5
// Reçoit les ordres depuis DiCiCino-multi ou depuis le PCC/TCO

// De l'autre les commandes de postionnement des servomoteurs
// Bus I²C maître (géré par soft)
// SDA = D6
// SCL = D7
// Commande les circuits PCA9685 pour déplacer les servomoteurs

// Le nombre d'aiguillages gérables est limité à 256,
// !!! Ce qui suit est faux : il n'y a pas assez de RAM pour cela !!!
// mais il peut être porté à 511 si les commandes I²C sont
// modifiées pour accepter un octet supplémentaire,
// mais dans ce cas, l'on pert la compatibilité avec la commande
// par électro-aimants

#include <Wire.h>
#include <EEPROM.h>

// ------------------------------------------------------------
// --- Paramétrages                                         ---
// ------------------------------------------------------------

#define I2C_ADDRESS 0x72 // Adresse par défaut du périphérique
#define DATASIZE 4 // Taille maximum des données à recevoir
#define NUM_AIG 256 // Nombre d'aiguillages à gérer

#define EEPROM_ADDR 1023 // Emplacement de l'adresse I²C dans la NVRAM
#define EEPROM_SPEED 1022 // Emplacement de la vitesse de déplacement dans la NVRAM

// Positions en butée des servos (valeur maxi : 511>>1)
#define SERVO_MIN 120>>1
#define SERVO_MAX 430>>1
// Valeurs par défaut de la position des servos
#define SERVO_DIR 200>>1
#define SERVO_DEV 350>>1

byte moveSpeed=0; // Valeur par défaut de la vitesse de déplacement
word cmd; // Commande en cours

typedef struct _servoParam
{
  byte dir;
  byte dev;
} servoParam;

servoParam servoList[NUM_AIG]; // Positions programmées de chaque aiguillage
byte aigPos[NUM_AIG]; // Position des aiguillages ( 0 = voie directe ; 1 = voie déviée )
byte aigCur[NUM_AIG]; // Position courante de chaque aiguillage ( utilisé pour le déplacement lent )
byte aigDst[NUM_AIG]; // Position de destination de chaque aiguillage

// ------------------------------------------------------------
// --- GESTION D'UN BUS I²C SOFT POUR COMMANDER LES PCA9685 ---
// ------------------------------------------------------------

#define SDA_PORT PORTD
#define SDA_PIN 6 // I²C SDA sur D6
#define SCL_PORT PORTD
#define SCL_PIN 7 // I²C SCL sur D7

// Télécharger la librairie I²C soft à l'adresse suivante :
// https://github.com/felias-fogg/SoftI2CMaster
#include <SoftI2CMaster.h>

byte readByte(byte addr, byte cmd) // Lecture d'un octet sur le bus I²C
{
  i2c_start(addr<<1);
  i2c_write(cmd);
  i2c_rep_start((addr<<1)|1);
  byte val = i2c_read(true);
  i2c_stop();
  return val;
}

void writeByte(byte addr, byte cmd, byte data) // Ecriture d'un octet sur le bus I²C
{
  i2c_start(addr<<1);
  i2c_write(cmd);
  i2c_write(data);
  i2c_stop();
}

// ------------------------------------------------------------
// --- PILOTAGE DES AIGUILLAGES                             ---
// ------------------------------------------------------------

void servoInit(byte addr) // Initialisation du circuit -> à faire pour chaque PCA9685 connecté
{
  writeByte(addr|0x40, 0, 0x80);
  delay(10);
  byte oldmode = readByte(addr|0x40, 0);
  byte sleepMode = (oldmode&0x7F) | 0x10;
  writeByte(addr|0x40, 0, sleepMode);
  writeByte(addr|0x40, 0xFE, 135); // réglage à 50 Hz
  writeByte(addr|0x40, 0, oldmode);
  delay(5);
  writeByte(addr|0x40, 0, oldmode | 0xa0);  //  auto increment
}

void servoPos(byte addr, byte servo, byte pos) // positionne un servomoteur
{
  i2c_start((addr|0x40)<<1);
  i2c_write(6+4*servo);
  i2c_write(0);
  i2c_write(0);
  i2c_write(pos<<1); // Les postions sont stockés sur un octet, mais elles varient entre 200 et 350 -> multiplication par 2
  i2c_write(pos>>7); // Le bit de poids fort est le seul à être dans l'octet de poids fort
  i2c_stop();
}

void changePos(byte aig, byte pos) // Positionne l'aiguillage, fonction de bas niveau (pos = SERVO_MIN .. SERVO_MAX)
{
  byte addr=aig>>4;
  byte servo=aig&0xf;
  servoPos(addr, servo, pos);
}

void setPos(byte aigNum,byte pos) // Positionne l'aiguillage, fonction de haut niveau (pos = 0 ou 1)
{
  byte dest=pos?servoList[aigNum].dev:servoList[aigNum].dir;
  if((dest<SERVO_MIN)||(dest>SERVO_MAX)) return; // Position extrême -> aiguillage non programmé (ou mal programmé)
  aigPos[aigNum]=pos;
  aigDst[aigNum]=dest;
  if(!moveSpeed) // Déplacement instantané
  {
    aigCur[aigNum]=dest;
    changePos(aigNum,dest);
  }
}

byte getPos(word param=0x100) // Retourne la position de l'aiguillage (0 = voie directe ; 1 = voie déviée)
{
  static byte aigNum; // Passage de paramètre entre la requête et la réponse
  if(param==0x100)
    return aigPos[aigNum];
  aigNum=param; // Stockage du paramètre pour la réponse
  return 0;
}

void setProg(byte aigNum, byte dir, byte dev) // Change la programmation de l'aiguillage
{
  EEPROM.write((aigNum<<1), dir);
  EEPROM.write((aigNum<<1)+1, dev);
  servoList[aigNum].dir=dir;
  servoList[aigNum].dev=dev;
}

byte getProg(word param) // Lit la programmation de l'aiguillage
{
  static byte aigNum; // Passage de paramètre entre la requête et la réponse
  if(param==0x100) // Voie directe
    return servoList[aigNum].dir;
  else if(param==0x200) // Voie déviée
    return servoList[aigNum].dev;
  aigNum=param; // Stockage du paramètre pour la réponse
  return 0;
}

void setMoveSpeed(byte ms) // Modifie la vitesse de déplacement des aiguilles
{
  if(ms<0xF0)
  {
    EEPROM.write(EEPROM_SPEED, ms);
    moveSpeed=ms;
  }
}

// ------------------------------------------------------------
// --- Communications I²C                                   ---
// ------------------------------------------------------------

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
      if(dataSize==1) setPos(data[0]&0x7F,!!(data[0]&0x80));
      break;
    case 0x33 : // Retourne la position actuelle de l'aiguillage (sauf manip manuelle) : identifiant de l'aiguillage
      if(dataSize==1) getPos(data[0]);
      break;
    case 0x34 : // Programmation de la configuration : identifiant de l'aiguillage + position voie directe + position voie déviée
      if(dataSize==3) setProg(data[0],data[1],data[2]);
      break;
    case 0x35 : // Demande de lecture de la configuration : identifiant de l'aiguillage
      if(dataSize==2) getProg(data[0]);
      break;
    case 0x37 : // Adressage direct : position
      if(dataSize==2) changePos(data[0],data[1]); // Exécution immédiate de l'ordre de déplacement
      break;
    case 0x38 : // Modification de la vitesse de déplacement
      if(dataSize==1) setMoveSpeed(data[0]);
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
      Wire.write("AISV"); // Identifiant AISV : AIguillage SErvo
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

// ------------------------------------------------------------
// --- Initialisations                                      ---
// ------------------------------------------------------------

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
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial);

  i2c_init();
  servoInit(0);
  delay(10);
  
  byte addr=EEPROM.read(EEPROM_ADDR); // Lit l'adresse programmée dans la NVRAM
  if((addr&0xF0==0)||(addr&0x80)) addr=I2C_ADDRESS; // Adresses interdites -> Adresse par défaut

  Wire.begin(addr); // Initialisation en mode esclave
  Wire.onReceive(wireReceiveEventFunction); // Événement réception
  Wire.onRequest(wireRequestEventFunction); // Événement émission
  ledDisplay(addr); // Affiche l'adresse active en faisant clignoter la LED

  byte driverOk[16]; // Tableau pour savoir si chaque PCA9685 présent a été initialisé
  for(byte i=0; i<sizeof(driverOk); i++)
    driverOk[i]=0;

  for(int aig=0; aig<NUM_AIG; aig++)  // Initialise la programmation des aiguillages
  {
    if(!driverOk[aig>>4]) // TODO : initialisation du PCA9685
    {
      servoInit(aig>>4);
      driverOk[aig>>4]=true;
    }
    servoList[aig].dir=EEPROM.read(aig<<1);
    servoList[aig].dev=EEPROM.read((aig<<1)+1);
    if((servoList[aig].dir>=SERVO_MIN)&&(servoList[aig].dir<=SERVO_MAX)&&(servoList[aig].dev>=SERVO_MIN)&&(servoList[aig].dev<=SERVO_MAX))
    {
      setPos(aig,0); // Tous les aiguillages sont placés en voie directe
      delay(20);  // Pour que tous les servos ne se déplacent pas en même temps (surtout en cas de vitesse instantanée)
    }
    else // L'aiguillage n'est pas programmé ou est mal programmé
    {
      servoList[aig].dir=0;
      servoList[aig].dev=0;
      aigPos[aig]=0;
      aigCur[aig]=0;
      aigDst[aig]=0;
    }
  }

  byte ms=EEPROM.read(EEPROM_SPEED); // Lit la vitesse de déplacement programmée
  if(ms<0xF0)
    moveSpeed=ms;
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  if(moveSpeed>0)
  {
    // Déplacement des aiguillages en vitesse lente
    for(int aig=0; aig<NUM_AIG; aig++)
    {
      if(aigCur[aig]!=aigDst[aig]) // Cette aiguille est en cours de déplacement
      {
        if(aigCur[aig]<aigDst[aig])
          aigCur[aig]++;
        else
          aigCur[aig]--;
        changePos(aig,aigCur[aig]);
      }
    }
    delay(moveSpeed); // La vitesse de déplacement dépend du nombre d'aiguillages en mouvement
  }
  else
    delay(1000);
}

