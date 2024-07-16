// AT-PCC
// PBA 2021-10-10

// Ce programme est encore en cours d'évolution
// La version la plus récente est disponible à l'adresse suivante :
// https://github.com/EditionsENI/Arduino-et-le-train/tree/master/V2/arduino/at-pcc

#include <Arduino.h>
#include <Wire.h>
#include <FlexiTimer2.h>
#include <EEPROM.h>

// ----------------------------------------------------
// --- Définition des entrées-sorties
// ----------------------------------------------------

// MX0 - MX3 : sorties multiplexées
// -> JLX1 + JLX2 = Colonnes LED + Clavier
// -> JKX2 + JKX2 = Colonnes clavier

#define MX0 6
#define MX1 7
#define MX2 8
#define MX3 9
#define MXE 10
#define NUM_COL 16

// JKY1 + JKY2 = Lignes clavier (entrées)

// JK1 : port F (A0-A7)
#define JKY11 A0
#define JKY18 A7
#define JKY1DDR DDRF
#define JKY1PORT PORTF
#define JKY1PIN PINF
// JK2 : port K (A8-A15)
#define JKY21 A8
#define JKY28 A15
#define JKY2DDR DDRK
#define JKY2PORT PORTK
#define JKY2PIN PINK

// JLY1 + JLY2 + JLY3 = Lignes LED (sorties)

// JLY1 : port A (D22-D29) (LED RGB rouge)
#define JLY11 22
#define JLY18 29
#define JLY1DDR DDRA
#define JLY1PORT PORTA
#define JLY1PIN PINA
// JLY2 : port C (D30-D37) (LED RGB verte)
#define JLY21 30
#define JLY28 37
#define JLY2DDR DDRC
#define JLY2PORT PORTC
#define JLY2PIN PINC
// JLY3 : port L (D42-D49) (LED RGB bleue)
#define JLY31 42
#define JLY38 49
#define JLY3DDR DDRL
#define JLY3PORT PORTL
#define JLY3PIN PINL

// ----------------------------------------------------
// --- Constantes utilisées pour définir le réseau
// ----------------------------------------------------

#define DIR 0 // Direction des aiguillages
#define DEV 1
#define ON 1 // Etat des détecteurs, des boutons et des autres signaux
#define OFF 0
#define OK 1
#define STOP 0
// Couleurs utilisées par les LED du panneau et par les signaux
#define NOIR 0
#define ROUGE 1
#define VERT 2
#define JAUNE 3
#define BLEU 4
#define VIOLET 5
#define CYAN 6
#define BLANC 7
#define CARRE 8 // correspond à 2 rouges

// ----------------------------------------------------
// ---  Tableau contenant l'état des cantons
// ----------------------------------------------------

#define MAX_CANTON 256
byte canton[MAX_CANTON];

#define setCanton(n,s) canton[(n)]=(s)

#define getCanton(n) (canton[(n)])

// ----------------------------------------------------
// --- Pile d'événements
// ----------------------------------------------------

#define EVSTACK_SIZE 256
word evStack[EVSTACK_SIZE];
byte evStackIn=0;
byte evStackOut=0;

// ----------------------------------------------------
// --- Etat des LED du TCO
// ----------------------------------------------------

#define MAX_LED 48
byte led[MAX_LED]; // 3 groupes de lignes x 16 colonnes x 8 bits par ligne = 384 LED

// Les LED sont accédées sycliquement par la routine sous interruption

#define setLed(n,s) led[(n)>>3]=s?(led[(n)>>3]|(1<<((n)&7))):(led[(n)>>3]&~(1<<((n)&7)))
// Change l'état d'une LED
// n = numéro de LED (0-383) GGCCCCBBB (groupe/colonne/bit)
// s = état de la LED ON / OFF

#define setLedRGB(n,s) setLed(n,(s)>>2);setLed(n+128,((s)>>1)&1);setLed(n+256,(s)&1)
// Change l'état d'une LED RGB
// n = numéro de LED (0-383) GGCCCCBBB (groupe/colonne/bit)
// s = couleur de la LED 

#define getLed(n) ((led[(n)>>3]>>((n)&7))&1)
// Lit l'état d'une LED
// n = numéro du bouton

#define getLedRGB(n) ((getLed(n)<<2)|(getLed(n+128)<<1)|(getLed(n+256)))

// ----------------------------------------------------
// --- Lecture des boutons du TCO
// ----------------------------------------------------

#define MAX_BTN 32
byte btn[MAX_BTN]; // 2 groupes de lignes x 16 colonnes x 8 bits par ligne = 256 boutons
byte btn2[MAX_BTN]; // Etat précédent afin de détecter les changements d'état

// La lecture est effectuée cycliquement par la routine sous interruption

#define getBtn(n) ((btn[(n)>>3]>>((n)&7))&1)
// Lit un bouton
// n = numéro du bouton

// ----------------------------------------------------
// --- Commande des aiguillages
// ----------------------------------------------------

// Commandes I2C
#define CMD_MOVE_DIR 0x30 // Déplace une aiguille en voie directe
#define CMD_MOVE_DEV 0x31 // Déplace une aiguille en voie déviée

#define MAX_AIG 32
byte aig[MAX_AIG]; // 32 x 8 = 256 aiguillages

struct _aig_list_
{
  byte addr; // Adresse I2C de l'interface
  byte offset; // Début du stockage dans le tableau aig
  byte size; // Nombre d'octets utilisés dans le tableau aig
};

// La valeur de size dépend du type de circuit de gestion des aiguillages :
// AT-AIG500 : 32 aiguillages -> 4 octets maximum
// AT-AIG5000 : 60 aiguillages -> 8 octets maximum
// AT-NANO+PCA9685 : 256 aiguillages -> 32 octets maximum
// La valeur de offset correspond à la somme des 'size' des circuits précédents
// -> voir l'exemple ci-dessous

struct _aig_list_ aigList[]
{
  {0x51,0,4}, // circuit AT-AIG500 en adresse 0x51, 4 octets réservés
  {0x52,4,8}, // circuit AT-AIG5000 en 0x52, l'offset 4 permet de commencer après le circuit précédent
  {0x53,12,3} // circuit AT-NANO+PCA9685 en 0x53, l'offset vaut 4+8 pour tenir compte des autres circuits
};

#define aigListSize (sizeof(aigList)/sizeof(*aigList))

void setAig(word n,byte s)
// Change la position d'un aiguillage
// n = numéro de l'aiguillage
// s = position de l'aiguillage
{
  word aigIdx=n>>3;
  byte aigBit=n&7;
  byte index;
  for(index=0; index<aigListSize ; index++)
    if(aigList[index].offset<=aigIdx) break;
  if(index==aigListSize) return; // numéro invalide
  if(aigIdx>aigList[index].offset+aigList[index].size) return; // numéro invalide

  Wire.beginTransmission(aigList[index].addr);
  Wire.write(s?CMD_MOVE_DEV:CMD_MOVE_DIR); // Commande moveDir ou moveDev
  Wire.write(n-(aigList[index].offset<<3)); // Numéro de l'aiguillage à commander
  Wire.endTransmission();

  // Stockage de la position dans le tableau
  aig[aigIdx]=s?(aig[aigIdx]|(1<<aigBit)):(aig[(n)>>3]&~(1<<aigBit));
}

#define getAig(n) ((aig[(n)>>3]>>((n)&7))&1)
// Lit la position d'un aiguillage
// n = numéro de l'aiguillage

// ----------------------------------------------------
// --- Lecture des détecteurs de passage
// ----------------------------------------------------

#define MAX_DET 24
byte det[MAX_DET];
byte det2[MAX_DET]; // Etat précédent afin de détecter les changements d'état

#define DET 0x1000 // Indique un événement détecteur

struct _det_list_
{
  byte addr; // Adresse I2C de l'interface
  byte offset; // Début du stockage dans le tableau det
  byte size; // Nombre d'octets utilisés dans le tableau det
};

struct _det_list_ detList[]
{
  {0x61,0,24}, // détecteurs en adresse 0x61, 24 octets réservés -> 192 détecteurs
};

#define detListSize (sizeof(detList)/sizeof(*detList))

void scanDet(void)
{
  for(byte i=0; i<detListSize; i++)
  {
    byte numBytes=Wire.requestFrom(detList[0].addr,(byte)detList[0].size); // Attend les données
    byte index=detList[0].offset;
    for(byte n=0; (n<numBytes)&&Wire.available(); n++)
    {
      det[index]=Wire.read(); // Lit les données
      byte change=det[index]^det2[index]; // Changement d'état d'un détecteur
      if(change)
      {
        byte button=det[index]&change;
        if(button)  // transition vers présence -> bit à un
        { // Pas de boucle pour optimiser le code
          if(button&1) evStack[evStackIn++]=(index<<3)|DET;
          if(button&2) evStack[evStackIn++]=((index<<3)+1)|DET;
          if(button&4) evStack[evStackIn++]=((index<<3)+2)|DET;
          if(button&8) evStack[evStackIn++]=((index<<3)+3)|DET;
          if(button&16) evStack[evStackIn++]=((index<<3)+4)|DET;
          if(button&32) evStack[evStackIn++]=((index<<3)+5)|DET;
          if(button&64) evStack[evStackIn++]=((index<<3)+6)|DET;
          if(button&128) evStack[evStackIn++]=((index<<3)+7)|DET;
        }
        det2[index]=det[index];
      }
      index++;
    }
  }
}

#define getDet(n) ((det[(n)>>3]>>((n)&7))&1)

// ----------------------------------------------------
// --- Commande des régulateurs
// ----------------------------------------------------

#define CMD_REG_NORM 0x50
#define CMD_REG_SET_SPEED_28 0x51

#define MAX_REG 32
byte reg[MAX_REG];

struct _reg_list_
{
  byte addr; // Adresse I2C de l'interface
  word offset; // Début du stockage dans le tableau reg
  byte size; // Nombre d'octets utilisés dans le tableau reg
};

struct _reg_list_ regList[]
{
  {0x71,0,4},
};

#define regListSize (sizeof(regList)/sizeof(*regList))

void setReg(word n,byte s)
{
  byte addr=0;
  byte ampli;
  for(byte i=0; i<regListSize; i++)
  {
    if(n<regList[i].size)
    {
      addr=regList[i].addr;
      ampli=n;
      break;
    }
    n-=regList[i].offset;
  }
  if(!addr) return; // n est trop élevé

  Wire.beginTransmission(addr);
  if(s)
  {
    Wire.write(CMD_REG_NORM);
    Wire.write(ampli);
  }
  else
  {
    Wire.write(CMD_REG_SET_SPEED_28);
    Wire.write(ampli);
    Wire.write((byte)0); // Arrêt
  }
  Wire.endTransmission();
}

#define getReg(n) ((reg[(n)>>3]>>((n)&7))&1)

// ----------------------------------------------------
// --- Pilotage des signaux
// ----------------------------------------------------

#define CMD_SIG_SET 0x60

struct _sig_list_
{
  byte addr; // Adresse I2C de l'interface
  word offset; // Début du stockage dans le tableau reg
  byte size; // Nombre d'octets utilisés dans le tableau sig
};

struct _sig_list_ sigList[]
{
  {0x81,0,8},
};

#define sigListSize (sizeof(sigList)/sizeof(*sigList))

void setSig(word n,byte s)
{
  byte addr=0;
  byte sigNum;
  for(byte i=0; i<sigListSize; i++)
  {
    if(n<sigList[i].size)
    {
      addr=sigList[i].addr;
      sigNum=n;
      break;
    }
    n-=sigList[i].offset;
  }
  if(!addr) return; // n est trop élevé
  
  Wire.beginTransmission(addr);
  Wire.write(CMD_SIG_SET);
  Wire.write(sigNum);
  Wire.write((byte)s);
  Wire.endTransmission();
}

// ----------------------------------------------------
// --- Interruption
// ----------------------------------------------------

#define BTN 0 // Indique un événement bouton

void interrupt(void)
// Balayage des matrices clavier et LED
{
  static byte count=0;
  byte count1=count+NUM_COL;
  byte count2;

  // Lecture des touches du clavier : inversion des données lues
  btn[count]=~JKY1PIN;
  byte change=btn[count]^btn2[count]; // Changement d'état d'un bouton
  if(change)
  {
    byte button=btn[count]&change;
    if(button)  // transition vers appui -> bit à un
    { // Pas de boucle pour optimiser le code de l'interruption
      if(button&1) evStack[evStackIn++]=(count<<3);
      if(button&2) evStack[evStackIn++]=(count<<3)+1;
      if(button&4) evStack[evStackIn++]=(count<<3)+2;
      if(button&8) evStack[evStackIn++]=(count<<3)+3;
      if(button&16) evStack[evStackIn++]=(count<<3)+4;
      if(button&32) evStack[evStackIn++]=(count<<3)+5;
      if(button&64) evStack[evStackIn++]=(count<<3)+6;
      if(button&128) evStack[evStackIn++]=(count<<3)+7;
    }
    btn2[count]=btn[count];
  }

  btn[count1]=~JKY2PIN;
  change=btn[count1]^btn2[count1]; // Changement d'état d'un bouton
  if(change)
  {
    byte button=btn[count1]&change;
    if(button)  // transition vers appui -> bit à un
    { // Pas de boucle pour optimiser le code de l'interruption
      if(button&1) evStack[evStackIn++]=(count1<<3);
      if(button&2) evStack[evStackIn++]=(count1<<3)+1;
      if(button&4) evStack[evStackIn++]=(count1<<3)+2;
      if(button&8) evStack[evStackIn++]=(count1<<3)+3;
      if(button&16) evStack[evStackIn++]=(count1<<3)+4;
      if(button&32) evStack[evStackIn++]=(count1<<3)+5;
      if(button&64) evStack[evStackIn++]=(count1<<3)+6;
      if(button&128) evStack[evStackIn++]=(count1<<3)+7;
    }
    btn2[count1]=btn[count1];
  }

  // Inhibition des colonnes pendant le changement de valeur des LED
  digitalWrite(MXE,HIGH);
  // Changement de colonne
  count=(count+1)&15;
  count1=count+NUM_COL;
  count2=count1+NUM_COL;

  JLY1PORT=led[count];
  JLY2PORT=led[count1];
  JLY3PORT=led[count2];

  // activation des sorties
  digitalWrite(MX0,count&1);
  digitalWrite(MX1,!!(count&2));
  digitalWrite(MX2,!!(count&4));
  digitalWrite(MX3,!!(count&8));
  digitalWrite(MXE,LOW);
}

// ----------------------------------------------------
// --- Initialisations
// ----------------------------------------------------

void setup()
{
  Wire.begin();

  pinMode(MX0,OUTPUT);
  pinMode(MX1,OUTPUT);
  pinMode(MX2,OUTPUT);
  pinMode(MX3,OUTPUT);
  pinMode(MXE,OUTPUT);
  digitalWrite(MXE,HIGH);
  JKY1DDR=0; // input
  JKY1PORT=0xFF; // pullup
  JKY2DDR=0; // input
  JKY2PORT=0xFF; // pullup
  JLY1DDR=0xFF; // output
  JLY1PORT=0; // off
  JLY2DDR=0xFF; // output
  JLY2PORT=0; // off
  JLY3DDR=0xFF; // output
  JLY3PORT=0; // off

  FlexiTimer2::set(1, 0.001, interrupt);
  FlexiTimer2::start();
}

// ----------------------------------------------------
// EXEMPLE DE DÉFINITION DE RÉSEAU
// ----------------------------------------------------

// Voici le cantonnement géré par le programme qui suit
// Il y a 3 cantons sur la voie principale : 1, 2 et 3
// Un autre canton est embranché par un aiguillage : 22

// -----1----- -----2----- -----3-----
//                        /
//             ----22----/

// Les cantons sont directement identifiés par leur numéro
// Les autres éléments (boutons, led, aiguillages, détecteurs, régulateurs, signaux)
// sont identifiés par un #define qui permet de le pas utiliser le numéro de l'élément
// Cela simplifie l'écriture du programme en le rendant plus lisible
// Dans tous les #define qui suivent le dernier chiffre correspond au numéro
// pour adresser le périphérique

// Pour les aiguillages, on indique les numéros de cantons raccordés côté talon
// la voie directe en premier, le canton côté pointe est ajouté en cas de doublon
#define AIG_2_22 17

// Il y a deux détecteurs par canton, celui d'entrée permet de savoir qu'un canton et occupé
// celui de sortie est placé avant le signal et permet l'arrêt à hauteur de celui-ci
#define DET_E1 21|DET
#define DET_S1 22|DET
#define DET_E2 23|DET
#define DET_S2 24|DET
#define DET_E3 25|DET
#define DET_S3 26|DET
#define DET_E22 31|DET
#define DET_S22 32|DET

// Les signaux sont aussi déclarés
#define SIG_1 31
#define SIG_2 32
#define SIG_3 33
#define SIG_22 34

// Les régulateurs le sont aussi de la même façon
#define REG_1 41
#define REG_2 42
#define REG_3 43
#define REG_22 44

// On paramètre aussi les boutons du PCC
#define BTN_STOP_1 21// Arrêt forcé dans le canton 1
#define BTN_AIG_2 22 // Aiguillage orienté vers le canton 2
#define BTN_AIG_22 23 // Aiguillage orienté vers le canton 22

// et les LED du PCC
#define LED_CANTON1 31
#define LED_CANTON2 32
#define LED_CANTON3 33
#define LED_CANTON22 34
#define LED_CANTON1 35
#define LED_SIG1 41
#define LED_SIG2 42
#define LED_SIG3 43
#define LED_SIG22 44
#define LED_AIG2 51
#define LED_AIG22 52

// La gestion du réseau est effectuée dans la fonction execEvent
// Il s'agit d'un gestionnaire d'événements
// Les fonctions get permettent de tester les conditions
// Les fonctions set permettent de changer d'état

void execEvent(word event)
{
  switch(event)
  {
    case BTN_AIG_2 : // Bascule l'aiguillage vers la voie directe
      setLed(LED_AIG2,ON);
      setLed(LED_AIG22,OFF);
      setAig(AIG_2_22,DIR);
      setSig(SIG_22,CARRE);
      setLed(LED_SIG22,CARRE);
      setCanton(22,CARRE);
      break;
    case LED_AIG22 : // Bascule l'aiguillage vers la voie déviée
      setLed(LED_AIG22,ON);
      setLed(LED_AIG2,OFF);
      setAig(AIG_2_22,DEV);
      setSig(SIG_2,CARRE);
      setLed(LED_SIG2,CARRE);
      setCanton(2,CARRE);
      break;
    case BTN_STOP_1 : // Bouton d'arrêt forcé sur le canton 1
      setSig(SIG_1,CARRE);
      setLed(LED_SIG1,CARRE);
      setCanton(1,CARRE);
      break;
    case DET_E1 :
      break;
    case DET_S1 :
      break;
    case DET_E2 :
      break;
    case DET_S2 :
      if(getCanton(2)==CARRE)
        setReg(REG_2,STOP);
      else
        setReg(REG_2,OK);
      break;
      break;
    case DET_E3 :
      break;
    case DET_S3 :
      break;
    case DET_E22 :
      break;
    case DET_S22 :
      if(getCanton(22)==CARRE)
        setReg(REG_22,STOP);
      else
        setReg(REG_22,OK);
      break;
  }
}

// ----------------------------------------------------
// --- Boucle principale
// ----------------------------------------------------

#define TEST false

void loop()
{
  scanDet(); // Lecture des détecteurs de passage
  while(evStackIn!=evStackOut) // Il y a des événements dans la pile
  {
    if(TEST) // Affichage des événements
    {
      Serial.print(evStack[evStackOut]&0xF000); // type d'événement
      Serial.print(" ");
      Serial.println(evStack[evStackOut]&0xFFF); // numéro de l'événement
    }
    execEvent(evStack[evStackOut++]); // Dépilage des événements
  }
  delay(10);
}


