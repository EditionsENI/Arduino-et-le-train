// AT-PCC

// Ce programme est encore en cours d'évolution
// La version la plus récente est disponible à l'adresse suivante :
// https://github.com/EditionsENI/Arduino-et-le-train/tree/master/V2/arduino/at-pcc

#include <Arduino.h>
#include <Wire.h>
#include <FlexiTimer2.h>
#include <EEPROM.h>

// --- Définition des entrées-sorties ---

// MX0 - MX3 : sorties multiplexées
// -> JLX1 + JLX2 = Colonnes LED + Clavier
// -> JKX2 + JKX2 = Colonnes clavier

#define MX0 6
#define MX1 7
#define MX2 8
#define MX3 9
#define MXE 10

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

// JLY1 : port A (D22-D29)
#define JLY11 22
#define JLY18 29
#define JLY1DDR DDRA
#define JLY1PORT PORTA
#define JLY1PIN PINA
// JLY2 : port C (D30-D37)
#define JLY21 30
#define JLY28 37
#define JLY2DDR DDRC
#define JLY2PORT PORTC
#define JLY2PIN PINC
// JLY3 : port L (D42-D49)
#define JLY31 42
#define JLY38 49
#define JLY3DDR DDRL
#define JLY3PORT PORTL
#define JLY3PIN PINL

// --- Etats --

#define DIR 0
#define DEV 1
#define ON 1
#define OFF 0

// ---  Tableaux contenant l'état des périphériques ---

#define MAX_AIG 32 // Nombre de groupes de 8 aiguillages à gérer
#define NUM_AIG 3 // Nombre de circuits de gestion des aiguillages

byte led[3][4]; // 3 groupes de lignes x 4 colonnes x 8 bits par ligne
byte key[2][4]; // 2 groupes de lignes x 4 colonnes x 8 bits par ligne
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

struct _aig_list_ aigList[3]
{
  {0x51,0,4}, // circuit AT-AIG500 en adresse 0x51, 4 octets réservés
  {0x52,4,8}, // circuit AT-AIG5000 en 0x52, l'offset 4 permet de commencer après le circuit précédent
  {0x53,12,3} // circuit AT-NANO+PCA9685 en 0x53, l'offset vaut 4+8 pour tenir compte des autres circuits
};

#define aigListSize (sizeof(aigList)/sizeof(*aigList))

// --- Macros pour un accès simplifié aux valeurs stockées ---

#define setLed(n,s) led[0][(n)>>3]=s?(led[0][(n)>>3]|(1<<((n)&7))):(led[0][(n)>>3]&~(1<<((n)&7)))
// Change l'état d'une LED
// n = numéro de LED (0-383) GGCCCCBBB (groupe/colonne/bit)
// s = état de la LED 0=éteinte / 1=allumée 

#define getLed(n) ((led[0][(n)>>3]>>((n)&7))&1)
// Lit l'état d'une LED
// n = numéro du bouton

// TODO : LED RGB

#define getKey(n) ((key[0][(n)>>3]>>((n)&7))&1)
// Lit un bouton
// n = numéro du bouton

#define getAig(n) (aig[(n)>>3]>>((n)&7))&1)
// Lit la position d'un aiguillage
// n = numéro de l'aiguillage

// --- Fonctions d'accès aux circuits périphériques ---

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
  if(index<aigListSize) return; // numéro invalide
  if(aigIdx>aigList[index].offset+aigList[index].size) return; // numéro invalide

  Wire.beginTransmission(aigList[index].addr);
  Wire.write(s?0x31:0x30); // Commande moveDir ou moveDev
  Wire.write(n-(aigList[index].offset<<3)); // Numéro de l'aiguillage à commander
  Wire.endTransmission();

  // Stockage de la position dans le tableau
  aig[aigIdx]=s?(aig[aigIdx]|(1<<aigBit)):(aig[(n)>>3]&~(1<<aigBit));
}

// TODO : lecture des détecteurs
// TODO : accès aux régulateurs

// --- Interruption ---

void interrupt(void)
// Balayage des matrices clavier et LED
{
  static byte count=0;

  // Lecture des touches du clavier : inversion des données lues
  key[0][count]=~JKY1PIN; 
  key[1][count]=~JKY2PIN; 

  // Inhibition des colonnes pendant le changement de valeur des LED
  digitalWrite(MXE,HIGH);
  // Changement de colonne
  count=(count+1)&15;

  JLY1PORT=led[0][count];
  JLY2PORT=led[1][count];
  JLY3PORT=led[2][count];

  // activation des sorties
  digitalWrite(MX0,count&1);
  digitalWrite(MX1,!!(count&2));
  digitalWrite(MX2,!!(count&4));
  digitalWrite(MX3,!!(count&8));
  digitalWrite(MXE,LOW);
}

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

void loop()
{
  // exemple d'utilisation
  // le bouton 1 commande la LED 5 et l'aiguillage 7 en voie directe
  // le bouton 2 commande la LED 6 et l'aiguillage 7 en voie déviée
  if(getKey(1))
  {
    setLed(5,ON);
    setLed(6,OFF);
    setAig(7,DIR);
  }
  if(getKey(2))
  {
    setLed(6,ON);
    setLed(5,OFF);
    setAig(7,DEV);

  }
  delay(10);
}

