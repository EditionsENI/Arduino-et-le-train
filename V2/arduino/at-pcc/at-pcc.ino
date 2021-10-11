// AT-PCC

// Ce programme est encore en cours d'évolution
// La version la plus récente est disponible à l'adresse suivante :
// https://github.com/EditionsENI/Arduino-et-le-train/tree/master/V2/arduino/at-pcc

#include <Arduino.h>
#include <Wire.h>
#include <FlexiTimer2.h>
#include <EEPROM.h>

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

// Tableaux contenant l'état des LED et du clavier
byte led[3][4]; // 3 groupes de lignes x 4 colonnes x 8 bits par ligne
byte key[2][4]; // 2 groupes de lignes x 4 colonnes x 8 bits par ligne

// Macros pour un accès simplifié aux LED et au clavier
// n = numéro de LED (0-383) GGCCCCBBB (groupe/colonne/bit)
// s = état de la LED 0=éteinte / 1=allumée 
#define LED(n,s) led[0][(n)>>3]=s?(led[0][(n)>>3]|(1<<((n)&7))):(led[0][(n)>>3]&~(1<<((n)&7)))
// n = numéro du bouton
#define KEY(n) ((key[0][(n)>>3]>>((n)&7))&1)

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
  delay(100);
  // exemple d'utilisation : le bouton 1 commande la LED 1
  byte a=KEY(1);
  LED(1,a);
}
