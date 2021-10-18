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
// --- Définitions utilisées pour définir le réseau
// ----------------------------------------------------

#define DIR 0
#define DEV 1
#define ON 1
#define OFF 0
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
// --- Etat des LED du TCO
// ----------------------------------------------------

#define MAX_LED 48
byte led[MAX_LED]; // 3 groupes de lignes x 16 colonnes x 8 bits par ligne = 384 LED

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
  if(index<aigListSize) return; // numéro invalide
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
      det[index++]=Wire.read(); // Lit les données
  }
}

#define getDet(n) ((det[(n)>>3]>>((n)&7))&1)

// ----------------------------------------------------
// --- Commande des régulateurs
// ----------------------------------------------------

#define MAX_REG 32
byte reg[MAX_REG];

void setReg(word n,byte s)
{
}

byte getReg(word n)
{
  return 0;
}

// ----------------------------------------------------
// --- Pilotage des signaux
// ----------------------------------------------------

#define MAX_SIG 32
byte sig[MAX_SIG];

void setSig(word n,byte s)
{
}

#define getSig(n) ((sig[(n)>>3]>>((n)&7))&1)

// ----------------------------------------------------
// --- Interruption
// ----------------------------------------------------

void interrupt(void)
// Balayage des matrices clavier et LED
{
  static byte count=0;

  // Lecture des touches du clavier : inversion des données lues
  btn[count]=~JKY1PIN; 
  btn[count+NUM_COL]=~JKY2PIN; 

  // Inhibition des colonnes pendant le changement de valeur des LED
  digitalWrite(MXE,HIGH);
  // Changement de colonne
  count=(count+1)&15;

  JLY1PORT=led[count];
  JLY2PORT=led[count+NUM_COL];
  JLY3PORT=led[count+(NUM_COL*2)];

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
#define DET_E1 21
#define DET_S1 22
#define DET_E2 23
#define DET_S2 24
#define DET_E3 25
#define DET_S3 26
#define DET_E22 31
#define DET_S22 32

// Les signaux sont aussi déclarés
#define SIG_1 31
#define SIG_2 32
#define SIG_3 33
#define SIG_22 34

// Les régulateurs le sont aussi de la même façon
#define REG_1 41
#define REG_2 42
#define REG_3 43
#define REG_4 44

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

// La gestion du réseau est effectuée dans la fonction demo
// Il s'agit d'un gestionnaire d'événements
// Les fonctions get permettent de tester les conditions
// Les fonctions set permettent de changer d'état

void demo()
{
  // Bascule l'aiguillage vers la voie directe
  if(getBtn(BTN_AIG_2))
  {
    setLed(LED_AIG2,ON);
    setLed(LED_AIG22,OFF);
    setAig(AIG_2_22,DIR);
    setSig(SIG_22,CARRE);
    setLed(LED_SIG22,CARRE);
    setCanton(22,CARRE);
  }
  // Bascule l'aiguillage vers la voie déviée
  if(getBtn(LED_AIG22))
  {
    setLed(LED_AIG22,ON);
    setLed(LED_AIG2,OFF);
    setAig(AIG_2_22,DEV);
    setSig(SIG_2,CARRE);
    setLed(LED_SIG2,CARRE);
    setCanton(2,CARRE);
  }
  // Bouton d'arrêt forcé sur le canton 1
  if(getBtn(BTN_STOP_1))
  {
    setSig(SIG_1,CARRE);
    setLed(LED_SIG1,CARRE);
    setCanton(1,CARRE);
  }
}

// ----------------------------------------------------
// --- Fonctions de test et boucle principale
// ----------------------------------------------------

void testBtn(void)
// Affiche les boutons qui sont pressés
{
  for(word i=0; i<(MAX_BTN*NUM_COL); i++)
    if(getBtn(i))
      Serial.println(i);
}

void testDet(void)
// Affiche les détecteurs qui sont actifs
{
  for(word i=0; i<(MAX_DET*NUM_COL); i++)
    if(getBtn(i))
      Serial.println(i);
}

void loop()
{
  scanDet(); // Lecture des détecteurs de passage
  demo();
  delay(10);
}

