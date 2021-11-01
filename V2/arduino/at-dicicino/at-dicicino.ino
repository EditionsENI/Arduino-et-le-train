// DiCiCino
// V2 PBA 2021-06-10
// Compilation : Arduino Mega2560 / Arduino ISP

// Ce programme est encore en cours d'évolution
// La version la plus récente est disponible à l'adresse suivante :
// https://github.com/EditionsENI/Arduino-et-le-train/tree/master/V2/arduino/at-dicicino

#include <Arduino.h>
#include <Wire.h>
#include <FlexiTimer2.h>
#include <EEPROM.h>
#include <U8g2lib.h>

U8G2_ST7920_128X64_F_8080 u8g2(U8G2_R2, 22, 23, 24, 25, 26, 27, 28, 29, 30, U8X8_PIN_NONE, 31, U8X8_PIN_NONE);

// Constantes

#define UNDEFINED 0xFFFF
#define FREE 0xFFFE
#define NO_PARAM 0x7FFF

// Types composites

typedef union l_4b
{
  long l;
  word w[2];
  byte b[4];
};

typedef union w_2b
{
  word w;
  byte b[2];
};

// Décommenter cette ligne en cas d'utilisation du contrôleur
// conçu pour la première édition du livre
// (le sens de marche n'est pas géré de la même façon)
//#define CTRL_V1

// -----------------------------------------------------
// +++ STOCKAGE EEPROM
// -----------------------------------------------------

// Décommenter la ligne suivante si un circuit de mémoire externe est installé
// La valeur correpond à la la capacité du circut en Kbits
// 24C256 => 256
//#define EEPROM_EXTERN 256

#ifdef EEPROM_EXTERN

#include <serialEEPROM.h>
serialEEPROM extEEPROM(0x50, EEPROM_EXTERN*128, 16);

#define MAX_LOCO EEPROM_EXTERN*128/sizeof(LOCO)

#endif

// EEPROM 0..31 : Paramètres généraux

#define EEPROM_ID 0 // 0..3 : Identifiant du circuit 'DCC2'
#define EEPROM_CONT 12 // Paramétrage du contraste
#define EEPROM_LUMA 13 // Paramétrage de la luminosité

// -----------------------------------------------------
// +++ Interface DCC
// -----------------------------------------------------

#define DCC_S 4 // SIGNAL (DIR)
#define DCC_E 5 // ENABLE (PWM)
#define DCC_C 6 // CUTOUT (BRAKE)

#define DCC_CUT_0 0 // Les deux états de l'automate cutout
#define DCC_CUT_1 1

#define DCC_BIT_HIGH 0 // Les quatre états de l'automate bit
#define DCC_BIT_HIGH0 1
#define DCC_BIT_LOW 2
#define DCC_BIT_LOW0 3

#define DCC_PACKET_IDLE 0 // Les cinq états de l'automate paquet
#define DCC_PACKET_HEADER 1
#define DCC_PACKET_START 2
#define DCC_PACKET_BYTE 3
#define DCC_PACKET_STOP 4
#define DCC_PACKET_END 5
#define DCC_PACKET_CUTOUT 6

#define DCC_PACKET_NUM 100 // Taille de la pile DCC
#define DCC_PACKET_SIZE 6 // Taille maximum d'un paquet DCC
#define DCC_HEADER_SIZE 20
#define DCC_CUTOUT_SIZE 18
#define DCC_FUNCTION_MAX 32 // Nombre de fonctions à commander

#define DCC_PACKET_TYPE_MODE 0xF
#define DCC_PACKET_TYPE_SPEED 0  
#define DCC_PACKET_TYPE_F0_F4 1
#define DCC_PACKET_TYPE_F5_F8 2
#define DCC_PACKET_TYPE_F9_F12 3
#define DCC_PACKET_TYPE_F13_F20 4
#define DCC_PACKET_TYPE_F21_F28 5
#define DCC_PACKET_TYPE_POM 11
#define DCC_PACKET_TYPE_POM_BIT 12
#define DCC_PACKET_TYPE_SERVICE 13
#define DCC_PACKET_TYPE_SERVICE_BIT 14
#define DCC_PACKET_TYPE_RESET 15

#define DCC_PACKET_TYPE_ADDR_LONG 0x80
#define DCC_PACKET_TYPE_STEP 0x30
#define DCC_PACKET_TYPE_STEP_14 0x00
#define DCC_PACKET_TYPE_STEP_27 0x10
#define DCC_PACKET_TYPE_STEP_28 0x20
#define DCC_PACKET_TYPE_STEP_128 0x30

byte DccCut; // 1/2 bit en cours d'envoi (pour la coupure de signal)
byte DccBit; // Bit en cours d'envoi
byte DccSubBit; // Partie du bit en cours d'envoi
byte DccDataMode;    // Variable d'état de l'automate paquet
byte DccPacketUsed = 0; // Nombre de paquets à envoyer
byte DccPacketIndex; // Paquet en cours d'envoi
byte DccHeaderCount; // Comptage des bits à un du préambule
byte DccCutoutCount; // Comptage de la séquence de coupure
byte DccByteCount;   // Index de l'octet en cours d'envoi
byte DccBitShift;    // Comptage des bits de l'octet à envoyer

typedef struct _dcc_stack_
{
word addr;
byte type;
byte size;
byte data[DCC_PACKET_SIZE];
} DCC_STACK; // 10 octets

DCC_STACK dccStack[DCC_PACKET_NUM];

//byte dccPacketData[DCC_PACKET_NUM][DCC_PACKET_SIZE]; // Paquets de données à envoyer
//byte dccPacketSize[DCC_PACKET_NUM]; // Taille des paquets à envoyer
//byte dccPacketType[DCC_PACKET_NUM]; // Type des paquets à envoyer

long dccCount=0;

void stackDump()
// Fonction de débuggage
{
  Serial.print("STACK (");
  Serial.print(dccCount);
  Serial.println(") : type [size] ...");
  for (int i = 0; i < DCC_PACKET_NUM; i++)
  {
    if(dccStack[i].size)
    {
      Serial.print(i);
      Serial.print(" ");
      switch(dccStack[i].type)
      {
        case DCC_PACKET_TYPE_SPEED :
          Serial.print("SPD   ");
          break;
        case DCC_PACKET_TYPE_F0_F4 :
          Serial.print("F0-4  ");
          break;
        case DCC_PACKET_TYPE_F5_F8 :
          Serial.print("F5-8  ");
          break;
        case DCC_PACKET_TYPE_F9_F12 :
          Serial.print("F9-12 ");
          break;
        case DCC_PACKET_TYPE_F13_F20 :
          Serial.print("F13-20");
          break;
        case DCC_PACKET_TYPE_F21_F28 :
          Serial.print("F21-28");
          break;
        default :
          Serial.print(dccStack[i].type);
          break;
      }
      Serial.print(" [");
      Serial.print(dccStack[i].size);
      Serial.print("] : ");
      for (int j = 0; j < dccStack[i].size; j++)
      {
        Serial.print(dccStack[i].data[j]);
        Serial.write(' ');
      }
      Serial.println();
    }
  }
  Serial.println("--");
}

void StackInit()
{
  for (int i = 0; i < DCC_PACKET_NUM; i++)
  {
    dccStack[i].addr=UNDEFINED;
    dccStack[i].type=0;
    dccStack[i].size=0;
  }
}

void dccInterrupt(void)
// Automate d'envoi bit à bit des données DCC
{
  dccCount++;
  switch(DccCut)
  {
    case DCC_CUT_0:
      switch (DccSubBit) // Automate bit
      {
       case DCC_BIT_HIGH :
        switch (DccDataMode) // Automate paquet
        {
          case DCC_PACKET_IDLE :
            if (DccPacketUsed)
            {
              DccDataMode = DCC_PACKET_HEADER;
              DccHeaderCount = DCC_HEADER_SIZE;
            }
            break;
          case DCC_PACKET_HEADER :
            DccBit = 1;
            if (!--DccHeaderCount)
            {
              DccDataMode = DCC_PACKET_START;
              DccByteCount = 0;
            }
            break;
          case DCC_PACKET_START :
            DccBit = 0;
            DccBitShift = 0x80;
            DccDataMode = DCC_PACKET_BYTE;
            break;
          case DCC_PACKET_BYTE :
            DccBit = !!(dccStack[DccPacketIndex].data[DccByteCount] & DccBitShift);
            DccBitShift >>= 1;
            if (!DccBitShift)
            {
              if (dccStack[DccPacketIndex].size == ++DccByteCount) // Fin du paquet
                DccDataMode = DCC_PACKET_STOP;
              else
                DccDataMode = DCC_PACKET_START;
            }
            break;
          case DCC_PACKET_STOP :
            DccBit = 1;
            // Les paquets de programmation sont effacés après envoi
            if((dccStack[DccPacketIndex].type==DCC_PACKET_TYPE_SERVICE)
              ||(dccStack[DccPacketIndex].type==DCC_PACKET_TYPE_SERVICE_BIT)
              ||(dccStack[DccPacketIndex].type==DCC_PACKET_TYPE_POM)
              ||(dccStack[DccPacketIndex].type==DCC_PACKET_TYPE_POM_BIT))
            {
              dccStack[DccPacketIndex].size = 0;
              DccPacketUsed--;  // Suppression du paquet envoyé
            }
            if (DccPacketUsed)
            {
              for (char i = DCC_PACKET_NUM; --i >= 0;)
              {
                DccPacketIndex++;
                if (DccPacketIndex == DCC_PACKET_NUM) DccPacketIndex = 0;
                if (dccStack[DccPacketIndex].size) break;
              }
              DccDataMode = DCC_PACKET_END;
              DccCutoutCount = DCC_CUTOUT_SIZE;
            }
            else
            {
              DccDataMode = DCC_PACKET_IDLE;
            }
            break;
          case DCC_PACKET_END :
              DccDataMode = DCC_PACKET_CUTOUT;
              DccCutoutCount = DCC_CUTOUT_SIZE;
            break;
        }
        digitalWrite(DCC_S, HIGH);
        if (DccBit)
          DccSubBit = DCC_BIT_LOW;
        else
          DccSubBit = DCC_BIT_HIGH0;
        break;
       case DCC_BIT_HIGH0 :
        digitalWrite(DCC_S, HIGH);
        DccSubBit = DCC_BIT_LOW;
        break;
       case DCC_BIT_LOW :
        digitalWrite(DCC_S, LOW);
        if (DccBit)
          DccSubBit = DCC_BIT_HIGH;
        else
          DccSubBit = DCC_BIT_LOW0;
        break;
       case DCC_BIT_LOW0 :
        digitalWrite(DCC_S, LOW);
        DccSubBit = DCC_BIT_HIGH;
        break;
      }
      DccCut=DCC_CUT_1;
      break;
    case DCC_CUT_1: // 1/2 bit : zone de coupure
      switch(DccDataMode)
      {
        case DCC_PACKET_CUTOUT :
          if (!--DccCutoutCount)
          {
            DccDataMode = DCC_PACKET_HEADER;
            DccHeaderCount = DCC_HEADER_SIZE;
            digitalWrite(DCC_C, LOW);
          }
          else
            digitalWrite(DCC_C, HIGH);
          break;
        default :
            DccCut=DCC_CUT_0;
      }
      break;
  }
}

void dccAdd(byte type, word addr, byte size, byte* data)
// Insère un paquet dans la liste d'envoi
{
  if (size > DCC_PACKET_SIZE) return; // Paquet trop gros
  DCC_STACK* packet=dccStack;
  DCC_STACK* freePacket=0;
  for(byte i=0; i<DCC_PACKET_NUM; i++, packet++)
  {
    if((packet->addr==addr)&&(packet->type==type)) // Paquet existant -> remplacement
    {
      freePacket=packet;
      break;
    }
    else if(packet->addr==FREE) // Emplacement libéré
    {
      freePacket=packet;
    }
    else if(packet->addr==UNDEFINED) // Emplacement libre (fin de la pile)
    {
      if(!freePacket)
        freePacket=packet;
      DccPacketUsed++;
      break;
    }
  }
  if(freePacket)
  {
    freePacket->addr=addr;
    freePacket->type=type;
    freePacket->size=size;
    memcpy(freePacket->data, data, size);
  }
}

void dccClear()
// Vide la pile d'envoi
{
  for (int i = 0; i < DCC_PACKET_NUM; i++)
  {
    dccStack[i].data[0] = 0xFF;
    dccStack[i].size = 0;
  }
  DccPacketUsed = 0;
}

void packetFormat(byte type, word addr, word data1, word data2)
// Formate un paquet DCC et le stocke dans la pile
// type : type de paquet
// addr : adresse du décodeur
// data1 + data2 : données dépendant du type de paquet
// Valeurs possibles de type :
// - DCC_PACKET_TYPE_SPEED : vitesse
// - DCC_PACKET_TYPE_F0_F4 : fonctions 0 à 4
// - DCC_PACKET_TYPE_F5_F8 : fonctions 5 à 8
// - DCC_PACKET_TYPE_F9_F12 : fonctions 9 à 12
// - DCC_PACKET_TYPE_F13_F20 : fonctions 13 à 20 (réservé)
// - DCC_PACKET_TYPE_F21_F28 : fonctions 21 à 28 (réservé)
// - DCC_PACKET_TYPE_POM : programmation CV (mode POM)
// - DCC_PACKET_TYPE_POM_BIT : programmation CV bit (mode POM)
// - DCC_PACKET_TYPE_SERVICE : programmation CV (mode service)
// - DCC_PACKET_TYPE_SERVICE_BIT : programmation CV bit (mode service)
// - DCC_PACKET_TYPE_RESET : réinitialisation
// Valeurs complémentaires de type pour l'adressage :
// - DCC_PACKET_TYPE_ADDR_LONG : adresse étendue
// Valeurs complémentaires de type pour le nombre de crans :
// - DCC_PACKET_TYPE_STEP_14 : 14 crans
// - DCC_PACKET_TYPE_STEP_27 : 27 crans
// - DCC_PACKET_TYPE_STEP_28 : 28 crans
// - DCC_PACKET_TYPE_STEP_128 : 128 crans
{
  byte packetData[DCC_PACKET_SIZE];
  byte checksum = 0;
  byte packetSize = 1;
  char* packetPtr = packetData;
  byte dir;
  byte ext;
  byte spd;

  if((type!=DCC_PACKET_TYPE_SERVICE)&&(type!=DCC_PACKET_TYPE_SERVICE_BIT)) // Une adresse doit être spécifiée
  {
    if (type & DCC_PACKET_TYPE_ADDR_LONG)
    {
      checksum ^= *packetPtr++ = 0xC0 | ((addr >> 8) & 0x3F);
      checksum ^= *packetPtr++ = addr & 0xFF;
      packetSize += 2;
    }
    else
    {
      checksum ^= *packetPtr++ = addr & 0x7F;
      packetSize++;
    }
  }
  switch (type&DCC_PACKET_TYPE_MODE)
  {
    case DCC_PACKET_TYPE_SPEED:
      dir = data2!=2;
      switch (type & DCC_PACKET_TYPE_STEP)
      {
        case DCC_PACKET_TYPE_STEP_14:
          spd=(data1>>6)&0xF;
          if(spd==1) spd=0;
          checksum ^= *packetPtr++ = (spd & 0xF) | (dir ? 0x60 : 0x40);
          packetSize++;
          break;
        case DCC_PACKET_TYPE_STEP_27:
        case DCC_PACKET_TYPE_STEP_28:
          spd=(data1>>5)&0x1F;
          if(spd<4) spd=0;
          ext = (spd & 1) << 4;
          spd >>= 1;
          checksum ^= *packetPtr++ = (spd & 0xF) | (dir ? 0x60 : 0x40) | ext;
          packetSize++;
          break;
        case DCC_PACKET_TYPE_STEP_128:
          spd=(data1>>3)&0x7F;
          if(spd==1) spd=0;
          checksum ^= *packetPtr++ = 0x3F;
          checksum ^= *packetPtr++ = (spd & 0x7F) | (dir ? 0x80 : 0);
          packetSize += 2;
          break;
      }
      break;
    case DCC_PACKET_TYPE_F0_F4:
      checksum ^= *packetPtr++ = 0x80 | (((byte)data1) & 0xF) | ((((byte)(data2))&1)<<4);
      packetSize++;
      break;
    case DCC_PACKET_TYPE_F5_F8:
      checksum ^= *packetPtr++ = 0xB0 | (data1 & 0xF);
      packetSize++;
      break;
    case DCC_PACKET_TYPE_F9_F12:
      checksum ^= *packetPtr++ = 0xA0 | (data1 & 0xF);
      packetSize++;
      break;
    case DCC_PACKET_TYPE_F13_F20:
      return; // TODO
    case DCC_PACKET_TYPE_F21_F28:
      return; // TODO
    case DCC_PACKET_TYPE_POM :   // data1 = Numéro de la variable ; data2 = Valeur
      checksum ^= *packetPtr++ = ((data1 >> 8) & 3) | 0x7C;
      checksum ^= *packetPtr++ = data1 & 0xFF;
      checksum ^= *packetPtr++ = data2 & 0xFF;
      packetSize += 3;
      break;
    case DCC_PACKET_TYPE_POM_BIT :   // data1 = Numéro de la variable ; data2 = masque + bit
      checksum ^= *packetPtr++ = ((data1 >> 8) & 3) | 0x78;
      checksum ^= *packetPtr++ = data1 & 0xFF;
      checksum ^= *packetPtr++ = 0xF0 | data2;
      packetSize += 3;
      break;
    case DCC_PACKET_TYPE_RESET :
      checksum ^= *packetPtr++ = 0;
      checksum ^= *packetPtr++ = 0;
      packetSize += 2;
      break;
  }
  *packetPtr = checksum;
  dccAdd(type & DCC_PACKET_TYPE_MODE, addr, packetSize, packetData);
}

void DccReset()
// Envoie une séquence de réinitialisation générale des décodeurs
{
  byte i;
  for (i = 0; i < 5; i++)
    packetFormat(DCC_PACKET_TYPE_RESET, 0xFF, 0, 0);
}

// -----------------------------------------------------
// +++ PROGRAMMATION EN VOIE
// -----------------------------------------------------

#define POM_ACCEL 1
#define POM_DECEL 2
#define POM_WRITE 3
#define POM_TEST 4
#define POM_BIT_WRITE 5
#define POM_BIT_TEST 6

void pomProgram(word addr, byte type, word cv, byte data)
{
byte out[3];
byte outSize=0;

  switch(type)
  {
  case POM_ACCEL :
    out[0]=0xf2;
    out[1]=data;
    outSize=2;
    break;
  case POM_DECEL :
    out[0]=0xf3;
    out[1]=data;
    outSize=2;
    break;
  case POM_WRITE :
    out[0]=0xec|((cv>>8)&3);
    out[1]=cv&0xff;
    out[2]=data;
    outSize=3;
    break;
  case POM_TEST :
    out[0]=0xe4|((cv>>8)&3);
    out[1]=cv&0xff;
    out[2]=data;
    outSize=3;
    break;
  case POM_BIT_WRITE :
    out[1]=cv&0xff;
    out[2]=0xf0|(data&0xf);
    outSize=3;
    break;
  case POM_BIT_TEST :
    out[1]=cv&0xff;
    out[2]=0xe0|(data&0xf);
    outSize=3;
    break;
  }
  if(outSize>0)
    for(byte i=0; i<2; i++)
      dccAdd(DCC_PACKET_TYPE_POM,addr,out,outSize);
}

// -----------------------------------------------------
// +++ Liste des locomotives
// -----------------------------------------------------

typedef struct _loco_
{
  word  id; // adresse du décodeur 0..9999 (étendue) / 10000..10099 (courte)
  word  vmax; // Consigne de vitesse maximum (bits 0 à 8) ( + le nombre de crans sur les bits 14 et 15)
  char  name[16]; // Nom de la locomotive
} LOCO; // 20 octets

#define EEPROM_LOCO 168 // Stockage en EEPROM du paramétrage des décodeurs
#define LOCO_SIZE 196 // Stockage EEPROM jusqu'à l'adresse 168+(196*20)-1 = 4087 (limite=4095)

LOCO locoList[LOCO_SIZE];
byte locoIndex[LOCO_SIZE]; // Tri des locos en RAM

byte numLoco=0; // Nombre de locomotives disponibles

void locoListInit()
// Lecture de la liste des locos depuis l'EEPROM
{
  for(byte i=0; i<LOCO_SIZE; i++)
  {
    byte* ptr=(byte*)&locoList[i];
    word index=i*sizeof(LOCO)+EEPROM_LOCO;
    *ptr++=EEPROM.read(index++);
    *ptr++=EEPROM.read(index++);
    if(locoList[i].id!=UNDEFINED)
      for(byte j=0; j<sizeof(LOCO)-2; j++)
        *ptr++=EEPROM.read(index++);
  }
}

void locoListSort()
// Tri alphabétique de la liste des locos
{
  byte lastIndex=0xFF;
  byte curIndex=0xFF;
  numLoco=0;

  for(byte j=0; j<LOCO_SIZE; j++) // Recherche la première par ordre alphabétique
  {
    if((*locoList[j].name)&&((curIndex==0xFF)||(strncmp(locoList[j].name,locoList[curIndex].name,sizeof(locoList->name))<0)))
      curIndex=j; // TODO : problème si plus d'une loco a le même nom
  }
  locoIndex[0]=lastIndex=curIndex;
  for(byte i=1; i<LOCO_SIZE; i++)
  {
    curIndex=0xFF;
    for(byte j=0; j<LOCO_SIZE; j++) // Recherche les suivantes
    {
      if((*locoList[j].name)&&(strncmp(locoList[j].name,locoList[lastIndex].name,sizeof(locoList->name))>0)&&((curIndex==0xFF)
         ||(strncmp(locoList[j].name,locoList[curIndex].name,sizeof(locoList->name))<0)))
      curIndex=j;
    }
    if(curIndex==0xFF)
    {
      numLoco=i;
      break;
    }
    locoIndex[i]=lastIndex=curIndex;
  }
}

// -----------------------------------------------------
// +++ GESTION DU CLAVIER
// -----------------------------------------------------

#define SW_H 42
#define SW_B 43
#define SW_G 44
#define SW_D 45
#define SW_P1 46
#define SW_P2 47
#define SW_P3 48
#define SW_P4 49
#define SW_ESC 50
#define SW_DEL 51
#define SW_NO 52
#define SW_OK 53
#define KB_R0 32
#define KB_R1 33
#define KB_R2 34
#define KB_R3 35
#define KB_C0 36
#define KB_C1 37
#define KB_C2 38
#define KB_C3 39

const PROGMEM byte KbdOut[4][4]=
{
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

void keyInit()
{
  for(byte pin=SW_H; pin<=SW_OK; pin++)
    pinMode(pin,INPUT_PULLUP);
  for(byte row=KB_R0; row<=KB_R3; row++)
    pinMode(row,OUTPUT);
  for(byte col=KB_C0; col<=KB_C3; col++)
    pinMode(col,INPUT_PULLUP);
}

byte keyRead()
{
  static char lastKey=0;
  char newKey=0;

  for(byte pin=SW_H; pin<=SW_OK; pin++)
    if(digitalRead(pin)==LOW) newKey=pin|0x80;

  for(byte row=KB_R0; row<=KB_R3; row++)
    digitalWrite(row,HIGH);
  for(byte row=KB_R0; row<=KB_R3; row++)
  {
    digitalWrite(row,LOW);
    for(byte col=KB_C0; col<=KB_C3; col++)
    {
      if(digitalRead(col)==LOW)
      {
        newKey=pgm_read_byte_near(&KbdOut+((row-KB_R0)<<2)+(col-KB_C0));
      }
    }
    digitalWrite(row,HIGH);
  }

  if(newKey!=lastKey)
  {
    delay(50);
    lastKey=newKey;
    return newKey;
  }
  return 0;
}

// -----------------------------------------------------
// +++ Interface utilisateur
// -----------------------------------------------------

/*

POLICES DE CARACTÈRES UTILISÉES PAR LE PROGRAMME

TEXTE GRAND
u8g2.setFont(u8g2_font_logisoso28_tn);

TEXTE NORMAL
u8g2.setFont(u8g2_font_finderskeepers_tf);

TEXTE RÉDUIT
u8g2.setFont(u8g2_font_profont10_tf);

TEXTE PETIT
u8g2.setFont(u8g2_font_blipfest_07_tr);

ICÔNES
u8g2.setFont(u8g2_font_open_iconic_all_1x_t);
u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);

*/

// Macro pour afficher un texte stocké en mémoire programme
#define printAt(X,Y,T) u8g2.setCursor(X,Y);u8g2.print(F(T))

// Liste des pages de l'interface utilisateur
enum
{
  PAGE_LOGO, PAGE_PILOT, PAGE_CONTROL, PAGE_LISTE, PAGE_PARAM,
  PAGE_EDIT_LOCO, PAGE_DEL_LOCO
};

#define ID_TYPE 1
#define ID_TRAIT 2
#define ID_COMPACT 4

char* formatId(word id,char* text, int mode=0)
// Formate l'adresse du décodeur
// id : adresse à formater
// text : chaîne de 5 à 9 caractères qui va recevoir le résultat
// mode : ID_TYPE = affiche EXT ou STD avant l'adresse
//        ID_TRAIT = affiche des traits si l'adresse n'est pas valide
//        ID_COMPACT = affiche les adresses courtes sur 2 caractères
{
  if(id<10000)
  {
    if(mode&ID_TYPE)
      sprintf(text,"EXT %04d",id);
    else
      sprintf(text,"%04d",id);
  }
  else if(id<10100)
  {
    if(mode&ID_TYPE)
      sprintf(text,"STD %02d",id-10000);
    else if(mode&ID_COMPACT)
      sprintf(text,"%02d",id-10000);
    else
      sprintf(text,"  %02d",id-10000);
  }
  else if(mode&ID_TRAIT)
    strcpy(text,"----");
  else
    *text=0;
  return text;
}

byte formatCrans(word spd)
{
  switch(spd>>14)
  {
    case 1 : return 27;
    case 2 : return 28;
    case 3 : return 128;
  }
  return 14;
}

#define EDIT_04 4 // Les chiffres de 0 à 4 (centaines de km/h)
#define EDIT_09 9 // Les chiffres de 0 à 9
#define EDIT_AZ09 1 // Les caractères, les chiffres et l'espace

byte editDigit(byte digit,char key,byte mode)
// Change la valeur d'un caractère selon l'appui sur les touches haut et bas
{
  switch(key)
  {
    case SW_H :
      switch(mode)
      {
        case EDIT_04:
          if(digit=='4') return '0';
        case EDIT_09:
          if(digit=='9') return '0';
          break;
        case EDIT_AZ09:
          if(digit==' ') return 'A';
          if(digit=='Z') return '0';
          if(digit=='9') return ' ';
          break;
      }
      return digit+1;
    case SW_B :
      switch(mode)
      {
        case EDIT_04:
          if(digit=='0') return '4';
          break;
        case EDIT_09:
          if(digit=='0') return '9';
          break;
        case EDIT_AZ09:
          if(digit=='A') return ' ';
          if(digit=='0') return 'Z';
          if(digit==' ') return '9';
          break;
      }
      return digit-1;
  }
  return 0;
}

// -----------------------------------------------------
// +++ AFFICHAGE DES MENUS
// -----------------------------------------------------

char btnPilot[]="PILOT";
char btnContol[]="CONTROL";
char btnListe[]="LISTE";
char btnParam[]="PARAM";
char* btn1[4]={btnPilot,btnContol,btnListe,btnParam};

void menu(char* btn[4])
// Affiche la ligne de menu en bas de l'écran
{
  u8g2.setFont(u8g2_font_blipfest_07_tr);

  byte espace=128;
  byte bw[4];
  for(byte i=0; i<4; i++)
    espace-=bw[i]=u8g2.getStrWidth(btn[i]);
  espace/=8;
  byte x=espace;
  for(byte i=0; i<4; i++)
  {
    u8g2.setDrawColor(1);
    u8g2.drawBox(x-1,57,bw[i]+2,7);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x,63,btn[i]);
    x+=espace*2+bw[i];
  }
}

// -----------------------------------------------------
// +++ Contrôleurs
// -----------------------------------------------------

// EEPROM 32..159 : Configuration des contrôleurs

typedef struct _control_
{
  word  num; // Numéro de la locomotive dans locoList
  word  spd; // consigne de vitesse (seulement en RAM)
  union l_4b  F; // Etat des fonctions (seulement en RAM)
  word  numFct; // Nombre d'octets à lire pour les fonctions
} CONTROL; // 8 octets en RAM ou 2 octets en EEPROM

#define EEPROM_CONTROL 32 // Stockage en EEPROM du paramétrage des contrôleurs
#define CONTROL_PANEL 16 // Nombre de contrôleurs sur le tableau
#define CONTROL_SIZE 32 // Nombre total de contrôleurs : 16 sur le tableau + 16 séparés (I²C 0x10 à 0x1F)
#define CONTROL_LIMIT 64 // Espace réservé pour augmenter le nombre de contrôleurs

CONTROL controlList[CONTROL_SIZE];

byte cmdName[4]={0,0xFF,0,0xFF};
byte controlName[4]={'C','T','R','L'};
#ifdef CTRL_V1
byte oldDir[4]={2,3,3,1};
#endif

#define CMD_SETVMAX 0x10
#define CMD_SETADDR 0x11
#define CMD_SETADDREXT 0x12
#define CMD_SETNAME 0x13
#define CMD_SETLED 0x14
#define CMD_GETSPEED 0x21
#define CMD_GETNUMFCT 0x22
#define CMD_GETFCT 0x23
#define CMD_GETCRANS 0x24

#define CTRL_EN 7
#define CTRL_A0 8
#define CTRL_A1 9
#define CTRL_A2 10
#define CTRL_A3 11
#define CTRL_V 0
#define CTRL_S 1
#define CTRL_F1 2
#define CTRL_F2 3

void controlDump()
// Fonction de débuggage
{
  Serial.println("CONTROL : num dir spd f1/32");
  for (int i = 0; i < CONTROL_SIZE; i++)
  {
    if(controlList[i].num==UNDEFINED) continue;
    Serial.print(controlList[i].num);
    Serial.print(" ");
    Serial.print(controlList[i].spd>>14);
    Serial.print(" ");
    Serial.print(controlList[i].spd&0x3FF);
    Serial.print(" ");
    Serial.println(controlList[i].F.l,BIN);
  }
  Serial.println("--");
}

void controlInit(void)
// -- Initialise le panneau de contrôle --
{
  pinMode(CTRL_EN,OUTPUT);
  pinMode(CTRL_A0,OUTPUT);
  pinMode(CTRL_A1,OUTPUT);
  pinMode(CTRL_A2,OUTPUT);
  pinMode(CTRL_A3,OUTPUT);
  digitalWrite(CTRL_EN,LOW);
  digitalWrite(CTRL_A0,LOW);
  digitalWrite(CTRL_A1,LOW);
  digitalWrite(CTRL_A2,LOW);
  digitalWrite(CTRL_A3,LOW);

  for(byte i=0; i<CONTROL_SIZE; i++)  // Tous les contrôleurs sont non affectés par défaut
  {
    controlList[i].num=UNDEFINED;
    controlList[i].spd=0;
    controlList[i].F.l=0;
  }
}

byte controlScan(void)
// -- Parcourt les contrôleurs pour lire les consignes appliquées aux locomotives --
// Met à jour les paquets DCC si les valeurs ont changé
// Retourne true si l'affichage doit être mis à jour
// TODO : mise à jour de l'écran de pilotage
{
  byte  changed=false;
  CONTROL* control=controlList;

  // Panneau de contrôle
  for(byte ctrl=0; ctrl<CONTROL_SIZE; ctrl++, control++)
  {
    word v;
    byte dir;
    word loco;
    word spd;
    byte ext;
    byte crans;
    union l_4b F;
    // Lecture du contrôleur
    if(control->num!=UNDEFINED) // Contrôleur affecté
    {
      loco=locoList[control->num].id;
      if(loco<10000)
      {
        ext=DCC_PACKET_TYPE_ADDR_LONG;
      }
      else if(loco<10100)
      {
        ext=0;
        loco-=10000;
      }
      else
        continue;
      crans=((byte)(locoList[control->num].vmax>>10))&0x30;

      if(ctrl<CONTROL_PANEL) // Contrôleur intégré au panneau de contrôle
      {
        // Sélection du contrôleur
        digitalWrite(CTRL_A0,ctrl&1);
        digitalWrite(CTRL_A1,!!(ctrl&2));
        digitalWrite(CTRL_A2,!!(ctrl&4));
        digitalWrite(CTRL_A3,!!(ctrl&8));
        digitalWrite(CTRL_EN,HIGH);
        // Lecture des valeurs de vitesse, direction et fonctions
        v=analogRead(CTRL_V);

        // NOTE : Si la vitesse maximum ne peut pas être atteinte, c'est que le 5 V
        // fourni par le circuit d'alimentation est inférieur au 5 V de l'Arduino
        // -> il faut augmenter légèrement la tension 5 V fournie par le régulateur
        // ATTENTION : Ne pas dépasser 5,2 V

#ifdef CTRL_V1 // Contrôleur première édition
        dir=oldDir[analogRead(CTRL_S)>>8];
        byte f2=0;
#else // Contrôleur deuxième édition
        dir=analogRead(CTRL_S)>>7; // AV=1; AR=2; STOP=3
        byte f2=(992-analogRead(CTRL_F2))>>6;
#endif
        byte f1=(992-analogRead(CTRL_F1))>>6;
        digitalWrite(CTRL_EN,LOW);
        // Mise en forme
        spd=(v&0x3FF)|((word)dir<<14);
        F.b[0]=(f2<<6)|f1;
        F.b[1]=(f2>>2);
        F.b[2]=0;
        F.b[3]=0;
      }
      else // Contrôleur séparé connecté sur le bus I²C
      {
        Wire.beginTransmission(ctrl);
        Wire.write(CMD_GETCRANS);
        if(Wire.endTransmission())
          continue; // Pas de réponse
        // Lecture de la réponse
        byte data[8];
        byte numBytes=Wire.requestFrom(ctrl,(byte)2); // Attend les données
        if(numBytes!=2) return 0xFF; // Mauvaise réponse
        for(int i=0; (i<numBytes)&&Wire.available(); i++) // Lecture de la vitesse
          data[i]=Wire.read(); // Lit les données
        spd=(data[0]<<8)|data[1];
  
        if(control->numFct>0)
        {
          Wire.beginTransmission(ctrl);
          Wire.write(CMD_GETFCT);
          if(Wire.endTransmission())
            continue; // Pas de réponse
          // Lecture de la réponse
          numBytes=control->numFct;
          if(numBytes>sizeof(data)) numBytes=sizeof(data);
          numBytes=Wire.requestFrom(ctrl,(byte)numBytes);
          F.l=0;
          byte i=0;
          for(i=0; (i<numBytes)&&Wire.available(); i++)  // Lecture des fonctions
            F.b[i]=Wire.read(); // Lit les données
          for(;i<sizeof(F);i++)
            F.b[i]=0;
        }
        else
          F.l=0;
      }

      // Mise à jour des paramètres de pilotage
      if(control->spd!=spd)
      {
        word sens=spd>>14;
        packetFormat(DCC_PACKET_TYPE_SPEED|ext|crans,loco,(sens!=3)?spd&0x3FF:0,sens); // Vitesse et sens de déplacement
        changed=true;
      }
      if(((control->F.b[0]&0xF)!=(F.b[0]&0xF))||((control->spd&0xC000)!=(spd&0xC000))) // F0 - F4
      {
        packetFormat(DCC_PACKET_TYPE_F0_F4|ext,loco,F.b[0]&0xF,(spd&0xC000)!=0xC000);
        changed=true;
      }
      if((control->F.b[0]&0xF0)!=(F.b[0]&0xF0)) // F5 - F8
      {
        packetFormat(DCC_PACKET_TYPE_F5_F8|ext,loco,F.b[0]>>4,0);
        changed=true;
      }
      if((control->F.b[1]&0xF)!=(F.b[1]&0xF)) // F9 - F12
      {
        packetFormat(DCC_PACKET_TYPE_F9_F12|ext,loco,F.b[1]&0xF,0);
        changed=true;
      }
      if(((control->F.b[1]&0xF0)!=(F.b[1]&0xF0))||((control->F.b[2]&0xF)!=(F.b[2]&0xF))) // F13 - F20
      {
        packetFormat(DCC_PACKET_TYPE_F13_F20|ext,loco,((F.b[1]>>4)||(F.b[2]<<4))&0xFF,0);
        changed=true;
      }
      if(((control->F.b[2]&0xF0)!=(F.b[2]&0xF0))||((control->F.b[3]&0xF)!=(F.b[3]&0xF))) // F21 - F28
      {
        packetFormat(DCC_PACKET_TYPE_F21_F28,loco|ext,((F.b[2]>>4)||(F.b[3]<<4))&0xFF,0);
        changed=true;
      }
      control->spd=spd;
      control->F.l=F.l;
    }
  }
  return(changed);
}

byte controlAssign(byte ctrl, word loco)
// Assignation d'un contrôleur
// ctrl : numéro du contrôleur à affecter
// loco : numéro de la locomotive
{
  if(ctrl>=CONTROL_PANEL) // Contrôleur I²C
  {
    // Vérifie le un contrôleur est présent
    Wire.beginTransmission(ctrl);
    Wire.write(cmdName,sizeof(cmdName));
    if(Wire.endTransmission())
      return 0xFF; // Pas de circuit I²C
    delay(100);
    // Lecture de la réponse
    byte data[4];
    byte numBytes=Wire.requestFrom(ctrl,(byte)sizeof(data)); // Attend les données
    if(numBytes!=4) return 0xFF; // Mauvaise réponse
    for(int i=0; (i<numBytes)&&Wire.available(); i++) // Lecture de l'identifiant
      data[i]=Wire.read(); // Lit les données
    if(memcmp(data,controlName,sizeof(data))) return 0xFF; // Mauvais type de circuit

    controlList[ctrl].num=loco;
    controlList[ctrl].spd=0;
    controlList[ctrl].F.l=0;
    delay(100);

    if(locoList[loco].id<10000)
    {
      Wire.beginTransmission(ctrl);
      Wire.write(CMD_SETADDREXT);
      Wire.write((byte)(locoList[loco].id>>8));
      Wire.write((byte)(locoList[loco].id&0xFF));
      Wire.endTransmission();
    }
    else if(locoList[loco].id<10100)
    {
      Wire.beginTransmission(ctrl);
      Wire.write(CMD_SETADDR);
      Wire.write((byte)(locoList[loco].id-10100));
      Wire.endTransmission();
    }
    delay(100);

    Wire.beginTransmission(ctrl);
    Wire.write(CMD_SETVMAX);
    Wire.write((byte)((locoList[loco].vmax>>8)&1));
    Wire.write((byte)(locoList[loco].vmax&0xFF));
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(ctrl);
    Wire.write(CMD_SETNAME);
    Wire.write(locoList[loco].name,sizeof(locoList->name));
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(ctrl);
    Wire.write(CMD_SETLED);
    Wire.write(5);
    Wire.write(0);
    Wire.write(0);
    Wire.endTransmission();

    Wire.beginTransmission(ctrl);
    Wire.write(CMD_GETNUMFCT);
    Wire.endTransmission();
    numBytes=Wire.requestFrom(ctrl,(byte)1);
    if((numBytes==1)&&Wire.available())
      controlList[ctrl].numFct=Wire.read();
    else
      controlList[ctrl].numFct=0;
  }
  else // Panneau de contrôle
  {
    controlList[ctrl].num=loco;
  }
  return 0;
}

byte pageControl(byte key, word loco=NO_PARAM)
{
// + Paramètre le nombre de contrôleurs installés
  static byte mode=0; // 0=affichage ; 1=édition; 2=assignation
  static byte x=0;
  static byte y=0;
  static word locoNum=0;
  char text[32];
  if(loco!=NO_PARAM)
  {
    locoNum=loco;
    mode=2;
  }

  switch(key)
  {
    case SW_H :
      if((mode)&&(y>0)) y--; break;
    case SW_B :
      if((mode)&&(y<7)) y++; break;
    case SW_G :
      if((mode)&&(x>0)) x--; break;
    case SW_D :
      if((mode)&&(x<3)) x++; break;
    case SW_OK :
      switch(mode)
      {
        case 0 : mode=1; break;
        case 1 : mode=0; break;
        case 2 : if(controlAssign(x*8+y,locoNum)!=0xFF) mode=0; // Assignation
          break;
      }
      break;
    case SW_DEL :
      // TODO : Suppression d'une affectation
      break;
  }

  u8g2.setFont(u8g2_font_profont10_tf);
  u8g2.setDrawColor(1);
  switch(mode)
  {
    case 0 :
      printAt(2,7,"CONTROLEURS  (OK=Edition)");
      break;
    case 1 : 
      if(x<=1)
        sprintf(text,"PANNEAU %d : ",x*8+y);
      else
        sprintf(text,"I2C %d : ",x*8+y);
      if(controlList[x*8+y].num!=UNDEFINED)
        strcat(text,locoList[controlList[x*8+y].num].name);
      u8g2.setCursor(2,7);
      u8g2.print(text);
      break;
    case 2 :
      u8g2.setCursor(2,7);
      sprintf(text,"ASSIGNATION : %s",locoList[locoNum].name);
      u8g2.print(text);
  }

  byte maxY=((mode==1)&&(x==0)&&(y==0))?4:8;

  u8g2.setFont(u8g2_font_blipfest_07_tr);
  for(byte xx=0; xx<4; xx++)
    for(byte yy=0; yy<maxY; yy++)
    {
      u8g2.setCursor(xx*32+3,yy*6+14);
      u8g2.print(xx*8+yy);
      u8g2.setCursor(xx*32+15,yy*6+14);
      char text[8];
      if(controlList[xx*8+yy].num!=UNDEFINED) // Contrôleur affecté
        u8g2.print(formatId(locoList[controlList[xx*8+yy].num].id,text));
    }

  maxY=maxY*6+7;
  for(byte y=9; y<=maxY; y+=2)
  {
    u8g2.drawPixel(32,y);
    u8g2.drawPixel(64,y);
    u8g2.drawPixel(96,y);
  }

  u8g2.setFont(u8g2_font_profont10_tf);
  if((mode==1)&&(x==0)&&(y==0)) // Affichage d'une boîte d'aide
  {
      printAt(2,44,"OK      : Edition");
      printAt(2,52,"        : D\xe9placement");
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(2,53,0x44); // Flèche vers le haut
      u8g2.drawGlyph(12,53,0x47); // Flèche vers le bas
      u8g2.drawGlyph(22,53,0x45); // Flèche vers la gauche
      u8g2.drawGlyph(32,53,0x46); // Flèche vers la droite
      u8g2.drawFrame(0,35,128,21); // Boîte d'information
  }

  if(mode)
  {
    u8g2.setDrawColor(2);
    u8g2.drawBox(x*32+2,y*6+8,29,7); // Élément actif
  }

  u8g2.setDrawColor(2);
  u8g2.drawBox(0,0,128,8); // Barre de titre
  return 0;
}

// -----------------------------------------------------
// +++ Pilotage
// -----------------------------------------------------

byte pagePilot(byte key)
// Affichage du contrôleur de pilotage
// On change de contrôleur avec les touches gauche et droite
// - Consigne de vitesse
// - Curseur de puissance
// - Sens de marche
// - Nom de la locomotive
// - Numéro du contrôleur
// TODO :
// Plusieurs tailles d'affichage en fonction des touches haut et bas :
// 1 / 2 / 4 / 8 / 16 contrôleurs à la fois
{
  static byte n=0;
  switch(key)
  {
    case SW_H :
      break;
    case SW_B :
      break;
    case SW_G :
      if(n>0) n--;
      else n=15;
      break;
    case SW_D :
      if(n<15) n++;
      else n=0;
      break;
  }
  
  word locoNum=controlList[n].num;
  if(locoNum!=UNDEFINED)
  {
    // Cadres vitesse et sens
    u8g2.setDrawColor(1);
    u8g2.drawFrame(17,12,58,36);
    u8g2.drawFrame(77,12,34,36);
    // Sens de marche
    switch(controlList[n].spd&0xC000)
    {
      case 0x4000 :u8g2.drawTriangle(85,20,102,30,85,40); break;
      case 0x8000 :u8g2.drawTriangle(102,20,85,30,102,40); break;
      case 0xC000 :u8g2.drawBox(85,20,102-85,40-20); break;
    }
    // Compteur de vitesse
    static char v[10];
    word spd=controlList[n].spd&0x3FF;
    sprintf(v,"%03d",((long)spd*((locoList[locoNum].vmax&0x1FF)+1))>>10);
    u8g2.setFont(u8g2_font_logisoso28_tn);
    u8g2.setCursor(19,44);
    u8g2.print(v);
    // Nom de la locomotive
    char text[17];
    strncpy(text,locoList[locoNum].name,sizeof(text));
    u8g2.setFont(u8g2_font_finderskeepers_tf);
    u8g2.setCursor(20,8);
    u8g2.print(text);
    // Curseur de vitesse
    for(byte x=4; x<=124; x+=12)
      u8g2.drawLine(x,53,x,54);
    word x=(spd*61)>>9;
    if(x>120) x=120;
    u8g2.drawTriangle(2+x,49,7+x,49,4+x,52);
  }

  // Numéro du contrôleur
  char num[3];
  sprintf(num,"%d",n);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_finderskeepers_tf);
  u8g2.setCursor(4,8);
  u8g2.print(num);

  // Barre de titre
  u8g2.setDrawColor(2);
  u8g2.drawBox(0,0,16,10);
  u8g2.drawBox(17,0,111,10);

  return 0;
}

// -----------------------------------------------------
// +++ Liste des locomotives
// -----------------------------------------------------

#define NEW_LOCO 1000

//byte pageAddpilot(byte key,byte n=0)
//{
//  u8g2.setFont(u8g2_font_profont10_tf);
//  u8g2.setDrawColor(1);
//  printAt(2,7,"PILOTAGE");
//  return 0;
//}

void locoDump()
// Fonction de débuggage
{
  Serial.println("LOCO : idx addr crans vmax name");
  for (int i = 0; i < LOCO_SIZE; i++)
  {
    if(locoList[i].id==UNDEFINED)
      continue;
    Serial.print(i);
    Serial.print(" ");
    if(locoList[i].id<10000)
      Serial.print(locoList[i].id);
    else if(locoList[i].id<10100)
      Serial.print(locoList[i].id-10000);
    Serial.print(" ");
    Serial.print(locoList[i].vmax>>14);
    Serial.print(" ");
    Serial.print(locoList[i].vmax&0x3FF);
    Serial.print(" ");
    char text[17];
    memcpy(text,locoList[i].name,sizeof(text)-1);
    text[sizeof(text)-1]=0;
    Serial.println(text);
  }
  Serial.println("--");
}

void saveLoco(word locoNum,char* locoName,word id,word Vmax)
{
  if(locoNum==NEW_LOCO) // Nouvelle locomotive
  {
    for(locoNum=0; locoNum<LOCO_SIZE; locoNum++)
    {
      if(locoList[locoNum].id==UNDEFINED) break; // Place libre
    }
    if(locoNum==LOCO_SIZE) return; // Pas de place libre
  }
  locoList[locoNum].id=id;
  locoList[locoNum].vmax=Vmax;
  strncpy(locoList[locoNum].name,locoName,sizeof(locoList->name));
  // Enregistrement dans l'EEPROM
  byte* ptr=(byte*)&locoList[locoNum];
  word index=locoNum*sizeof(LOCO)+EEPROM_LOCO;
  for(byte i=0; i<sizeof(LOCO); i++)
    EEPROM.update(index++,*ptr++);

}

byte pageDelLoco(byte key, word loco=NO_PARAM)
{
  static word locoNum;
  if(loco!=NO_PARAM) locoNum=loco;
  switch(key)
  {
    case SW_OK :
      // Effacement en mémoire
      locoList[locoNum].id=UNDEFINED;
      locoList[locoNum].vmax=0;
      memset(locoList[locoNum].name,0,sizeof(locoList->name));
      // Effacement en EEPROM
      {
        byte* ptr=(byte*)&locoList[locoNum];
        word index=locoNum*sizeof(LOCO)+EEPROM_LOCO;
        for(byte i=0; i<sizeof(LOCO); i++)
          EEPROM.update(index++,*ptr++);
      }
      // Mise à jour de la liste
      locoListSort();
      return PAGE_LISTE;
    case SW_NO :
    case SW_ESC :
      return PAGE_EDIT_LOCO;
  }

  u8g2.setFont(u8g2_font_profont10_tf);
  u8g2.setDrawColor(1);
  printAt(2,7,"SUPPRESSION");

  printAt(2,20,"Confirmer la suppression");
  char text[32];
  sprintf(text,"de %s",locoList[locoNum].name);
  u8g2.drawStr(2,30,text);

  printAt(2,45,"NON Annuler  OK Confirmer");
  u8g2.drawRFrame(0,37,18,10,1); // Bouton
  u8g2.drawRFrame(63,37,17,10,1); // Bouton

  u8g2.setDrawColor(2);
  u8g2.drawBox(0,0,128,8); // Barre de titre
  return 0; 
}

byte pageEditLoco(byte key, char num, word n=NO_PARAM)
// Edition des paramètres d'une locomotive
// key : touche du clavier principal
// num : touche du clavier numérique (option)
// n : numéro de la locomotive dans le tableau
{
  static word locoNum;
  static char locoName[17];
  static bool locoExt=true;
  static char locoId[5];
  static char locoVmax[4];
  static word locoCrans;
  static byte s=0;
  static byte p=0;
  switch(key)
  {
    case SW_H :
    case SW_B :
      switch(s)
      {
        case 0: // Nom
          locoName[p]=editDigit(locoName[p],key,EDIT_AZ09);
          break;
        case 1: // Adr
          if(p==0)
            locoExt=!locoExt;
          else if(locoExt)
            locoId[p-1]=editDigit(locoId[p-1],key,EDIT_09);
          else
            locoId[p+1]=editDigit(locoId[p+1],key,EDIT_09);
          break;
        case 2: // Vmax
          locoVmax[p]=editDigit(locoVmax[p],key,(p>0)?EDIT_09:EDIT_04);
          break;
        case 3: // Crans
          if(key==SW_H) locoCrans+=0x4000;
          else locoCrans-=0x4000;
          break;
      }
      break;
    case SW_G :
      if(p>0) p--;
      break;
    case SW_D :
      switch(s)
      {
        case 0: // Nom
          if(p<15) p++;
          break;
        case 1: // Adr
          if(locoExt)
            {if(p<4) p++;}
          else
            {if(p<2) p++;}
          break;
        case 2: // Vmax
          if(p<2) p++;
          break;
      }
      break;
    case SW_NO :
      if(s<3) s++;
      else s=0;
      p=0;
      break;
    case SW_OK :
      {
        word id;
        word Vmax;
        if(locoExt)
          id=atoi(locoId);
        else
          id=atoi(locoId+2)+10000;
        Vmax=atoi(locoVmax)|locoCrans;
        for(byte i=sizeof(locoName)-2; i>0; i--) // supprime les espaces
        {
          if(locoName[i]==' ') locoName[i]=0;
          else break;
        }
        saveLoco(locoNum,locoName,id,Vmax);
        locoListSort();
        return PAGE_LISTE;
      }
      break;
    case SW_ESC :
      return PAGE_LISTE;
    case SW_DEL :
      if(locoNum!=NEW_LOCO)
      {
        pageDelLoco(0,locoNum);
        return PAGE_DEL_LOCO;
      }
      break;
  }

  u8g2.setFont(u8g2_font_profont10_tf);
  u8g2.setDrawColor(1);
  printAt(2,15,"Nom :");
  printAt(2,23,"Adr :");
  printAt(2,31,"Vmax:");
  printAt(78,31,"Crans:");

  // Initialisation des données à éditer
  if(n!=NO_PARAM)
  {
    locoNum=n;
    s=0;
    p=0;
    if(locoNum!=NEW_LOCO)
    {
      memcpy(locoName,locoList[locoNum].name,sizeof(locoName)-1);
      if((sizeof(locoName)-1)>strlen(locoName))
        memset(locoName+strlen(locoName),' ',sizeof(locoName)-strlen(locoName)-1);
      locoName[16]=0;
      if(locoList[locoNum].id<10000)
      {
        sprintf(locoId,"%04d",locoList[locoNum].id);
        locoExt=true;
      }
      else if(locoList[locoNum].id<10100)
      {
        sprintf(locoId,"%04d",locoList[locoNum].id-10000);
        locoExt=false;
      }
      sprintf(locoVmax,"%03d",locoList[locoNum].vmax&0x1FF);
      locoCrans=locoList[locoNum].vmax&0xC000;
    }
    else
    {
      memset(locoName,' ',sizeof(locoName)-1);
      locoName[sizeof(locoName)-1]=0;
      memset(locoId,'0',sizeof(locoId)-1);
      locoId[sizeof(locoId)-1]=0;
      memset(locoVmax,'0',sizeof(locoVmax)-1);
      locoVmax[sizeof(locoVmax)-1]=0;
      locoExt=true;
      locoCrans=0xC000; // 128 crans
    }
  }

  // Affichage du titre
  if(locoNum!=NEW_LOCO)
    {printAt(2,7,"EDITION");}
  else
    {printAt(2,7,"NOUVEAU");}

  // Affichage des données
  u8g2.setFont(u8g2_font_profont10_mf);
  u8g2.setCursor(30,15);
  u8g2.print(locoName);
  u8g2.setCursor(50,23);
  if(locoExt)
    {
      u8g2.print(locoId);
      printAt(30,23,"EXT");
    }
  else
    {
      u8g2.print(locoId+2);
      printAt(30,23,"STD");
    }

  u8g2.setCursor(30,31);
  u8g2.print(locoVmax);
  u8g2.setCursor(110,31);
  u8g2.print(formatCrans(locoCrans));
  
  u8g2.setFont(u8g2_font_profont10_tf);
  printAt(2,40,"OK  : Enregistre");
  printAt(2,47,"ESC : Annule");
  printAt(2,54,"DEL : Supprime");
  u8g2.drawFrame(0,32,128,24); // Boîte d'information

  u8g2.setDrawColor(2);
  u8g2.drawBox(0,0,128,8); // Barre de titre

  switch(s)
  {
    case 0: // Nom
        u8g2.drawBox(29+p*5,8,6,8);
      break;
    case 1: // Adr
        u8g2.drawBox(29+p*5+(p>0)*15,16,(p>0)?6:18,8);
      break;
    case 2: // Vmax
        u8g2.drawBox(29+p*5,24,6,8);
      break;
    case 3: // Crans
        u8g2.drawBox(110,24,18,8);
      break;
  }
  
  return 0;
}

byte pageListe(byte key)
{
  static byte s=0;
  static byte offset=0;
  switch(key)
  {
    case SW_H :
      if(s>0) s--;
      else if(offset>0) offset--;
      break;
    case SW_B :
      if(s<6) s++;
      else if(offset<numLoco-7) offset++;
      break;
    case SW_G :
      pageEditLoco(0, 0, locoIndex[s+offset]);
      return PAGE_EDIT_LOCO;
    case SW_D :
      pageEditLoco(0, 0, NEW_LOCO);
      return PAGE_EDIT_LOCO;
    case SW_OK :
      pageControl(0,locoIndex[s+offset]);
      return PAGE_CONTROL;
  }
  u8g2.setFont(u8g2_font_profont10_tf);
  u8g2.setDrawColor(1);
  byte y=7;
  byte num=numLoco;
  if(num>7) num=7;
  if((s==0)&&(offset==0)&&(num>4)) num=4;
  for(byte i=0; i<num; i++)
  {
    byte loco=locoIndex[i+offset];
    u8g2.setCursor(2,y);
    char text[17];
    memcpy(text,locoList[loco].name,sizeof(text)-1);
    text[sizeof(text)-1]=0;
    u8g2.print(text);
    u8g2.setCursor(95,y);
    u8g2.print(formatId(locoList[loco].id,text));
    y+=8;
  }
  u8g2.setFont(u8g2_font_open_iconic_all_1x_t);
  u8g2.drawGlyph(118,8,0x4c); // Flèche vers le haut
  u8g2.drawGlyph(118,56,0x49); // Flèche vers le bas
  u8g2.setFont(u8g2_font_profont10_tf);
  if((s==0)&&(offset==0)) // Affichage d'une boîte d'aide
  {
      printAt(2,40,"OK : Pilotage");
      printAt(2,47,"   : Edition");
      printAt(2,54,"   : Nouveau");
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(2,48,0x45); // Flèche vers la gauche
      u8g2.drawGlyph(2,55,0x46); // Flèche vers la droite
      u8g2.drawFrame(0,32,116,24); // Boîte d'information
  }
  u8g2.setDrawColor(2);
  u8g2.drawBox(0,s*8,116,8); // Élément actif
  return 0;
}

// -----------------------------------------------------
// +++ Paramètres
// -----------------------------------------------------

#define LCD_LUMA 13
#define LCD_CONT 12
#define DEFAULT_LUMA 127
#define DEFAULT_CONT 223

byte lcdLuma=DEFAULT_LUMA;
byte lcdCont=DEFAULT_CONT;

void saveControlList(void)
// Sauve l'affectation des contrôleurs dans l'EEPROM
{
  union w_2b num;
  word index=EEPROM_CONTROL;

  for(byte i=0; i<CONTROL_SIZE; i++)
  {
    num.w=controlList[i].num;
    EEPROM.update(index++,num.b[0]);
    EEPROM.update(index++,num.b[1]);
  }
}

void loadControlList(void)
// Lit l'affectation des contrôleurs depuis l'EEPROM
{
  union w_2b num;
  word index=EEPROM_CONTROL;

  for(byte i=0; i<CONTROL_SIZE; i++)
  {
    num.b[0]=EEPROM.read(index++);
    num.b[1]=EEPROM.read(index++);
    if(num.w!=UNDEFINED)
      controlAssign(i, num.w);
  }
}

byte pageParam(byte key)
{
  static byte s=0;
  switch(key)
  {
    case SW_H :
      if(s>0) s--;
      break;
    case SW_B :
      if(s<2) s++;
      break;
  }
  u8g2.setFont(u8g2_font_finderskeepers_tf);
  u8g2.setDrawColor(1);
  printAt(2,8,"CONTROLEURS");
  printAt(2,17,"LUMINOSITE");
  printAt(2,26,"CONTRASTE");
//  printAt(2,35,"REMISE A ZERO"); // TODO

  u8g2.setFont(u8g2_font_open_iconic_all_1x_t);
  u8g2.drawGlyph(118,8,0x4c); // Flèche vers le haut
  u8g2.drawGlyph(118,35,0x49); // Flèche vers le bas
  u8g2.setDrawColor(2);
  u8g2.drawBox(0,s*9,116,9); // Élément actif

  u8g2.setFont(u8g2_font_profont10_tf);
  u8g2.setDrawColor(1);
  switch(s)
  {
    case 0 :
      printAt(2,46,"OK : Enregistre le plan");
      printAt(2,54,"     de pilotage");
      if(key==SW_OK)
      {
        saveControlList();
        // TODO : confirmation de l'enregistrement
      }
      break;
    case 1 :
      printAt(25,46,": R\xe9glage de la");
      printAt(25,54,"  luminosit\xe9");
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(3,48,0x45); // Flèche vers la gauche
      u8g2.drawGlyph(13,48,0x46); // Flèche vers la droite
      switch(key)
      {
        case SW_G :
          if(lcdLuma>16) lcdLuma-=16;
          analogWrite(LCD_LUMA,lcdLuma);
          EEPROM.write(EEPROM_LUMA,lcdLuma);
          break;
        case SW_D :
          if(lcdLuma<=255-16) lcdLuma+=16;
          analogWrite(LCD_LUMA,lcdLuma);
          EEPROM.write(EEPROM_LUMA,lcdLuma);
          break;
      }
      break;
    case 2 :
      printAt(25,46,": R\xe9glage du");
      printAt(25,54,"  contraste");
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(3,48,0x45); // Flèche vers la gauche
      u8g2.drawGlyph(13,48,0x46); // Flèche vers la droite
      switch(key)
      {
        case SW_G :
          if(lcdCont>192) lcdCont-=8;
          analogWrite(LCD_CONT,lcdCont);
          EEPROM.write(EEPROM_CONT,lcdCont);
          break;
        case SW_D :
          if(lcdCont<=255-8) lcdCont+=8;
          analogWrite(LCD_CONT,lcdCont);
          EEPROM.write(EEPROM_CONT,lcdCont);
          break;
      }
      break;
    case 3 :
        // TODO
//      printAt(2,46,"DEL : Effacement de la");
//      printAt(2,54,"      m\xe9moire");
//      if(key==SW_DEL)
//      {
//      }
      break;
  }
  u8g2.drawFrame(0,37,128,19); // Boîte d'information
  return 0;
}

byte pageLogo()
{
  u8g2.setFont(u8g2_font_luBIS18_tr);
  u8g2.setDrawColor(1);
  u8g2.drawStr(4,32,"DiCiCino");
  return 0;
}

byte showPage(byte p,byte key,char num)
{
  byte n;
  u8g2.clearBuffer();
  switch(p)
  {
    case PAGE_LOGO : n=pageLogo(); break;
    case PAGE_PILOT : n=pagePilot(key); break;
    case PAGE_CONTROL : n=pageControl(key); break;
    case PAGE_LISTE : n=pageListe(key); break;
    case PAGE_PARAM : n=pageParam(key); break;
    case PAGE_EDIT_LOCO : n=pageEditLoco(key,num); break;
    case PAGE_DEL_LOCO : n=pageDelLoco(key); break;
  }
  menu(btn1);
  u8g2.updateDisplay();
  return n;
}

// -----------------------------------------------------
// +++ Initialisations
// -----------------------------------------------------

char dccId[4]={'D','C','C',2};

void memInit(void)
{
  EEPROM.write(0,dccId[0]);
  EEPROM.write(1,dccId[1]);
  EEPROM.write(2,dccId[2]);
  EEPROM.write(3,dccId[3]);
  EEPROM.write(EEPROM_LUMA,DEFAULT_LUMA);
  EEPROM.write(EEPROM_CONT,DEFAULT_CONT);

  for(word i=EEPROM_CONTROL; i<4096; i++)
    EEPROM.write(i,0xFF);
}

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  if((EEPROM.read(0)!=dccId[0])||(EEPROM.read(1)!=dccId[1])||(EEPROM.read(2)!=dccId[2])||(EEPROM.read(3)!=dccId[3]))
    memInit();
  
  pinMode(LCD_LUMA,OUTPUT);
  pinMode(LCD_CONT,OUTPUT);
  u8g2.begin();
  lcdLuma=EEPROM.read(EEPROM_LUMA);
  analogWrite(LCD_LUMA,lcdLuma);
  lcdCont=EEPROM.read(EEPROM_CONT);
  analogWrite(LCD_CONT,lcdCont);

//  Serial.print("LUMA ");
//  Serial.println(lcdLuma);
//  Serial.print("CONT ");
//  Serial.println(lcdCont);

  locoListInit();
  locoListSort();
  controlInit();
  StackInit();
  keyInit();

  pinMode(DCC_S,OUTPUT);
  pinMode(DCC_E,OUTPUT);
  pinMode(DCC_C,OUTPUT);
  digitalWrite(DCC_S,LOW);
  digitalWrite(DCC_E,HIGH);
  digitalWrite(DCC_C,LOW);

  FlexiTimer2::set(1, 0.000028, dccInterrupt);  // Démarre l'envoi des données DCC
  FlexiTimer2::start();
  loadControlList();
  showPage(PAGE_LOGO,0,0);
}

// -----------------------------------------------------
// +++ Boucle principale
// -----------------------------------------------------

void loop(void)
{
  static byte page=0; // Page active
  byte key=keyRead();
  byte num=0;
  if(key&0x80) // Boutons-poussoirs
  {
    key&=0x7f;
  }
  else // Pavé numérique
  {
    num=key;
    key=0;
  }
  switch(key)
  {
    case SW_P1 :
      page=PAGE_PILOT;
      break;
    case SW_P2 :
      page=PAGE_CONTROL;
      break;
    case SW_P3 :
      page=PAGE_LISTE;
      break;
    case SW_P4 :
      page=PAGE_PARAM;
      break;
  }
  if(key)
  {
    byte newPage=showPage(page,key,num);
    if(newPage)
    {
      page=newPage;
      showPage(page,0,0);
    }
  }
  byte displayUpdate=controlScan();  // Lecture des contrôleurs
  if(displayUpdate&&(page==PAGE_PILOT))
    showPage(page,0,0);  // Si changement de vitesse ou de sens : mise à jour de l'affichage

  // Affichage d'infos de débogage en appuyant sur la touche ESC
  if(key==SW_ESC)
  {
//    locoDump(); // Liste des décodeurs
//    controlDump(); // Liste des contrôleurs
    stackDump(); // Pile DCC
  }
  delay(50);
}

