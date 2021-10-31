// AT-MULTI2
// Compilation : Arduino Uno / Arduino ISP
// Circuit : KEYPAD SHIELD

// Ce programme est encore en cours d'évolution
// La version la plus récente est disponible à l'adresse suivante :
// https://github.com/EditionsENI/Arduino-et-le-train/tree/master/V2/arduino/at-multi2

#include<Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <FlexiTimer2.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // L'assignation des connexions est imposée par le shield LCD / clavier

#define DATASIZE 4 // On se limite à des trames de 4 octets

#define DISPLAY_WIDTH 16
#define DISPLAY_HEIGHT 2
#define LCD_BACKLIGHT 10

// ========================================
// === INTERFACE UTILISATEUR            ===
// ========================================

#define UI_PAGE_MAIN 1
#define UI_PAGE_PILOT_MAIN 11
#define UI_PAGE_PILOT 12
#define UI_PAGE_PROG_MAIN 21
#define UI_PAGE_I2C_TEST 31
#define UI_PAGE_I2C_SCAN 32
#define UI_PAGE_I2C_ADDR 33
#define UI_PAGE_I2C_MONI 34
#define UI_PAGE_I2C_NAME 35
#define UI_PAGE_PCA9685_MAIN 41
#define UI_PAGE_PCA9685 42
#define UI_PAGE_I2C_AIEA 51
#define UI_PAGE_I2C_AISV 52

#define UI_NUM_MENU_OPTIONS 10

byte uiCurrentPage;
byte uiPageMainCursor;

#define BUTTONS_ANALOG_INPUT 0 // Entrée analogique où sont câblés les boutons
#define SPEED_ANALOG_INPUT 1 // Entrée analogique pour le contrôle de vitesse

#define UI_MODE_DRAW_ALL 1
#define UI_MODE_SET_CURSOR 2
#define UI_MODE_LOOP 4

#define DUI_KEY_NONE   0
#define DUI_KEY_UP     1
#define DUI_KEY_DOWN   2
#define DUI_KEY_LEFT   3
#define DUI_KEY_RIGHT  4
#define DUI_KEY_ESC    5
#define DUI_KEY_OK     6
#define DUI_KEY_SPEED  7

word speedInput;

byte readKeyboard()
// Lecture des boutons : Les boutons forment des ponts diviseurs
// à résistances et sont connectés sur une entrée analogique
// Le shield doit être modifié pour pouvoir gérér un bouton supplémentaire
{
  word keyInput = analogRead(BUTTONS_ANALOG_INPUT);
  static word oldSpeedInput;
  if (keyInput > 980) // S'il n'y a pas de touche enfoncée -> on lit le potentiomètre
  {
    speedInput = analogRead(SPEED_ANALOG_INPUT);
    if (speedInput != oldSpeedInput)
    {
      oldSpeedInput = speedInput;
      return DUI_KEY_SPEED; // Le changement de position du potentiomètre est retourné comme une touche
    }
    return DUI_KEY_NONE;
  }
  if (keyInput < 50)   return DUI_KEY_RIGHT;
  if (keyInput < 180)  return DUI_KEY_UP;
  if (keyInput < 330)  return DUI_KEY_DOWN;
  if (keyInput < 530)  return DUI_KEY_LEFT;
  if (keyInput < 760)  return DUI_KEY_ESC;
  return DUI_KEY_OK;
}

// ========================================
// === Interface DCC                    ===
// ========================================

#define DCC_E 11 // ENABLE (PWM)
#define DCC_S 12 // SIGNAL (DIR)
#define DCC_C 13 // CUTOUT (BRAKE)

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

#define DCC_PACKET_NUM 16
#define DCC_PACKET_SIZE 6 // Taille maximum d'un paquet DCC
#define DCC_HEADER_SIZE 20
#define DCC_CUTOUT_SIZE 18
#define DCC_FUNCTION_MAX 12 // Nombre de fonctions à commander

#define DCC_PACKET_TYPE_MODE 0xF
#define DCC_PACKET_TYPE_SPEED 0  
#define DCC_PACKET_TYPE_F0_F4 1
#define DCC_PACKET_TYPE_F5_F8 2
#define DCC_PACKET_TYPE_F9_F12 3
#define DCC_PACKET_TYPE_CV 8
#define DCC_PACKET_TYPE_CV_BIT 9
#define DCC_PACKET_TYPE_RESET 10

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

byte dccPacketData[DCC_PACKET_NUM][DCC_PACKET_SIZE]; // Paquets de données à envoyer
byte dccPacketSize[DCC_PACKET_NUM]; // Taille des paquets à envoyer
byte dccPacketType[DCC_PACKET_NUM]; // Type des paquets à envoyer

long dccCount=0;

void dumpDccPackets()
// Fonction de débuggage
{
  Serial.println(dccCount);
  static byte counter = 0;
  // if(++counter==10)
  {
    counter = 0;
    for (int i = 0; i < DCC_PACKET_NUM; i++)
    {

      Serial.print(dccPacketSize[i]);
      Serial.print(" [");
      Serial.print(dccPacketType[i]);
      Serial.print("] : ");
      for (int j = 0; j < DCC_PACKET_SIZE; j++)
      {
        Serial.print(dccPacketData[i][j]);
        Serial.write(' ');
      }
      Serial.println();
    }
    Serial.println();
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
				    DccBit = !!(dccPacketData[DccPacketIndex][DccByteCount] & DccBitShift);
				    DccBitShift >>= 1;
				    if (!DccBitShift)
				    {
				      if (dccPacketSize[DccPacketIndex] == ++DccByteCount) // Fin du paquet
				        DccDataMode = DCC_PACKET_STOP;
				      else
				        DccDataMode = DCC_PACKET_START;
				    }
				    break;
				  case DCC_PACKET_STOP :
				    DccBit = 1;
				    if(dccPacketType[DccPacketIndex]&DCC_PACKET_TYPE_CV) // Les paquets de programmation sont effacés après envoi
				    {
				      dccPacketSize[DccPacketIndex] = 0;
				      DccPacketUsed--;  // Suppression du paquet envoyé
				    }
				    if (DccPacketUsed)
				    {
				      for (char i = DCC_PACKET_NUM; --i >= 0;)
				      {
				        DccPacketIndex++;
				        if (DccPacketIndex == DCC_PACKET_NUM) DccPacketIndex = 0;
				        if (dccPacketSize[DccPacketIndex]) break;
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

void dccAdd(byte* packetData, byte packetSize, byte packetType)
// Ajoute un paquet dans la liste d'envoi
{
  if (packetSize > DCC_PACKET_SIZE) return;
  byte index=DCC_PACKET_NUM;
  if(!(packetType&DCC_PACKET_TYPE_CV)) // Paquet de pilotage
    for (index = 0; index < DCC_PACKET_NUM; index++)
      if (dccPacketSize[index]&&(dccPacketType[index]==packetType))  // On remplace un paquet existant
        break;
  if(index==DCC_PACKET_NUM)
    for (index = 0; index < DCC_PACKET_NUM; index++) // On cherche un emplacement libre
      if (!dccPacketSize[index])
        {DccPacketUsed++; break;} // Nouveau paquet
  if (index == DCC_PACKET_NUM) return; // Plus de place libre
  memcpy(dccPacketData[index], packetData, packetSize);
  dccPacketType[index] = packetType;
  dccPacketSize[index] = packetSize;
}

void dccClear()
// Vide la liste d'envoi
{
  for (int i = 0; i < DCC_PACKET_NUM; i++)
  {
    dccPacketData[i][0] = 0xFF;
    dccPacketSize[i] = 0;
  }
  DccPacketUsed = 0;
}

void dccPacketFormat(byte type, word addr, word data1, word data2)
// Formate des données pour former un paquet DCC et le stocker
{
  byte packetData[DCC_PACKET_SIZE];
  byte checksum = 0;
  byte packetSize = 1;
  char* packetPtr = packetData;
  byte dir;
  byte ext;

  if(!(type&DCC_PACKET_TYPE_CV)) // Paquet de pilotage
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
      dir = !!(data1 & 0x100);
      switch (type & DCC_PACKET_TYPE_STEP)
      {
        case DCC_PACKET_TYPE_STEP_14:
          checksum ^= *packetPtr++ = (data1 & 0xF) | (dir ? 0x60 : 0x20);
          packetSize++;
          break;
        case DCC_PACKET_TYPE_STEP_27:
        case DCC_PACKET_TYPE_STEP_28:
          ext = (data1 & 1) << 4;
          data1 >>= 1;
          checksum ^= *packetPtr++ = (data1 & 0xF) | (dir ? 0x60 : 0x20) | ext;
          packetSize++;
          break;
        case DCC_PACKET_TYPE_STEP_128:
          checksum ^= *packetPtr++ = 0x3F;
          checksum ^= *packetPtr++ = (data1 & 0x7F) | (dir ? 0x80 : 0);
          packetSize += 2;
          break;
      }
      break;
    case DCC_PACKET_TYPE_F0_F4:
      checksum ^= *packetPtr++ = 0x80 | (data1 & 0x1F);
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
    case DCC_PACKET_TYPE_CV :   // data1 = Numéro de la variable ; data2 = Valeur
      checksum ^= *packetPtr++ = ((data1 >> 8) & 3) | 0x7C;
      checksum ^= *packetPtr++ = data1 & 0xFF;
      checksum ^= *packetPtr++ = data2 & 0xFF;
      packetSize += 3;
      break;
    case DCC_PACKET_TYPE_CV_BIT :   // data1 = Numéro de la variable ; data2 = masque + bit
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
  dccAdd(packetData, packetSize, type & DCC_PACKET_TYPE_MODE); // TODO pour PILOT
}

void DccReset()
// Envoie une séquence de réinitialisation générale des décodeurs
{
  byte i;
  for (i = 0; i < 5; i++)
    dccPacketFormat(DCC_PACKET_TYPE_RESET, 0xFF, 0, 0);
}

// ========================================
// === PILOTAGE                         ===
// ========================================

#define UI_PAGE_MAIN_CURSOR_ADRMODE 0
#define UI_PAGE_MAIN_CURSOR_ADR0 1
#define UI_PAGE_MAIN_CURSOR_ADR1 2
#define UI_PAGE_MAIN_CURSOR_ADR2 3
#define UI_PAGE_MAIN_CURSOR_ADR3 4
#define UI_PAGE_MAIN_CURSOR_STEP 5

byte uiPageMainValAdrMode = 1;
char uiPageMainValAddressString[5] = "0000";
int uiPageMainValAddress = 0;
byte uiPageMainValStep = 3;

// ----------------------------------------
// --- Page : Pilot                     ---
// ----------------------------------------

byte dccSpeed;
char dccDir = 1;
byte dccFctIndex = 1;
long dccFctField = 0;

void setFunction(byte index)
{
  byte type;
  word data;
  if (index <= 4)
  {
    type = DCC_PACKET_TYPE_F0_F4;
    data = (dccFctField >> (DCC_FUNCTION_MAX - 4)) & 0x1F;
  }
  else if (index <= 8)
  {
    type = DCC_PACKET_TYPE_F5_F8;
    data = (dccFctField >> (DCC_FUNCTION_MAX - 8)) & 0xF;
  }
  else if (index <= 12)
  {
    type = DCC_PACKET_TYPE_F9_F12;
    data = (dccFctField >> (DCC_FUNCTION_MAX - 12)) & 0xF;
  }
  if (uiPageMainValAdrMode) type |= DCC_PACKET_TYPE_ADDR_LONG;
  dccPacketFormat(type, uiPageMainValAddress, data, 0);
}

byte setSpeedAndDir()
{
  byte type;
  switch (uiPageMainValStep)
  {
    case 0 :
      type = DCC_PACKET_TYPE_SPEED | DCC_PACKET_TYPE_STEP_14;
      dccSpeed = map(speedInput, 0, 1023, 0, 14);
      if (dccSpeed) dccSpeed++; // Pas de cran 1
      break;
    case 1 :
      type = DCC_PACKET_TYPE_SPEED | DCC_PACKET_TYPE_STEP_27;
      dccSpeed = map(speedInput, 0, 1023, 0, 27);
      if (dccSpeed) dccSpeed += 3; // Pas de crans 1, 2 et 3
      break;
    case 2 :
      type = DCC_PACKET_TYPE_SPEED | DCC_PACKET_TYPE_STEP_28;
      dccSpeed = map(speedInput, 0, 1023, 0, 28);
      if (dccSpeed) dccSpeed += 3; // Pas de crans 1, 2 et 3
      break;
    case 3 :
      type = DCC_PACKET_TYPE_SPEED | DCC_PACKET_TYPE_STEP_128;
      dccSpeed = map(speedInput, 0, 1023, 0, 126);
      if (dccSpeed) dccSpeed++; // Pas de cran 1
      break;
  }
  if (uiPageMainValAdrMode) type |= DCC_PACKET_TYPE_ADDR_LONG;
  word data = (dccDir > 0) ? 0x100 : 0;
  if (dccDir) data |= dccSpeed;
  dccPacketFormat(type, uiPageMainValAddress, data, 0);
  return (dccSpeed);
}

byte uiPagePilotSpeed(byte button, byte mode)
{
  lcd.setCursor(0, 1);
  byte type;
  byte speedStep = setSpeedAndDir();
  lcd.print(speedStep);
  if (dccSpeed < 100) lcd.write(' ');
  if (dccSpeed < 10) lcd.write(' ');
  return 0;
}

byte uiPagePilotDir(byte button, byte mode)
{
  lcd.setCursor(4, 1);
  switch (button)
  {
    case DUI_KEY_LEFT :
      if (dccDir > -1) dccDir--;
      break;
    case DUI_KEY_RIGHT :
      if (dccDir < 1) dccDir++;
      break;
  }
  switch (dccDir)
  {
    case 1 : lcd.write(0x7E); dccFctField |= 0x1000; break;
    case 0 : lcd.write('-'); dccFctField &= ~0x1000; break;
    case -1 : lcd.write(0x7F); dccFctField |= 0x1000; break;
  }
  setSpeedAndDir();
  setFunction(0); // F0 = éclairage des feux
  return 0;
}

byte uiPagePilotFunction(byte button, byte mode)
{
  lcd.setCursor(8 , 1);
  switch (button)
  {
    case DUI_KEY_UP :
      if (dccFctIndex < 12) dccFctIndex++;
      break;
    case DUI_KEY_DOWN :
      if (dccFctIndex > 1) dccFctIndex--;
      break;
  }
  long fctBit = 1 << (DCC_FUNCTION_MAX - dccFctIndex);
  long fctData = dccFctField & fctBit;
  if (button == DUI_KEY_OK)
  {
    fctData = (~fctData)&fctBit;
    dccFctField = (dccFctField & ~fctBit) | fctData;
  }
  lcd.write('F');
  lcd.print(dccFctIndex);
  if (dccFctIndex < 10) lcd.write(' ');
  lcd.print(fctData ? F(": ON ") : F(": OFF"));
  if (mode & UI_MODE_DRAW_ALL) return 0;
  if (button != DUI_KEY_OK) return 0;
  setFunction(dccFctIndex);
  return 0;
}

byte uiPagePilot(byte button, byte mode)
{
  if(mode==UI_MODE_LOOP) return 0;
  lcd.noBlink();
  if (mode & UI_MODE_DRAW_ALL)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Vitesse Fonction"));
    uiPagePilotSpeed(button, mode);
    uiPagePilotDir(button, mode);
    uiPagePilotFunction(button, mode);
  }
  switch (button)
  {
    case DUI_KEY_SPEED :
      uiPagePilotSpeed(button, mode);
      break;
    case DUI_KEY_LEFT :
    case DUI_KEY_RIGHT :
      uiPagePilotDir(button, mode);
      break;
    case DUI_KEY_UP :
    case DUI_KEY_DOWN :
    case DUI_KEY_OK :
      uiPagePilotFunction(button, mode);
      break;
    case DUI_KEY_ESC :
      return (UI_PAGE_PILOT_MAIN);
  }
  return 0;
}

// ----------------------------------------
// --- Page : Main                      ---
// ----------------------------------------

void uiPageMainAddress(byte button, byte mode)
{
  byte index = uiPageMainCursor - UI_PAGE_MAIN_CURSOR_ADR0;
  if (mode & UI_MODE_SET_CURSOR)
  {
    lcd.setCursor(6 + index, 1);
    return;
  }
  lcd.setCursor(6, 1);
  switch (button)
  {
    case DUI_KEY_UP :
      if (index < sizeof(uiPageMainValAddressString) - 1)
      {
        uiPageMainValAddressString[index]++;
        if (uiPageMainValAddressString[index] > '9') uiPageMainValAddressString[index] = '0';
      }
      break;
    case DUI_KEY_DOWN :
      if (index < sizeof(uiPageMainValAddressString) - 1)
      {
        uiPageMainValAddressString[index]--;
        if (uiPageMainValAddressString[index] < '0') uiPageMainValAddressString[index] = '9';
      }
      break;
    case DUI_KEY_RIGHT :
      uiPageMainCursor++;
      break;
    case DUI_KEY_LEFT :
      uiPageMainCursor--;
      if (!uiPageMainValAdrMode && (uiPageMainCursor < UI_PAGE_MAIN_CURSOR_ADR2))
        uiPageMainCursor = UI_PAGE_MAIN_CURSOR_ADRMODE; // Adresse courte : seulement 2 digits
  }
  if (uiPageMainValAdrMode)
  {
    lcd.print(uiPageMainValAddressString);
  }
  else
  {
    lcd.print(F("  "));
    lcd.print(uiPageMainValAddressString + 2);
  }
}

void uiPageMainAddrMode(byte button, byte mode)
{
  lcd.setCursor(0, 1);
  if (mode & UI_MODE_SET_CURSOR)
    return;
  switch (button)
  {
    case DUI_KEY_UP :
    case DUI_KEY_DOWN :
      uiPageMainValAdrMode = !uiPageMainValAdrMode;
      break;
    case DUI_KEY_RIGHT :
      if (uiPageMainValAdrMode)
        uiPageMainCursor = UI_PAGE_MAIN_CURSOR_ADR0;
      else
        uiPageMainCursor = UI_PAGE_MAIN_CURSOR_ADR2;
  }
  lcd.print(uiPageMainValAdrMode ? F("long: ") : F("court:"));
  uiPageMainAddress(0, 0);
}

void uiPageMainStep(byte button, byte mode)
{
  lcd.setCursor(13, 1);
  if (mode & UI_MODE_SET_CURSOR)
    return;
  switch (button)
  {
    case DUI_KEY_UP :
      uiPageMainValStep++;
      if (uiPageMainValStep > 3) uiPageMainValStep = 0;
      break;
    case DUI_KEY_DOWN :
      uiPageMainValStep--;
      if (uiPageMainValStep > 3) uiPageMainValStep = 3;
      break;
    case DUI_KEY_LEFT :
      uiPageMainCursor--;
  }
  switch (uiPageMainValStep)
  {
    case 0 : lcd.print(F("14 ")); break;
    case 1 : lcd.print(F("27 ")); break;
    case 2 : lcd.print(F("28 ")); break;
    case 3 : lcd.print(F("128")); break;
  }
}

byte uiPageMainPilot(byte button, byte mode)
{
  if(mode==UI_MODE_LOOP) return 0;
  lcd.noBlink();
  if (mode & UI_MODE_DRAW_ALL)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Adresse    Crans"));
    uiPageMainAddrMode(button, mode);
    uiPageMainAddress(button, mode);
    uiPageMainStep(button, mode);
    uiPageMainCursor = UI_PAGE_MAIN_CURSOR_ADRMODE;
  }
  switch (uiPageMainCursor)
  {
    case UI_PAGE_MAIN_CURSOR_ADRMODE :
      uiPageMainAddrMode(button, mode);
      break;
    case UI_PAGE_MAIN_CURSOR_ADR0 :
    case UI_PAGE_MAIN_CURSOR_ADR1 :
    case UI_PAGE_MAIN_CURSOR_ADR2 :
    case UI_PAGE_MAIN_CURSOR_ADR3 :
      uiPageMainAddress(button, mode);
      break;
    case UI_PAGE_MAIN_CURSOR_STEP :
      uiPageMainStep(button, mode);
      break;
  }
  if (button == DUI_KEY_OK)
  {
    if (uiPageMainValAdrMode)
      uiPageMainValAddress = atoi(uiPageMainValAddressString);
    else
      uiPageMainValAddress = atoi(uiPageMainValAddressString + 2);
    dccClear();
    dccFctField = 0;
    return (UI_PAGE_PILOT);
  }
  if (button == DUI_KEY_ESC)
  {
    return (UI_PAGE_MAIN);
  }

  lcd.blink();
  return 0;
}

// ========================================
// === PROGRAMMEUR DE CV                ===
// ========================================

#define BUTTONS_ANALOG_INPUT 0 // Entrée analogique où sont câblés les boutons
#define SPEED_ANALOG_INPUT 1

#define UI_MODE_DRAW_ALL 1
#define UI_MODE_SET_CURSOR 2

#define DUI_KEY_NONE   0
#define DUI_KEY_UP     1
#define DUI_KEY_DOWN   2
#define DUI_KEY_LEFT   3
#define DUI_KEY_RIGHT  4
#define DUI_KEY_ESC    5
#define DUI_KEY_OK     6
#define DUI_KEY_SPEED  7

#define UI_PAGE_MAIN_CURSOR_ADR0 0
#define UI_PAGE_MAIN_CURSOR_ADR1 1
#define UI_PAGE_MAIN_CURSOR_ADR2 2
#define UI_PAGE_MAIN_CURSOR_MODE 3
#define UI_PAGE_MAIN_CURSOR_VAL 4

char uiPageMainValAddrString[4] = "001";
int uiPageMainValAddr = 1;
byte uiPageMainValMode = 0;
word uiPageMainValVal = 0;
byte uiPageMainValBit = 0;
char uiPageMainValValString[9];

// ----------------------------------------
// --- Constantes et variables          ---
// ----------------------------------------

void SetCV(word cvNum, word cvVal, byte cvBit)
{
  byte i;
  for (i = 0; i < 3; i++) dccPacketFormat(DCC_PACKET_TYPE_RESET, 0xFF, 0, 0);
  if (cvBit > 0) // Mode manipulation de bit
  {
    cvBit--;
    byte bitData = !!((1 << cvBit)&cvVal);
    for (i = 0; i < 11; i++) dccPacketFormat(DCC_PACKET_TYPE_CV_BIT, 0xFF, cvNum - 1, (bitData << 3) | cvBit);
  }
  else if (cvNum == 0) // Pseudo-CV pour adresse longue
  {
    byte highVal = (cvVal >> 8) | 0xC0;
    byte lowVal = (cvVal & 0xFF);
    for (i = 0; i < 11; i++) dccPacketFormat(DCC_PACKET_TYPE_CV, 0xFF, 16, highVal);
    delay(2000);  // Attente entre les deux commandes
    for (i = 0; i < 3; i++) dccPacketFormat(DCC_PACKET_TYPE_RESET, 0xFF, 0, 0);
    for (i = 0; i < 11; i++) dccPacketFormat(DCC_PACKET_TYPE_CV, 0xFF, 17, lowVal);
  }
  else // Autres cas
  {
    for (i = 0; i < 11; i++) dccPacketFormat(DCC_PACKET_TYPE_CV, 0xFF, cvNum - 1, cvVal);
  }
  dumpDccPackets();
}

void GetCV()
{

}

// ----------------------------------------
// --- Page : Main                      ---
// ----------------------------------------

void setPageValBit(void)
{
  sprintf(uiPageMainValValString, "%d(%d) %s", uiPageMainValBit + 1, uiPageMainValBit, ((uiPageMainValVal >> uiPageMainValBit) & 1) ? "ON " : "OFF");
}

void uiPageMainVal(byte button, byte mode)
{
  byte index = uiPageMainCursor - UI_PAGE_MAIN_CURSOR_VAL;
  if (mode & UI_MODE_SET_CURSOR)
  {
    lcd.setCursor(6 + index, 1);
    return;
  }
  lcd.setCursor(6, 1);
  switch (button)
  {
    case DUI_KEY_UP :
      uiPageMainValValString[index]++;
      switch (uiPageMainValMode)
      {
        case 0 :
          if (uiPageMainValValString[index] > '9') uiPageMainValValString[index] = '0';
          break;
        case 1 :
          if ((uiPageMainValValString[index] > '9') && (uiPageMainValValString[index] < 'A')) uiPageMainValValString[index] = 'A';
          else if (uiPageMainValValString[index] > 'F') uiPageMainValValString[index] = '0';
          break;
        case 2 :
          if (uiPageMainValValString[index] > '1') uiPageMainValValString[index] = '0';
          break;
        case 3 :
          if (index == 0)
          {
            uiPageMainValBit++;
            if (uiPageMainValBit > 7) uiPageMainValBit = 0;
          }
          else
          {
            uiPageMainValVal ^= 1 << uiPageMainValBit;
          }
          setPageValBit();
          break;
      }
      break;
    case DUI_KEY_DOWN :
      uiPageMainValValString[index]--;
      switch (uiPageMainValMode)
      {
        case 0 :
          if (uiPageMainValValString[index] < '0') uiPageMainValValString[index] = '9';
          break;
        case 1 :
          if (uiPageMainValValString[index] < '0')
            uiPageMainValValString[index] = 'F';
          else if ((uiPageMainValValString[index] < 'A') && (uiPageMainValValString[index] > '9'))
            uiPageMainValValString[index] = '9';
          break;
        case 2 :
          if (uiPageMainValValString[index] < '0') uiPageMainValValString[index] = '1';
          break;
        case 3 :
          if (index == 0)
          {
            uiPageMainValBit--;
            if ((char)uiPageMainValBit < 0) uiPageMainValBit = 7;
          }
          else
          {
            uiPageMainValVal ^= 1 << uiPageMainValBit;
          }
          setPageValBit();
          break;
      }
      break;
    case DUI_KEY_RIGHT :
      switch (uiPageMainValMode)
      {
        case 0 : if (index < ((uiPageMainValAddr == 0) ? 3 : 2)) uiPageMainCursor++; break;
        case 1 : if (index < 1) uiPageMainCursor++; break;
        case 2 : if (index < 7) uiPageMainCursor++; break;
        case 3 : if (index == 0) uiPageMainCursor += 5; break;
      }
      break;
    case DUI_KEY_LEFT :
      if ((uiPageMainValMode == 3) && (index == 5)) uiPageMainCursor -= 5;
      else uiPageMainCursor--;
      break;
  }
  if ((uiPageMainValMode == 0) && (uiPageMainValAddr != 0) && (atoi(uiPageMainValValString) > 255))
    memcpy(uiPageMainValValString, "255 ", 4);
  lcd.print(uiPageMainValValString);
  switch (uiPageMainValMode)
  {
    case 0 :
      uiPageMainValVal = atoi(uiPageMainValValString);
      break;
    case 1 :
      if (uiPageMainValValString[0] <= '9') uiPageMainValVal = uiPageMainValValString[0] - '0';
      else uiPageMainValVal = uiPageMainValValString[0] - 'A' + 10;
      uiPageMainValVal <<= 4;
      if (uiPageMainValValString[1] <= '9') uiPageMainValVal |= uiPageMainValValString[1] - '0';
      else uiPageMainValVal |= uiPageMainValValString[1] - 'A' + 10;
      break;
    case 2 :
      uiPageMainValVal = 0;
      for (int i = 0; i < 8; i++)
        uiPageMainValVal = (uiPageMainValVal << 1) | ((uiPageMainValValString[i] == '1') ? 1 : 0);
      break;
    case 3 :
      break;
  }
}

void uiPageMainMode(byte button, byte mode)
{
  lcd.setCursor(0, 1);
  if (mode & UI_MODE_SET_CURSOR)
    return;
  switch (button)
  {
    case DUI_KEY_UP :
      uiPageMainValMode++;
      if (uiPageMainValMode > 3) uiPageMainValMode = 0;
      break;
    case DUI_KEY_DOWN :
      uiPageMainValMode--;
      if (uiPageMainValMode > 3) uiPageMainValMode = 3;
      break;
    case DUI_KEY_RIGHT :
      uiPageMainCursor++;
      break;
    case DUI_KEY_LEFT :
      uiPageMainCursor--;
      break;
  }
  if (uiPageMainValAddr == 0) // adresse longue
    uiPageMainValMode = 0; // mode DEC obligatoire
  switch (uiPageMainValMode)
  {
    case 0 :
      lcd.print(F("DEC :"));
      if (uiPageMainValAddr == 0)
        sprintf(uiPageMainValValString, "%04d     ", uiPageMainValVal);
      else
        sprintf(uiPageMainValValString, "%03d     ", uiPageMainValVal);
      break;
    case 1 :
      lcd.print(F("HEX :"));
      sprintf(uiPageMainValValString, "%02X      ", uiPageMainValVal);
      break;
    case 2 :
      lcd.print(F("BIN :"));
      for (int i = 0; i < 8; i++)
        uiPageMainValValString[i] = ((uiPageMainValVal << i) & 0x80) ? '1' : '0';
      break;
    case 3 :
      lcd.print(F("BIT :"));
      setPageValBit();
      break;
  }
  uiPageMainVal(0, 0);
}

void uiPageMainAddr(byte button, byte mode)
{
  byte index = uiPageMainCursor - UI_PAGE_MAIN_CURSOR_ADR0;
  if (mode & UI_MODE_SET_CURSOR)
  {
    lcd.setCursor(2 + index, 0);
    return;
  }
  lcd.setCursor(2, 0);
  switch (button)
  {
    case DUI_KEY_UP :
      if (index < sizeof(uiPageMainValAddrString) - 1)
      {
        uiPageMainValAddrString[index]++;
        if (uiPageMainValAddrString[index] > '9')
        {
          uiPageMainValAddrString[index] = '0';
          if (index > 0)
          {
            uiPageMainValAddrString[index - 1]++;
          }
          if (uiPageMainValAddrString[index - 1] > '9')
          {
            uiPageMainValAddrString[index - 1] = '0';
            if (index > 1)
            {
              uiPageMainValAddrString[index - 2]++;
              if (uiPageMainValAddrString[index - 2] > '9')
                uiPageMainValAddrString[index - 2] = '0';
            }
          }
        }
      }
      break;
    case DUI_KEY_DOWN :
      if (index < sizeof(uiPageMainValAddrString) - 1)
      {
        uiPageMainValAddrString[index]--;
        if (uiPageMainValAddrString[index] < '0')
        {
          uiPageMainValAddrString[index] = '9';
          if (index > 0)
          {
            uiPageMainValAddrString[index - 1]--;
          }
          if (uiPageMainValAddrString[index - 1] < '0')
          {
            uiPageMainValAddrString[index - 1] = '9';
            if (index > 1)
            {
              uiPageMainValAddrString[index - 2]--;
              if (uiPageMainValAddrString[index - 2] < '0')
                uiPageMainValAddrString[index - 2] = '9';
            }
          }
        }
      }
      break;
    case DUI_KEY_RIGHT :
      uiPageMainCursor++;
      break;
    case DUI_KEY_LEFT :
      uiPageMainCursor--;
      break;
  }
  uiPageMainValAddr = atoi(uiPageMainValAddrString);
  lcd.print(uiPageMainValAddrString);
  if (uiPageMainValAddr);
  uiPageMainMode(0, 0);
  lcd.setCursor(6, 0);
  switch (uiPageMainValAddr)
  {
    case 0  : lcd.print(F("adr longue")); break;
    case 1  : lcd.print(F("adr courte")); break;
    case 2  : lcd.print(F("V min dem ")); break;
    case 3  : lcd.print(F("tempo acc ")); break;
    case 4  : lcd.print(F("tempo stop")); break;
    case 5  : lcd.print(F("V max     ")); break;
    case 6  : lcd.print(F("V mid     ")); break;
    case 7  : lcd.print(F("version   ")); break;
    case 8  : lcd.print(F("code const")); break;
    case 9  : lcd.print(F("periode   ")); break;
    case 10 : lcd.print(F("cut moteur")); break;
    case 11 : lcd.print(F("delai stop")); break;
    case 12 : lcd.print(F("conv alim ")); break;
    case 13 : lcd.print(F("alt F1-8  ")); break;
    case 14 : lcd.print(F("alt F9-12 ")); break;
    case 15 : lcd.print(F("verrou 1  ")); break;
    case 16 : lcd.print(F("verrou 2  ")); break;
    case 17 : lcd.print(F("ext adr hi")); break;
    case 18 : lcd.print(F("ext adr lo")); break;
    case 19 : lcd.print(F("adr multi ")); break;
    case 21 : lcd.print(F("multi F1-8")); break;
    case 22 : lcd.print(F("multiF9-12")); break;
    case 23 : lcd.print(F("mod accel ")); break;
    case 24 : lcd.print(F("mod stop  ")); break;
    case 25 : lcd.print(F("pos moyen ")); break;
    case 27 : lcd.print(F("arret auto")); break;
    case 28 : lcd.print(F("RailCom   ")); break;
    case 29 : lcd.print(F("config    ")); break;
    case 30 : lcd.print(F("erreur    ")); break;
    //    case 00 : lcd.print(F("")); break;
    default : lcd.print(F("          "));
  }
}

byte uiPageMainProg(byte button, byte mode)
{
  if(mode==UI_MODE_LOOP) return 0;
  lcd.noBlink();
  if (mode & UI_MODE_DRAW_ALL)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("cv"));
    uiPageMainAddr(button, mode);
    uiPageMainMode(button, mode);
    uiPageMainVal(button, mode);
    uiPageMainCursor = UI_PAGE_MAIN_CURSOR_ADR0;
  }
  switch (uiPageMainCursor)
  {
    case UI_PAGE_MAIN_CURSOR_ADR0 :
    case UI_PAGE_MAIN_CURSOR_ADR1 :
    case UI_PAGE_MAIN_CURSOR_ADR2 :
      uiPageMainAddr(button, mode);
      break;
    case UI_PAGE_MAIN_CURSOR_MODE :
      uiPageMainMode(button, mode);
      break;
    default :
      uiPageMainVal(button, mode);
  }
  switch (button)
  {
    case DUI_KEY_ESC :
      uiPageMainValVal = 0;
      uiPageMainValBit = 0;
      uiPageMainMode(0, 0);
      uiPageMainVal(0, 0);
      return UI_PAGE_MAIN;
    case DUI_KEY_OK :
      dccClear();
      SetCV(uiPageMainValAddr, uiPageMainValVal, (uiPageMainValMode == 3) ? uiPageMainValBit + 1 : 0);
      break;
  }
  lcd.blink();
  return 0;
}

// ========================================
// === SCANNNER I²C                     ===
// ========================================

byte uiPageMainI2CScan(byte button, byte mode)
{
  static byte baseAddr=0x10;
  if(mode==UI_MODE_LOOP)
  {
    char text[4];
    for(int i=0; i<8; i++)
    {
      lcd.setCursor((i<<2)&0xF,(i>>2)&1);
      if(baseAddr+i==0x7F) {lcd.print(F("   "));break;}

      sprintf(text,"%02X",baseAddr+i);
      lcd.print(text);
  
      Wire.beginTransmission(baseAddr+i);
      byte ret = Wire.endTransmission();
      lcd.write(ret?' ':0xFF);
    }
  }

  lcd.noBlink();
  if (mode & UI_MODE_DRAW_ALL)
  {
    lcd.clear();
  }
  switch (button)
  {
    case DUI_KEY_UP :
      if(baseAddr>=0x18)
        baseAddr-=8;
      else
        baseAddr=0x78;
      break;
    case DUI_KEY_DOWN :
      if(baseAddr<=0x70)
        baseAddr+=8;
      else
        baseAddr=0x10;
      break;
    case DUI_KEY_ESC :
      return UI_PAGE_MAIN;
  }
  return 0;
}

// ========================================
// === TESTEUR I²C                      ===
// ========================================

byte uiPageHexInput(byte button, byte x,byte y, byte hiLo, byte val)
// Saisie d'une valeur hexadécimale
// Fonction utilisée par la plupart des fonctions I²C
// button : bouton enfoncé (DUI_KEY_UP, DUI_KEY_DOWN)
// x,y : position du premier digit (celui de poids fort, à gauche)
// hiLo : digit à modifier : 0 = poids fort, à gauche ; 1 = poids faible, à droite; 2 = pas de modification
// val : valeur à modifier
// -> valeur modifiée
{
  if(hiLo!=2)
  {
    switch (button)
    {
      case DUI_KEY_UP :
        val+=hiLo?1:0x10;
        break;
      case DUI_KEY_DOWN :
        val-=hiLo?1:0x10;
        break;
    }
  }
  lcd.setCursor(x, y);
  if(val<0x10)
    lcd.write('0');
  lcd.print(val,HEX);

  if(hiLo!=2)
  {
    lcd.setCursor(x+hiLo, y);
    lcd.blink();
  }
  return val;
}

byte uiPageMainI2CTest(byte button, byte mode)
{
  static byte i2c_addr=0;
  static byte pos=0;  // position du curseur
  static byte test=0; // 0 = choix de l'adresse ; 1 = test en cours
  static byte ok=0x80; // Indicateur de bon fonctionnement

  if(!test)
  {
    if(mode & UI_MODE_DRAW_ALL)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("TEST :"));
      uiPageHexInput(0,7,0,0,i2c_addr);
      return 0;
    }
    switch (button)
    {
      case DUI_KEY_LEFT :
        if(pos>0) pos--;
        break;
      case DUI_KEY_RIGHT :
        if(pos<1) pos++;
        break;
      case DUI_KEY_ESC :
        return UI_PAGE_MAIN;
      case DUI_KEY_OK :
        test=1;
        ok=0;
        return 0;
    }
    i2c_addr=uiPageHexInput(button,7,0,pos,i2c_addr);
    return 0;
  }

  switch (button)
  {
    case DUI_KEY_ESC :
      test=0;
      return UI_PAGE_I2C_TEST; // Provoque le réaffichage de la page
  }

  if(mode!=UI_MODE_LOOP) return 0;

  lcd.noBlink();
  // Prépare les données à envoyer
  byte writeData[DATASIZE];
  byte readData[DATASIZE];
  unsigned long micSec = micros(); // le compteur de microsecondes
  byte i;
  for(i=0; i<DATASIZE; i++)
  {
    writeData[i]=micSec&0xFF; // les microsecondes servent de données
    micSec>>=8;
  }
  
  // Envoi des données
  Wire.beginTransmission(i2c_addr);
  Wire.write(0x22);
  Wire.write(sizeof(writeData));
  Wire.write(writeData, sizeof(writeData));
  Wire.endTransmission(); // effectue l'envoi et termine
  delay(100); // on attend

  // Réception des données
  digitalWrite(LED_BUILTIN, HIGH); // Pour faire un flash si OK
  Wire.beginTransmission(i2c_addr);
  Wire.write(0x11);
  Wire.write(sizeof(readData)); // Demande de lecture
  Wire.endTransmission(); // effectue l'envoi et termine
  Wire.requestFrom(i2c_addr,sizeof(readData)); // Attend les données 
  i=0;
  while(Wire.available()) // Tant que des données sont disponibles
  {
    readData[i++]=Wire.read(); // Lit les données
    if(i==sizeof(readData)) break; // Pour éviter un buffer overflow
  }

  // Vérification des données reçues
  if(!memcmp(writeData,readData,DATASIZE)) // Compare les deux trames
  { // OK
    if(ok==0x80)
    {
      lcd.setCursor(0,1);
      lcd.print(F("                "));
      ok=0;
    }
    lcd.setCursor(ok&0xF,1); // Animation pour indiquer le bon fonctionnement
    lcd.write(ok&0x10?' ':0xFF);
    ok=(ok+1)&0x1F;
  }
  else // ERREUR
  {
    lcd.setCursor(0,1);
    lcd.print(F("ERREUR          "));
    ok=0x80;
    digitalWrite(LCD_BACKLIGHT, 0); // Fait clignoter l'afficheur LCD
    delay(100);
    digitalWrite(LCD_BACKLIGHT, 1);
  }
  return 0;
}

// ========================================
// === Changement d'adresse I²C         ===
// ========================================

byte uiPageMainI2CAddr(byte button, byte mode)
{
  static byte addrFrom=0;
  static byte addrTo=0;
  static byte pos=0;  // position du curseur

  if(mode & UI_MODE_DRAW_ALL)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("ADDR : 00 \x7E 00"));
    uiPageHexInput(0,7,0,0,addrFrom);
    uiPageHexInput(0,12,0,0,addrTo);
    return 0;
  }
  switch (button)
  {
    case DUI_KEY_LEFT :
      if(pos>0) pos--;
      break;
    case DUI_KEY_RIGHT :
      if(pos<3) pos++;
      break;
    case DUI_KEY_ESC :
      return UI_PAGE_MAIN;
    case DUI_KEY_OK :
      lcd.setCursor(0, 1);
      // Gestion de tous les cas d'erreur possibles
      if((addrFrom<0x10)||(addrFrom>0x7E))
      {
        lcd.print(F("Adr src interdit"));
        break;
      }
      if((addrTo<0x10)||(addrTo>0x7E))
      {
        lcd.print(F("Adr dst interdit"));
        break;
      }
      Wire.beginTransmission(addrFrom);
      if(Wire.endTransmission())
      {
        lcd.print(F("Adr src erreur  "));
        break;
      }
      Wire.beginTransmission(addrTo);
      if(!Wire.endTransmission())
      {
        lcd.print(F("Adr dst occupee "));
        break;
      }
      // Programmation du changement d'adresse
      Wire.beginTransmission(addrFrom);
      Wire.write(0);
      Wire.write(addrTo);
      Wire.write(0xFF);
      Wire.write(~addrTo);
      Wire.endTransmission(); // effectue l'envoi
      delay(1000); // on attend que la programmation se fasse
      // Teste si la programmation a réussi
      Wire.beginTransmission(addrTo);
      if(!Wire.endTransmission())
        lcd.print(F("Programmation OK"));
      else
        lcd.print(F("ERREUR prog     "));
      return 0;
  }
  switch(pos)
  {
    case 0 :
    case 1 :
      addrFrom=uiPageHexInput(button,7,0,pos,addrFrom);
      break;
    case 2 :
    case 3 :
      addrTo=uiPageHexInput(button,12,0,pos&1,addrTo);
      break;
  }
  return 0;
}

// ========================================
// === Moniteur I²C                     ===
// ========================================

// Caractères programmables pour numéroter les octets
uint8_t charNum[20][8]=
{
  {7,5,5,5,7,0,0}, // 0
  {6,2,2,2,2,0,0}, // 1
  {7,1,7,4,7,0,0}, // 2
  {7,1,7,1,7,0,0}, // 3
  {5,5,7,1,1,0,0}, // 4
  {7,4,7,1,7,0,0}, // 5
  {7,4,7,5,7,0,0}, // 6
  {7,1,1,1,1,0,0}, // 7
  {7,5,7,5,7,0,0}, // 8
  {7,5,7,1,7,0,0}, // 9
  {0x17,0x15,0x15,0x15,0x17,0,0}, // 10
  {0x12,0x12,0x12,0x12,0x12,0,0}, // 11
  {0x17,0x11,0x17,0x14,0x17,0,0}, // 12
  {0x17,0x11,0x17,0x11,0x17,0,0}, // 13
  {0x15,0x15,0x17,0x11,0x11,0,0}, // 14
  {0x17,0x14,0x17,0x11,0x17,0,0}, // 15
  {0x17,0x14,0x17,0x15,0x17,0,0}, // 16
  {0x17,0x11,0x11,0x11,0x11,0,0}, // 17
  {0x17,0x15,0x17,0x15,0x17,0,0}, // 18
  {0x17,0x15,0x17,0x11,0x17,0,0}  // 19
};

byte uiPageMainI2CMoni(byte button, byte mode)
{
  static byte addr=0;
  static byte pos=0;  // position du curseur
  static byte data[20];
  static byte dataSize=1;
  static byte offset=0;
  byte redraw=false;

  if(mode & UI_MODE_DRAW_ALL)
  {
    addr=0;
    pos=0;
    dataSize=1;
    offset=0;
    memset(data,0,sizeof(data));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("00 : 00"));
    return 0;
  }
  switch (button)
  {
    case DUI_KEY_LEFT :
      if((pos==2)&&(offset>0)) {offset--; redraw=true;}
      else if(pos>0) pos--;
      break;
    case DUI_KEY_RIGHT :
      if(pos<9) pos++;
      else if(offset<16) {offset++; redraw=true;}
      if((pos/2+offset)>dataSize) dataSize=(pos/2+offset);
      break;
    case DUI_KEY_ESC :
      if(dataSize==1) return UI_PAGE_MAIN;
      lcd.setCursor(dataSize*3+1, 0);
      lcd.print(F("   "));
      dataSize--;
      if(offset>0) {offset--; redraw=true;}
      if(pos/2>dataSize) pos=dataSize<<1;
      break;
    case DUI_KEY_OK :
      lcd.setCursor(0, 1);
      if((addr<0x10)||(addr>0x7E))
      {
        lcd.print(F("Adr interdite   "));
        break;
      }
      // Envoi des données
      Wire.beginTransmission(addr);
      for(byte i=0; i<dataSize; i++)
      {
        Wire.write(data[i]);
      }
      if(Wire.endTransmission())
      {
        lcd.print(F("Pas de reponse  "));
        break;
      }
      delay(100);
      // Lecture et affichage de la réponse
      lcd.setCursor(0,1);
      lcd.print(F("OK :            "));
      byte numBytes=Wire.requestFrom(addr,(byte)4); // Attend les données 
      for(int i=0; (i<numBytes)&&Wire.available(); i++) // Tant que des données sont disponibles
      {
        byte readData=Wire.read(); // Lit les données
        lcd.setCursor(5+i*3, 1);
        if(readData<0x10) lcd.print('0');
        lcd.print(readData, HEX);
        if(i==4) break; // Pour éviter un buffer overflow
      }
      return 0;
  }
  lcd.createChar(0, charNum[offset]);
  lcd.setCursor(4, 0);
  lcd.write((byte)0);
  if(redraw) uiPageHexInput(0,5,0,2,data[offset]);
  if(dataSize>1)
  {
    lcd.createChar(1, charNum[offset+1]);
    lcd.setCursor(7, 0);
    lcd.write((byte)1);
    if(redraw) uiPageHexInput(0,8,0,2,data[offset+1]);
  }
  if(dataSize>2)
  {
    lcd.createChar(2, charNum[offset+2]);
    lcd.setCursor(10, 0);
    lcd.write((byte)2);
    if(redraw) uiPageHexInput(0,11,0,2,data[offset+2]);
  }
  if(dataSize>3)
  {
    lcd.createChar(3, charNum[offset+3]);
    lcd.setCursor(13, 0);
    lcd.write((byte)3);
    if(redraw) uiPageHexInput(0,14,0,2,data[offset+3]);
  }
  switch(pos/2)
  {
    case 0 :
      addr=uiPageHexInput(button,0,0,pos,addr);
      break;
    case 1 :
    case 2 :
    case 3 :
    case 4 :
      byte index=(pos/2)+offset-1;
      data[index]=uiPageHexInput(button,(pos/2)*3+2,0,pos&1,data[index]);
      break;
  }
  for(byte i=0; i<dataSize; i++)
  {
    byte b=data[i];
    if(b<0x10) Serial.print("0");
    Serial.print(b,HEX);
    Serial.print(" ");
  }
  Serial.print(" pos=");
  Serial.print(pos);
  Serial.print(" offset=");
  Serial.print(offset);
  Serial.print(" dataSize=");
  Serial.print(dataSize);
  Serial.println();
  return 0;
}

// ========================================
// === Lecture de l'identifiant I²C     ===
// ========================================

byte cmdName[4]={0,0xFF,0,0xFF};

byte uiPageMainI2CName(byte button, byte mode)
{
  static byte addr=0;
  static byte pos=0;  // position du curseur
  static byte data[4];
  static byte dataSize=1;

  if(mode & UI_MODE_DRAW_ALL)
  {
    addr=0;
    pos=0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("00 :"));
    return 0;
  }
  switch (button)
  {
    case DUI_KEY_LEFT :
      if(pos>0) pos--;
      break;
    case DUI_KEY_RIGHT :
      if(pos<1) pos++;
      break;
    case DUI_KEY_ESC :
      return UI_PAGE_MAIN;
    case DUI_KEY_OK :
      lcd.setCursor(0, 1);
      if((addr<0x10)||(addr>0x7E))
      {
        lcd.print(F("Adr interdite   "));
        break;
      }
      // Envoi des données
      Wire.beginTransmission(addr);
      Wire.write(cmdName,sizeof(cmdName));
      if(Wire.endTransmission())
      {
        lcd.print(F("Pas de reponse  "));
        break;
      }
      delay(100);
      // Lecture et affichage de la réponse
      lcd.setCursor(0,1);
      lcd.print(F("OK              "));
      byte numBytes=Wire.requestFrom(addr,(byte)4); // Attend les données 
      for(int i=0; (i<numBytes)&&Wire.available(); i++) // Tant que des données sont disponibles
      {
        byte readData=Wire.read(); // Lit les données
        lcd.setCursor(5+i, 0);
        lcd.write((char)readData);
      }
      return 0;
  }
  addr=uiPageHexInput(button,0,0,pos,addr);
  return 0;
}

// ========================================
// === Pilotage direct du PCA9685       ===
// ========================================

// Caractères programmables pour faire une échelle graduée
uint8_t charGrad0[8]  = {0,0,0,0,0,0x15,0x15};
uint8_t charGrad1[8]  = {0x10,0x10,0x10,0x10,0,0x15,0x15};
uint8_t charGrad2[8]  = {4,4,4,4,0,0x15,0x15};
uint8_t charGrad3[8]  = {1,1,1,1,0,0x15,0x15};

byte PCA9685addr=0;

byte uiPagePCA9685(byte button, byte mode)
{
  static Adafruit_PWMServoDriver pwm;
  static byte selOut=0;  // sortie sélectionnée
  static byte curPos, oldPos;
  char text[4];
  word out;
  if(mode & UI_MODE_DRAW_ALL)
  {
    pwm = Adafruit_PWMServoDriver(PCA9685addr);
    pwm.begin();        // Initialisation du circuit
    pwm.setPWMFreq(50); // Fréquence de fonctionnement
    selOut=0;
    lcd.createChar(0, charGrad0);
    lcd.createChar(1, charGrad1);
    lcd.createChar(2, charGrad2);
    lcd.createChar(3, charGrad3);
    lcd.clear();
    lcd.setCursor(0, 1);
    for(int i=0; i<12; i++) lcd.write((byte)0);
    lcd.setCursor(0, 0);
    lcd.print(F("0123456789ABCDEF"));
    lcd.blink();
    button=DUI_KEY_SPEED; // Pour afficher le curseur dès le démarrage
  }
  switch (button)
  {
    case DUI_KEY_LEFT :
      if(selOut>0) selOut--;
      button=DUI_KEY_SPEED;
      break;
    case DUI_KEY_RIGHT :
      if(selOut<15) selOut++;
      button=DUI_KEY_SPEED;
      break;
    case DUI_KEY_ESC :
      return UI_PAGE_PCA9685_MAIN;
  }
  if(button==DUI_KEY_SPEED) // Action sur le potentiomètre
  {
      out=(speedInput*3)>>3;
      curPos=out>>5;
      if(curPos!=oldPos)
      {
        lcd.setCursor(oldPos, 1);
        lcd.write((byte)0);
      }
      lcd.setCursor(curPos, 1);
      lcd.write((byte)(((out&0x1F)<11)?1:(((out&0x1F)<22)?2:3)));
      oldPos=curPos;
      out+=64;
      sprintf(text,"%3d",out);
      lcd.setCursor(13, 1);
      lcd.print(text);
      lcd.setCursor(selOut, 0);
      pwm.setPWM(selOut, 0, out); // Fixe la valeur du servo
  }
  return 0;
}

byte uiPageMainPCA9685(byte button, byte mode)
{
  static byte pos=0;  // position du curseur

  if(mode & UI_MODE_DRAW_ALL)
  {
    PCA9685addr=0;
    pos=0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("PCA9685 : 00"));
    return 0;
  }
  switch (button)
  {
    case DUI_KEY_LEFT :
      if(pos>0) pos--;
      break;
    case DUI_KEY_RIGHT :
      if(pos<1) pos++;
      break;
    case DUI_KEY_ESC :
      return UI_PAGE_MAIN;
    case DUI_KEY_OK :
      lcd.setCursor(0, 1);
      if((PCA9685addr<0x10)||(PCA9685addr>0x7E))
      {
        lcd.print(F("Adr interdite   "));
        break;
      }
      return UI_PAGE_PCA9685;
  }
  PCA9685addr=uiPageHexInput(button,10,0,pos,PCA9685addr);
  return 0;
}

// =======================================================
// === Test et programmation aiguillage électro-aimant ===
// =======================================================

uint8_t charAigDir[8]  = {0x11,0x12,0x14,0x10,0x10,0x10,0x10};
uint8_t charAigDev[8]  = {0x11,0x12,0x4,0x8,0x10,0x10,0x10};

byte uiPageMainProgAIEA(byte button, byte mode)
{
  static byte addr=0;
  static byte pos=0; // Position du curseur
  static byte dir=0; // Programmation en voie directe
  static byte dev=0; // Programmation en voie déviée
  static byte num=0; // Identifiant de l'aiguillage
  static byte dirDev=0; // Voie active pour le test

  if(mode & UI_MODE_DRAW_ALL)
  {
    lcd.createChar(0, charAigDir);
    lcd.createChar(1, charAigDev);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("ea 00: \x8=00 \x9=00"));
    lcd.setCursor(2, 1);
    lcd.print(F("TEST \x8 PROG 00"));
    lcd.setCursor(0, 0);
    lcd.blink();
    pos=0;
    dir=0;
    dev=0;
    num=0;
    dirDev=0;
    return 0;
  }
  switch (button)
  {
    case DUI_KEY_LEFT :
      if(pos>0) pos--;
      break;
    case DUI_KEY_RIGHT :
      if(pos<8) pos++;
      break;
    case DUI_KEY_ESC :
      return UI_PAGE_MAIN;
    case DUI_KEY_OK :
      if((pos==7)||(pos==8)) // La programmation n'est possible que si le curseur est sur la case PROG
      {
          Wire.beginTransmission(addr);
          Wire.write((byte)0x34); // Programmation de la configuration
          Wire.write(num);
          Wire.write(dir);
          Wire.write(dev);
          Wire.endTransmission();
      }
      return 0;
  }
  switch(pos)
  {
    case 0 :
    case 1 :
      addr=uiPageHexInput(button,3,0,pos,addr);
      break;
    case 2 :
    case 3 :
      dir=uiPageHexInput(button,9,0,pos&1,dir);
      break;
    case 4 :
    case 5 :
      dev=uiPageHexInput(button,14,0,pos&1,dev);
      break;
    case 6 :
      lcd.setCursor(7, 1);
      switch (button)
      {
        case DUI_KEY_UP :
        case DUI_KEY_DOWN :
          dirDev=!dirDev;
          lcd.write(dirDev);
          Wire.beginTransmission(addr);
          Wire.write((byte)0x36); // Adressage direct
          Wire.write(dirDev?dev:dir);
          Wire.endTransmission();
          break;
      }
      lcd.setCursor(7, 1);
      break;
    case 7 :
    case 8 :
      num=uiPageHexInput(button,14,1,pos-7,num);
      break;
  }
  return 0;
}

// =======================================================
// === Test et programmation aiguillage servomoteur    ===
// =======================================================

byte uiPageMainProgAISV(byte button, byte mode)
{
  static byte addr=0;
  static byte pos=0; // Position du curseur
  static byte dir=0; // Programmation en voie directe
  static byte dev=0; // Programmation en voie déviée
  static byte num=0; // Identifiant de l'aiguillage
  static byte dirDev=0; // Voie active pour le test

  if(mode & UI_MODE_DRAW_ALL)
  {
    lcd.createChar(0, charAigDir);
    lcd.createChar(1, charAigDev);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("00: 00 \x8=00 \x9=00"));
    lcd.setCursor(2, 1);
    lcd.print(F("TEST \x8  PROG "));
    lcd.write(0x7E);
    lcd.setCursor(0, 0);
    lcd.blink();
    pos=0;
    dir=0;
    dev=0;
    num=0;
    dirDev=0;
    return 0;
  }
  switch (button)
  {
    case DUI_KEY_LEFT :
      if(pos>0) pos--;
      else pos=8; // Permet de passer à la case test sans passer par les réglages de position
      break;
    case DUI_KEY_RIGHT :
      if(pos<9) pos++;
      else pos=0;
      break;
    case DUI_KEY_ESC :
      return UI_PAGE_MAIN;
    case DUI_KEY_OK :
      if(pos==9) // La programmation n'est possible que si le curseur est sur la case PROG
      {
          Wire.beginTransmission(addr);
          Wire.write((byte)0x34); // Programmation de la configuration
          Wire.write(num);
          Wire.write(dir);
          Wire.write(dev);
          Wire.endTransmission();
      }
      return 0;
  }
  switch(pos)
  {
    case 0 :
    case 1 :
      addr=uiPageHexInput(button,0,0,pos,addr);
      break;
    case 2 :
    case 3 :
      num=uiPageHexInput(button,4,0,pos-2,num);
      Wire.beginTransmission(addr);
      Wire.write((byte)0x35); // Adressage direct
      Wire.write(num);
      Wire.write(dir);
      Wire.endTransmission();
      Wire.requestFrom(addr,(byte)2);
      dir=Wire.read(); // Lit la programmation existante
      dev=Wire.read();
      dir=uiPageHexInput(0,9,0,0,dir);
      dev=uiPageHexInput(0,14,0,0,dev);
      uiPageHexInput(0,4,0,pos-2,num);
      break;
    case 4 :
    case 5 :
      if(button==DUI_KEY_SPEED)
        dir=uiPageHexInput(0,9,0,pos&1,speedInput>>1);
       else
        dir=uiPageHexInput(button,9,0,pos&1,dir);
      Wire.beginTransmission(addr);
      Wire.write((byte)0x37); // Adressage direct
      Wire.write(num);
      Wire.write(dir);
      Wire.endTransmission();
      break;
    case 6 :
    case 7 :
      if(button==DUI_KEY_SPEED)
        dev=uiPageHexInput(0,14,0,pos&1,speedInput>>1);
       else
        dev=uiPageHexInput(button,14,0,pos&1,dev);
      Wire.beginTransmission(addr);
      Wire.write((byte)0x37); // Adressage direct
      Wire.write(num);
      Wire.write(dev);
      Wire.endTransmission();
      break;
    case 8 :
      lcd.setCursor(7, 1); // Test de l'aiguillage
      switch (button)
      {
        case DUI_KEY_UP :
        case DUI_KEY_DOWN :
          dirDev=!dirDev;
          lcd.write(dirDev);
          Wire.beginTransmission(addr);
          Wire.write((byte)0x37); // Adressage direct
          Wire.write(num);
          Wire.write(dirDev?dev:dir);
          Wire.endTransmission();
          break;
      }
      lcd.setCursor(7, 1);
      break;
    case 9 :
      lcd.setCursor(15, 1);
      break;
  }
  return 0;
}

// ========================================
// === Page d'accueil                   ===
// ========================================

byte uiPageMain(byte button, byte mode)
{
  if(mode==UI_MODE_LOOP) return 0;
  static byte mainOption = 6;
  lcd.noBlink();
  if (mode & UI_MODE_DRAW_ALL)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("DiCiCino multi"));
    lcd.setCursor(0, 1);
    lcd.write(0x7F);
    lcd.write(0x7E);
    dccClear();
  }

  switch (button)
  {
    case DUI_KEY_RIGHT :
      if (mainOption < UI_NUM_MENU_OPTIONS) mainOption++;
      else mainOption = 1;
      break;
    case DUI_KEY_LEFT :
      if (mainOption > 1) mainOption--;
      else mainOption = UI_NUM_MENU_OPTIONS;
      break;
    case DUI_KEY_OK :
      switch (mainOption)
      {
        case 1 : return UI_PAGE_PILOT_MAIN;
        case 2 : return UI_PAGE_PROG_MAIN;
        case 3 : return UI_PAGE_I2C_SCAN;
        case 4 : return UI_PAGE_I2C_TEST;
        case 5 : return UI_PAGE_I2C_ADDR;
        case 6 : return UI_PAGE_I2C_MONI;
        case 7 : return UI_PAGE_I2C_NAME;
        case 8 : return UI_PAGE_PCA9685_MAIN;
        case 9 : return UI_PAGE_I2C_AIEA;
        case 10 : return UI_PAGE_I2C_AISV;
      }
      break;
  }
  lcd.setCursor(2, 1);
  switch (mainOption)
  {
    case 1 : lcd.print(F("DCC Pilot     ")); break;
    case 2 : lcd.print(F("DCC Prog      ")); break;
    case 3 : lcd.print(F("I2C Scan      ")); break;
    case 4 : lcd.print(F("I2C Test      ")); break;
    case 5 : lcd.print(F("I2C Addr      ")); break;
    case 6 : lcd.print(F("I2C Dialog    ")); break;
    case 7 : lcd.print(F("I2C Id        ")); break;
    case 8 : lcd.print(F("TEST PCA9685  ")); break;
    case 9 : lcd.print(F("PROG Aig EA   ")); break;
    case 10 : lcd.print(F("PROG Aig SV   ")); break;

//    Les fonctions suivantes sont en cours de mise au point
//    -> https://codeinter-net.blogspot.com/


//    case 7 : lcd.print(F("TEST Aig elec")); break;
//    case 7 : lcd.print(F("TEST Aig servo")); break;
//    case 10: lcd.print(F("TEST Signaux  ")); break;
//    case 12: lcd.print(F("TEST Capteurs ")); break;
//    case 8 : lcd.print(F("PROG Aig elec ")); break;
//    case 9 : lcd.print(F("PROG Aig servo")); break;
//    case 11: lcd.print(F("PROG Signaux  ")); break;
//    case 13: lcd.print(F("PROG Capteurs ")); break;

  }
  return 0;
}

// ----------------------------------------
// --- Code principal                   ---
// ----------------------------------------

byte uiPage(byte button, byte mode)
// Aiguillage vers la bonne page de l'interface utilisateur
{
  switch (uiCurrentPage)
  {
    case UI_PAGE_MAIN : return uiPageMain(button, mode);
    case UI_PAGE_PILOT_MAIN : return uiPageMainPilot(button, mode);
    case UI_PAGE_PILOT : return uiPagePilot(button, mode);
    case UI_PAGE_PROG_MAIN : return uiPageMainProg(button, mode);
    case UI_PAGE_I2C_TEST : return uiPageMainI2CTest(button, mode);
    case UI_PAGE_I2C_SCAN : return uiPageMainI2CScan(button, mode);
    case UI_PAGE_I2C_ADDR : return uiPageMainI2CAddr(button, mode);
    case UI_PAGE_I2C_MONI : return uiPageMainI2CMoni(button, mode);
    case UI_PAGE_I2C_NAME : return uiPageMainI2CName(button, mode);
    case UI_PAGE_PCA9685_MAIN : return uiPageMainPCA9685(button, mode);
    case UI_PAGE_PCA9685 : return uiPagePCA9685(button, mode);
    case UI_PAGE_I2C_AIEA : return uiPageMainProgAIEA(button, mode);
    case UI_PAGE_I2C_AISV : return uiPageMainProgAISV(button, mode);
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Wire.begin(); 

  pinMode(DCC_E, OUTPUT);
  pinMode(DCC_S, OUTPUT);
  pinMode(DCC_C, OUTPUT);
  digitalWrite(DCC_E, LOW);
  digitalWrite(DCC_S, LOW);
  digitalWrite(DCC_C, LOW);

  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, LOW);
  delay(200);

  dccClear();

  FlexiTimer2::set(1, 0.000028, dccInterrupt);
  FlexiTimer2::start();
  
  lcd.begin(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  lcd.noCursor();
  lcd.noBlink();

  uiCurrentPage=UI_PAGE_MAIN;
  uiPage(0, UI_MODE_DRAW_ALL);
  uiPage(0, UI_MODE_SET_CURSOR);
  digitalWrite(LCD_BACKLIGHT, 1);
  digitalWrite(DCC_E, HIGH);
}

void loop()
{
  static byte count;
  byte key = readKeyboard();
  if (key)
  {
    byte ret = uiPage(key, 0);
    if (ret)
    {
      uiCurrentPage = ret; // Changement de page
      uiPage(0, UI_MODE_DRAW_ALL);
    }
    uiPage(0, UI_MODE_SET_CURSOR);
    while (readKeyboard());
  }
  uiPage(0, UI_MODE_LOOP);
  delay(100);

//  if(!((count++)&15)) // Retirer les commentaires pour activer l'affichage de la liste des paquets
//    dumpDccPackets();
}
