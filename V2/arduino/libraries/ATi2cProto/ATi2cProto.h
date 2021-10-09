// AT-I2C-PROTO
// PBA 2021
// Librairie d'interface pour les périphériques I2C

#ifndef ATi2cProto_h
#define ATi2cProto_h

class ATi2cProto
{
public:
ATi2cProto(char id[4],byte defAddr,void (*init)(),void(*rcv)(byte* data, byte length), byte(*req)(byte* data, byte length, byte* out));
byte getAddr(void);
void dump(void);

private:
  byte id[4];
  byte data[32];
  byte length;
  word command;
	byte addr;
  void ATi2cProto::receive(void);
  void ATi2cProto::request(void);
  void(*atReceive)(byte* data, byte length);
  byte(*atRequest)(byte* data, byte length, byte* out);
  static void ATi2cProto::receiveStatic(void);
  static void ATi2cProto::requestStatic(void);
};

#endif
