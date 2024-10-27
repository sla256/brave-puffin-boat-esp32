#include "eeprom_m.h"

void initEeprom() {
  Serial.print("EEPROM: ");
  Serial.println(EEPROM.begin(256));
}

int16_t readInt16FromEeprom(int addr) {
  return EEPROM.read(addr) | (EEPROM.read(addr+1) << 8);
}

void writeInt16ToEeprom(int addr, int16_t value) {
  EEPROM.write(addr, value & 0xFF);
  EEPROM.write(addr+1, value >> 8);
  EEPROM.commit();
}

int32_t readInt32FromEeprom(int addr) {
  int32_t lowInt = readInt16FromEeprom(addr) & 0xFFFF;
  int32_t highInt = readInt16FromEeprom(addr+2) << 16;

  return lowInt | highInt;
}

void writeInt32ToEeprom(int addr, int32_t value) {
  writeInt16ToEeprom(addr, value & 0xFFFF);
  writeInt16ToEeprom(addr+2, value >> 16);
}
