#include <Arduino.h>
#include <EEPROM.h>

#ifndef BOAT_EEPROM_H
#define BOAT_EEPROM_H

#define EEPROM_RUN_NUMBER_ADDR          0
#define EEPROM_PROPULSION_STATE_ADDR    2
#define EEPROM_WAYPOINT_INDEX_ADDR      4
#define EEPROM_LOG_FILE_NUMBER_ADDR     6
#define EEPROM_SIMULATOR_LAT_ADDR       8
#define EEPROM_SIMULATOR_LON_ADDR       12
#define EEPROM_SIMULATOR_FLAG_ADDR      16

void initEeprom();

int16_t readInt16FromEeprom(int addr);
void writeInt16ToEeprom(int addr, int16_t value);

int32_t readInt32FromEeprom(int addr);
void writeInt32ToEeprom(int addr, int32_t value);

#endif