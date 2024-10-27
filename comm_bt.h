#include "controls.h"

#ifndef REAL_MISSION

#include <BluetoothSerial.h>
#define BT_SERIAL_TYPE BluetoothSerial

#else 

#define BT_SERIAL_TYPE HardwareSerial

#endif

BT_SERIAL_TYPE* getBtSerial();
void initBt();
bool isBtOn();
void turnBtOff();
void turnBtOn();
bool isBtConnected();
void btPrintln(char *s);
void btPrintln(int i);