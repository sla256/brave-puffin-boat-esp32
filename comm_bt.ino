#include <Arduino.h>
#include "comm_bt.h"

#ifndef REAL_MISSION

BT_SERIAL_TYPE SerialBT;

BT_SERIAL_TYPE* getBtSerial() {
  return &SerialBT;
}

bool btOnOffState;

void initBt() {
  Serial.print("BT: ");
  Serial.println(SerialBT.begin("bt-esp32"));
  turnBtOn();
}

bool isBtOn() {
  return btOnOffState  &&  isBtConnected();
}

void turnBtOff() {
  btStop();
  btOnOffState = false;
}

void turnBtOn() {
  btStart();
  getBtSerial()->enableSSP();
  btOnOffState = true;
}

bool isBtConnected() {
  return getBtSerial()->connected()  &&  getBtSerial()->isReady();
}

void btPrintln(char *s) {
  if (!isBtOn()) {
    return;
  }
  getBtSerial()->println(s);
}

void btPrintln(int i) {
  if (!isBtOn()) {
    return;
  }
  getBtSerial()->println(i);
}

#else

BT_SERIAL_TYPE* getBtSerial() {
  return NULL;
}

void initBt() {
  btStop();
}

bool isBtOn() {
  return false;
}

bool isBtConnected() {
  return false;
}

void btPrintln(char *s) {
}

void btPrintln(int i) {
}

#endif
