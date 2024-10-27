#define U8X8_USE_PINS

#include "debug.h"
#include "display.h"
#include "telemetry.h"

#ifdef USE_DISPLAY

U8G2_SSD1306_128X64_NONAME_F_SW_I2C *u8g2 = NULL;

bool isDisplayOn;

void initDisplay() {
  resetDisplayObject();
}

void showInfoOnDisplay(TelemetryRecord* telemetryRecord) {
  if (!isDisplayOn) {
    return;
  }

  u8g2->clearBuffer();
  
  u8g2->setCursor(15, 10); u8g2->print(telemetryRecord->ms / 1000);
  u8g2->setCursor(60, 10); u8g2->print("R:"); u8g2->print(telemetryRecord->runNumber);
  u8g2->setCursor(90, 10); u8g2->print(telemetryRecord->pilotMode);

  u8g2->setCursor(0, 20); u8g2->print(telemetryRecord->lat / 10000000.0, 5);
  u8g2->setCursor(60, 20); u8g2->print(telemetryRecord->lon / 10000000.0, 5);

  // u8g2->setCursor(0, 30); u8g2->print("MHz:"); u8g2->print(ESP.getCpuFreqMHz());
  u8g2->setCursor(0, 30); u8g2->print("Tb:"); u8g2->print(telemetryRecord->mainUnderwaterBatteryTemperatureC);
  u8g2->setCursor(60, 30); u8g2->print("mem:"); u8g2->print(telemetryRecord->memHeapFree / 1024);

  u8g2->setCursor(0, 40); u8g2->print("M:"); u8g2->print(telemetryRecord->lastMagneticHeading);
  u8g2->setCursor(60, 40); u8g2->print(telemetryRecord->throttleInput);
  u8g2->setCursor(90, 40); u8g2->print(telemetryRecord->steeringInput);

  u8g2->setCursor(0, 50); u8g2->print("ML:"); u8g2->print(telemetryRecord->motorLeftPulseWidth);
  u8g2->setCursor(60, 50); u8g2->print("MR:"); u8g2->print(telemetryRecord->motorRightPulseWidth);
  u8g2->print(telemetryRecord->isPropulsionOn ? "+" : "-");

  u8g2->setCursor(0, 60); u8g2->print("Ib:"); u8g2->print(telemetryRecord->batteryCurrentDrawMa);
  u8g2->setCursor(60, 60); u8g2->print("Vb:"); u8g2->print(telemetryRecord->batteryVoltageMv);

  u8g2->setCursor(0, 70); u8g2->print("Ib:"); u8g2->print(telemetryRecord->batteryCurrentDrawMa);

  u8g2->sendBuffer();
}

void turnDisplayOn() {
  if (isDisplayOn) return;

  u8g2->setPowerSave(false);
  isDisplayOn = true;
}

void turnDisplayOff() {
  if (!isDisplayOn) return;

  resetDisplayObject();
  u8g2->setPowerSave(true);
  isDisplayOn = false;
}

void displayDebugSymbol(char* s) {
  if (!isDisplayOn) return;

  u8g2->setCursor(0, 10); u8g2->print(s);
  u8g2->sendBuffer();
}

void resetDisplayObject() {
  if (u8g2) {
    delete u8g2;
    u8g2 = NULL;
  }

  u8g2 = new U8G2_SSD1306_128X64_NONAME_F_SW_I2C(U8G2_R0, /* SCL pin*/ 15, /* SDA pin*/ 4, /* reset=*/ 16);

  u8g2->begin();
  u8g2->enableUTF8Print();
  u8g2->setFont(u8g2_font_5x8_tf); //u8g2_font_t0_11_tf);
  u8g2->setFontDirection(0);
}

#else

void initDisplay() {
}

void showInfoOnDisplay(TelemetryRecord* telemetryRecord) {
}

void turnDisplayOn() {
}

void turnDisplayOff() {
}

#endif