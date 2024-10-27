#define USE_DISPLAY

#ifdef USE_DISPLAY

#include <U8x8lib.h>
#include <U8g2lib.h>

#endif

#include "telemetry.h"

void initDisplay();
void turnDisplayOn();
void turnDisplayOff();
void showInfoOnDisplay(TelemetryRecord* telemetryRecord);
void displayDebugSymbol(char* s);

