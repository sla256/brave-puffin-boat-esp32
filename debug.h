// #define BOAT_DEBUG_ON
#include "display.h"


#ifdef BOAT_DEBUG_ON
// #define BOAT_DEBUG(x) displayDebugSymbol(x);
#define BOAT_DEBUG(x) Serial.print(x);
#else
#define BOAT_DEBUG(x)
#endif

void addTelemetryForSerialOutput(TelemetryRecord* telemetryRecord);