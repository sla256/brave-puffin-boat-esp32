#include <Arduino.h>

#include "controls.h"
#include "debug.h"
#include "gps.h"
#include "pins.h"
#include "simulator.h"
#include "telemetry.h"
#include "test.h"

HardwareSerial gpsSerial(2);

NMEAGPS gps; // This parses the GPS characters
gps_fix lastGoodFix; // This holds on to the latest values
bool gpsAvailable;

unsigned long lastWorkCycleMillis = 0; // to remember the cycle when the last non gps work handler was invoked
const int workCycleDelayMs = 100; // desired delay between non nav work cycles

void (*nonGpsWorkHandlerPtr)(gps_fix *lastGoodFix);

void initGps(void (*workHandlerPtr)(gps_fix *lastGoodFix)) {
  Serial.println("GPS");
  gpsSerial.begin(9600, SERIAL_8N1, PIN_RX2, PIN_TX2);
  nonGpsWorkHandlerPtr = workHandlerPtr;
}

bool isGpsAvailable() {
  return gpsAvailable;
}

gps_fix *getLastGoodFix() {
  return &lastGoodFix;
}

void gpsLoop() {
  unsigned long currentMillis = getConsistentMillis();

  obtainFixFromActualGps();

#ifndef REAL_MISSION
  // keep most parts of the actual gps fix i.e. date time
  // but overwrite the location from the simulated one
  if (getSimulationOnOffState()) {
    NeoGPS::Location_t* simulatedLocation = generateSimulatedGpsLocation();
    lastGoodFix.location.lat(simulatedLocation->lat());
    lastGoodFix.location.lon(simulatedLocation->lon());
  }
#endif

  if (isWorkCycle(currentMillis)) {
    nonGpsWorkHandlerPtr(&lastGoodFix);
    lastWorkCycleMillis = currentMillis;
  }
}

void obtainFixFromActualGps() {
  gpsAvailable = gps.available(gpsPort);

  if (!gpsAvailable) return;

  gps_fix fix = gps.read();
  if (fix.valid.location) {
    lastGoodFix = fix;
  }
}

// determines if it's time to perform non GPS processing work
bool isWorkCycle(unsigned long currentMillis) {
  return lastWorkCycleMillis + workCycleDelayMs < currentMillis  ||  lastWorkCycleMillis > currentMillis;
}
