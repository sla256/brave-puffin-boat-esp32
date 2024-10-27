#include <esp_task_wdt.h>
#include <NMEAGPS.h>
#include "compass.h"
#include "comm_bt.h"
#include "comm_wifi.h"
#include "controls.h"
#include "debug.h"
#include "eeprom_m.h"
#include "energy.h"
#include "display.h"
#include "gps.h"
#include "heartbeat.h"
#include "ina219.h"
#include "nav.h"
#include "pilot.h"
#include "power.h"
#include "power_budget.h"
#include "propulsion.h"
#include "rc.h"
#include "route.h"
#include "sat_comm.h"
#include "sd_m.h"
#include "sensors.h"
#include "simulator.h"
#include "telemetry.h"
#include "watchdog.h"

void setup()
{
	Serial.begin(115200);
  Serial.println("Starting...");

  initPropulsion();
  initEeprom();
  initCompass();
  initControls();
  initHeartbeat();
  initRc();
  initDisplay();
  initBt();
  initWifi();
  initTelemetry();
  initGps(&nonNavWorkHandler);
  initSd();
  initSensors();
  initRoute();
  initNav();
  initSatComm(SatCommRunOnDefaultArduinoCore1);
  initSimulator(1);
  initEnergyTracking();
  initRestarter(4*24*60*60); // auto reset every 4 days; this is if the external watchdog/resetter won't do it in 3 days for some reason

  delay(1000); // allow motor ESC to init fully
}

// the main loop's priority is to process the GPS signal. then, every 100ms, 
// nonNavWorkHandler is called from within gpsLoop to perform all other tasks.
void loop() {
  setConsistentMillis(millis());
  gpsLoop();
}

void nonNavWorkHandler(gps_fix *lastGoodFix) {
  handleForcedRestart();

  startTelemetryRecord();

  int lastTrueHeading = convertMagneticHeadingToTrueViaMagneticVariation(getCompassMagneticHeading(), 
    (int) lastGoodFix->location.latF(), (int) lastGoodFix->location.lonF());
  cacheLastTrueHeading(lastTrueHeading);
  
  handlePower();
  handleEnergyTracking(); // should be called after handlePower()
  handlePowerBudget();
  handleControls();
  handleRc();
  handlePropulsion(lastGoodFix);
  handleHeartbeat();
  handleTelemetry();
  handleWifiComm(getConsistentMillis());
  handleSatComm();
  showInfoOnDisplay(getCurrentTelemetryRecord());
  addTelemetryForSerialOutput(getCurrentTelemetryRecord());
  saveTelemetry();
}