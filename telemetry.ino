#include <Arduino.h>

#include "comm_bt.h"
#include "controls.h"
#include "debug.h"
#include "eeprom_m.h"
#include "gps.h"
#include "pins.h"
#include "telemetry.h"
#include "telemetry_record.h"
#include "sat_comm.h"
#include "sd_m.h"

unsigned long lastSaveTelemetryMillis;
unsigned long currentTelemetryMillis;

unsigned long consistentMillis;

TelemetryRecord telemetryRecord;
SatTrackingRecord satTrackingRecord;

#ifdef REAL_MISSION
#define TELEMETRY_SAVE_DELAY_MS 60000
#else
#define TELEMETRY_SAVE_DELAY_MS 10000
#endif

int16_t runNumber;

void initTelemetry() {
  pinMode(PIN_STATUS_LED, OUTPUT);

  runNumber = readInt16FromEeprom(EEPROM_RUN_NUMBER_ADDR) + 1;
  writeInt16ToEeprom(EEPROM_RUN_NUMBER_ADDR, runNumber);

  Serial.print("Run: ");
  Serial.println(runNumber);
}

void handleTelemetry() {
  fillInTelemetryRecord(&telemetryRecord, &satTrackingRecord);
  setDataForNextCommSession();
  satCommSetTrackingRecordForNextCommSession(&satTrackingRecord);
}

int16_t currentRunNumber() {
  return runNumber;
}

unsigned long getConsistentMillis() {
  return consistentMillis;
}

void setConsistentMillis(unsigned long currentMillis) {
  consistentMillis = currentMillis;
}

unsigned long getLastSTelemetryMillis() {
  return lastSaveTelemetryMillis;
}

void startTelemetryRecord() {
  currentTelemetryMillis = consistentMillis;

  if (isTimeToSaveTelemetry()) {
    digitalWrite(PIN_STATUS_LED, HIGH);
  }

  addTelemetryDataInt32_t("\tG", isGpsAvailable() ? 1 : 0);
}

void addTelemetryDataFloat(const char *prefix, float value) {
  if (!isTimeToSaveTelemetry()) return;
  
  if (prefix) { 
    if (isBtOn()) getBtSerial()->print(prefix);
    Serial.print(prefix);
  }
  Serial.print(value, 5);  
  if (isBtOn()) getBtSerial()->print(value, 5);  
}

void addTelemetryDataInt32_t(const char *prefix, int32_t value) {
  if (!isTimeToSaveTelemetry()) return;
  
  if (prefix) {
    if (isBtOn()) getBtSerial()->print(prefix);
    Serial.print(prefix);
  }
  Serial.print(value);  
  if (isBtOn()) getBtSerial()->print(value);  
}

void addTelemetryDataString(const char *prefix, const char *value) {
  if (!isTimeToSaveTelemetry()) return;

  if (prefix) {
    if (isBtOn()) getBtSerial()->print(prefix);
    Serial.print(prefix);
  }
  Serial.print(value);  
  if (isBtOn()) getBtSerial()->print(value);  
}

void saveTelemetry() {
  if (!isTimeToSaveTelemetry()) return;

  Serial.println();
  Serial.flush();

  if (isBtOn()) getBtSerial()->println();
  if (isBtOn()) getBtSerial()->flush();

  saveLogRecordToSdCard(getLastTelemetryDataInCsv());

  lastSaveTelemetryMillis = currentTelemetryMillis;

  digitalWrite(PIN_STATUS_LED, LOW);
}

bool isTimeToSaveTelemetry() {
  bool isItTime = currentTelemetryMillis - lastSaveTelemetryMillis > TELEMETRY_SAVE_DELAY_MS
    ||  currentTelemetryMillis <= lastSaveTelemetryMillis;
  
  return isItTime;
}

const int commDataBufferJsonLen =
1+    // first line curly brace
8+5+  // Rn
8+10+ // millis
8+3+  // M(agnetic heading)
8+3+  // T(rue heading)
8+5+  // Vb battery voltage
8+3+  // To temp onboard
8+3+  // Ho humidiy onboard
8+3+  // Po pressure onboard
8+3+  // Te temp external
8+3+  // He
8+3+  // Pe
8+10+ // La
8+10+ // Lo
8+10+ // Dw
8+5+  // B bearing (desired heading)
8+5+  // Wn waypoint number
8+3+  // Cm
8+5+  // Ib
8+5+  // Is
8+5+  // Pc (pulse width cruise)
8+5+  // Pl (pulse width left)
8+5+  // Pr (pulse width right)
8+5+  // Pi(tch)
8+5+  // Ro(ll)
8+5+  // Sq (sat comm quality signal)
8+5+  // Ss (sat comm state)
1;    // last curly

// enough padding to not needing +1 for zero termination
char commDataBufferJson[commDataBufferJsonLen];
char commDataBufferCsv[commDataBufferJsonLen];

void setDataForNextCommSession() {
  TelemetryRecord* record = &telemetryRecord;
  snprintfTelemetryRecord(commDataBufferJson, commDataBufferJsonLen, "{\
\"Rn\":\"%d\",\
\"Ms\":\"%lu\",\
\"M\":\"%d\",\
\"T\":\"%d\",\
\"Vb\":\"%d\",\
\"To\":\"%d\",\
\"Ho\":\"%d\",\
\"Po\":\"%d\",\
\"Te\":\"%d\",\
\"He\":\"%d\",\
\"Pe\":\"%d\",\
\"La\":\"%ld\",\
\"Lo\":\"%ld\",\
\"Dw\":\"%ld\",\
\"B\":\"%d\",\
\"Wn\":\"%d\",\
\"Cm\":\"%d\",\
\"Ib\":\"%d\",\
\"Is\":\"%d\",\
\"Pb\":\"%d\",\
\"Pu\":\"%d\",\
\"Es\":\"%d\",\
\"Er\":\"%d\",\
\"Pc\":\"%d\",\
\"Pl\":\"%d\",\
\"Pr\":\"%d\",\
\"Pi\":\"%d\",\
\"Ro\":\"%d\",\
\"Sq\":\"%d\",\
\"Ss\":\"%d\",\
\"Se\":\"%d\"\
}", record);
  
  snprintfTelemetryRecord(commDataBufferCsv, commDataBufferJsonLen,
"%d,\
%lu,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%ld,\
%ld,\
%ld,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,\
%d,", record);
}

void snprintfTelemetryRecord(char* buffer, size_t bufferMaxLen, const char* format, TelemetryRecord* record) {
  snprintf(buffer, bufferMaxLen, format,
    record->runNumber, record->ms, (int)(record->lastMagneticHeading),
    (int)(record->lastTrueHeading), record->batteryVoltageMv,
    (int) record->onboardTemperatureC, (int) record->onboardHumidity, (int) record->onboardPressurePa,  
    (int) record->externalTemperatureC, (int) record->externalHumidity, (int) record->externalPressurePa,  
    record->lat, record->lon, record->distanceToNextWaypointMeters,
    (int)(record->bearing), record->waypointNumber, record->lastMagnetomerCalibrationState,
    record->batteryCurrentDrawMa, record->solarCurrentMa,
    record->optimalPowerBudgetTargetW, record->currentlyUsedPowerW,
    record->totalSpentEnergyMilliwattHours, record->totalReceivedEnergyMilliwattHours,
    record->cruisePulseWidth, record->motorLeftPulseWidth, record->motorRightPulseWidth,
    record->pitch, record->roll,
    record->satCommLastSignalQuality, record->satCommCurrentState, record->satCommLastError);
}

char* getLastTelemetryDataInJson() {
  return commDataBufferJson;
}

char* getLastTelemetryDataInCsv() {
  return commDataBufferCsv;
}

TelemetryRecord* getCurrentTelemetryRecord() {
  return &telemetryRecord;
}