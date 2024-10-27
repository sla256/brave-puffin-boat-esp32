#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "sat_comm.h"

struct TelemetryRecord {
  int16_t runNumber;
  unsigned long ms;

  int16_t lastMagneticHeading;
  int16_t lastTrueHeading;
  int16_t bearing;

  uint8_t lastMagnetomerCalibrationState;
  uint8_t lastAccelerometerCalibrationState;
  int maxCountOfReadingsWithBadCompassCalibration;

  float onboardTemperatureC;
  float onboardHumidity;
  float onboardPressurePa;

  float externalTemperatureC;
  float externalHumidity;
  float externalPressurePa;

  float mainUnderwaterBatteryTemperatureC;

  uint8_t seconds;    //!< 00-59
  uint8_t minutes;    //!< 00-59
  uint8_t hours;      //!< 00-23
  uint8_t day;       //!< 01-31 Day of Month
  uint8_t month;      //!< 01-12
  uint8_t year;       //!< 00-99

  int32_t lat;
  int32_t lon;

  // int32_t targetLat;
  // int32_t targetLon;
  unsigned long distanceToNextWaypointMeters;
  int16_t waypointNumber;

  int16_t batteryVoltageMv;
  int16_t batteryCurrentDrawMa;

  int16_t leftMotorCurrentDrawMilliamps;
  int16_t rightMotorCurrentDrawMilliamps;

  int16_t optimalPowerBudgetTargetW;
  int16_t currentlyUsedPowerW;

  int16_t solarCurrentMa;
  int16_t solarVoltageMv;
  
  int32_t totalSpentEnergyMilliwattHours;
  int32_t totalReceivedEnergyMilliwattHours;

  int16_t cruisePulseWidth;
  int16_t motorLeftPulseWidth;
  int16_t motorRightPulseWidth;
  bool isPropulsionOn;

  unsigned long memHeapFree;
  unsigned long memHeapSize;
  unsigned long memMinFreeHeap;
  unsigned long memMaxAllocHeap;

  int pilotMode;
  int steeringInput;
  int throttleInput;

  int pitch;
  int roll;

  int satCommLastSignalQuality;
  int satCommCurrentState;
  int satCommLastError;
};

void fillInTelemetryRecord(TelemetryRecord* telemetryRecord, SatTrackingRecord* satTrackingRecord);

#endif