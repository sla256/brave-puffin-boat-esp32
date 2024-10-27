#include "compass.h"
#include "controls.h"
#include "energy.h"
#include "gps.h"
#include "nav.h"
#include "pilot.h"
#include "power.h"
#include "power_budget.h"
#include "propulsion.h"
#include "sat_comm.h"
#include "sensors.h"
#include "telemetry.h"

void fillInTelemetryRecord(TelemetryRecord* telemetryRecord, SatTrackingRecord* satTrackingRecord) {
  
  if (!isTimeToSaveTelemetry()) return;

  satTrackingRecord->lastTrueHeading = telemetryRecord->lastTrueHeading = getCachedLastTrueHeading();
  satTrackingRecord->lastMagneticHeading = telemetryRecord->lastMagneticHeading = getCachedLastMagneticHeading();

  telemetryRecord->batteryVoltageMv = getLastBatteryVoltageMv();
  satTrackingRecord->batteryVoltageDecivolts = telemetryRecord->batteryVoltageMv / 100;

  telemetryRecord->batteryCurrentDrawMa = getLastBatteryDrawCurrentMa();
  satTrackingRecord->batteryCurrentDrawDeciamps = telemetryRecord->batteryCurrentDrawMa / 100;

  telemetryRecord->solarVoltageMv = getLastSolarChargingVoltageMv();
  telemetryRecord->solarCurrentMa = getLastSolarChargingCurrentMa();
  satTrackingRecord->solarCurrentDeciamps = telemetryRecord->solarCurrentMa / 100;
  
  telemetryRecord->totalSpentEnergyMilliwattHours = getTotalSpentEnergyMilliwattHours();
  satTrackingRecord->totalSpentEnergyWattHours = telemetryRecord->totalSpentEnergyMilliwattHours / 1000;

  telemetryRecord->totalReceivedEnergyMilliwattHours = getTotalReceivedEnergyMilliwattHours();
  satTrackingRecord->totalReceivedEnergyWattHours = telemetryRecord->totalReceivedEnergyMilliwattHours / 1000;

  telemetryRecord->optimalPowerBudgetTargetW = getOptimalPowerBudgetTargetW();
  telemetryRecord->currentlyUsedPowerW = getLastBatterDrawPowerW();

  telemetryRecord->leftMotorCurrentDrawMilliamps = getCurrentFromIna219(LeftMotor);
  satTrackingRecord->leftMotorCurrentDrawDeciamps = telemetryRecord->leftMotorCurrentDrawMilliamps / 100;

  telemetryRecord->rightMotorCurrentDrawMilliamps = getCurrentFromIna219(RightMotor);
  satTrackingRecord->rightMotorCurrentDrawDeciamps = telemetryRecord->rightMotorCurrentDrawMilliamps / 100;

  telemetryRecord->ms = getConsistentMillis();
  satTrackingRecord->runtimeHours = telemetryRecord->ms / 1000 / 3600;

  satTrackingRecord->runNumber = telemetryRecord->runNumber = currentRunNumber();
  satTrackingRecord->waypointNumber = telemetryRecord->waypointNumber = getCurrentWaypointIndex();
  satTrackingRecord->distanceToNextWaypointKilometers = telemetryRecord->distanceToNextWaypointMeters = getDistanceToNextRealWaypointMeters();

  gps_fix* lastGoodFix = getLastGoodFix();
  satTrackingRecord->lat = telemetryRecord->lat = lastGoodFix->location.lat();
  satTrackingRecord->lon = telemetryRecord->lon = lastGoodFix->location.lon();

  satTrackingRecord->bearing = telemetryRecord->bearing  = getLastAutonomousBearingDegrees();

  telemetryRecord->year = lastGoodFix->dateTime.year;
  telemetryRecord->month = lastGoodFix->dateTime.month;
  telemetryRecord->day = lastGoodFix->dateTime.date;
  telemetryRecord->hours = lastGoodFix->dateTime.hours;
  telemetryRecord->minutes = lastGoodFix->dateTime.minutes;
  telemetryRecord->seconds = lastGoodFix->dateTime.seconds;

  PropulsionState* propulsionState = getPropulsionState();
  telemetryRecord->isPropulsionOn = isPropulsionOn();

  telemetryRecord->cruisePulseWidth = propulsionState->cruisePulseWidth;
  satTrackingRecord->cruisePulseWidthTens = telemetryRecord->cruisePulseWidth / 10;

  telemetryRecord->motorLeftPulseWidth = propulsionState->motorLeftPulseWidth;
  satTrackingRecord->motorLeftPulseWidthTens = telemetryRecord->motorLeftPulseWidth / 10;

  telemetryRecord->motorRightPulseWidth = propulsionState->motorRightPulseWidth;
  satTrackingRecord->motorRightPulseWidthTens = telemetryRecord->motorRightPulseWidth / 10;

  satTrackingRecord->onboardTemperatureC = telemetryRecord->onboardTemperatureC = getTemperatureFromBme280(OnboardBme280);
  satTrackingRecord->onboardHumidity = telemetryRecord->onboardHumidity = getHumidityFromBme280(OnboardBme280);
  telemetryRecord->onboardPressurePa = getPressureFromBme280(OnboardBme280);
  satTrackingRecord->onboardPressureHectopascals = telemetryRecord->onboardPressurePa / 100;

  satTrackingRecord->mainUnderwaterBatteryTemperatureC = telemetryRecord->mainUnderwaterBatteryTemperatureC = getTemperatureFromTmp36(MainUnderwaterBattery);

  telemetryRecord->memHeapFree = ESP.getFreeHeap();
  telemetryRecord->memHeapSize = ESP.getHeapSize();
  telemetryRecord->memMinFreeHeap = ESP.getMinFreeHeap();
  telemetryRecord->memMaxAllocHeap = ESP.getMaxAllocHeap();

  telemetryRecord->pilotMode = getPilotMode();

  telemetryRecord->pitch = getLastPitch();
  telemetryRecord->roll = getLastRoll();

  satTrackingRecord->lastMagnetomerCalibrationState = telemetryRecord->lastMagnetomerCalibrationState = getLastMagnetomerCalibrationState();
  telemetryRecord->lastAccelerometerCalibrationState = getLastAccelerometerCalibrationState();
  telemetryRecord->maxCountOfReadingsWithBadCompassCalibration = getMaxCountOfReadingsWithBadCompassCalibration();

  telemetryRecord->satCommLastSignalQuality = satCommGetLastSignalQuality();
  telemetryRecord->satCommCurrentState = satCommGetCurrentState();
  telemetryRecord->satCommLastError = satCommGetLastError();
}