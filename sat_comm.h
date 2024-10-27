#ifndef SAT_COMM
#define SAT_COMM

struct SatTrackingRecord {
  uint16_t runNumber;
  uint8_t runtimeHours;

  int16_t lastMagneticHeading;
  int16_t lastTrueHeading;
  int16_t bearing;

  uint8_t lastMagnetomerCalibrationState;

  uint8_t onboardTemperatureC;
  uint8_t onboardHumidity;
  uint16_t onboardPressureHectopascals;

  uint8_t mainUnderwaterBatteryTemperatureC;

  int32_t lat;
  int32_t lon;

  uint8_t waypointNumber;
  uint16_t distanceToNextWaypointKilometers;

  uint8_t batteryVoltageDecivolts;
  uint8_t batteryCurrentDrawDeciamps;

  uint8_t leftMotorCurrentDrawDeciamps;
  uint8_t rightMotorCurrentDrawDeciamps;

  uint8_t solarCurrentDeciamps;

  uint8_t cruisePulseWidthTens;
  uint8_t motorLeftPulseWidthTens;
  uint8_t motorRightPulseWidthTens;

  uint16_t totalSpentEnergyWattHours;
  uint16_t totalReceivedEnergyWattHours;
};

enum SatCommRunningCoreSelection {
  SatCommRunOnDefaultArduinoCore1,
  SatCommRunOnSpareCore0 // however... does not work when all of I2C devices are connected
};

void initSatComm(SatCommRunningCoreSelection coreSelector);

// call directly if running on default Core 1; otherwise will be a no-op - sat comm processing will be done in another loop on Core 0
void handleSatComm();

void satCommSetTrackingRecordForNextCommSession(SatTrackingRecord*);
int satCommGetLastSignalQuality();
int satCommGetCurrentState();
int satCommGetLastError();
void startSatCommSession();

#endif