#include "controls.h"
#include "debug.h"
#include "simulator.h"
#include "telemetry.h"
#include "test.h"

void addTelemetryForSerialOutput(TelemetryRecord* telemetryRecord) {
    if (!isTimeToSaveTelemetry()) return;

    addTelemetryDataInt32_t("\tS", telemetryRecord->ms / 1000);
    addTelemetryDataInt32_t("\tRn", telemetryRecord->runNumber);

    addTelemetryDataInt32_t("\t", telemetryRecord->year);
    addTelemetryDataInt32_t("-", telemetryRecord->month);
    addTelemetryDataInt32_t("-", telemetryRecord->day);
    addTelemetryDataInt32_t(" ", telemetryRecord->hours);
    addTelemetryDataInt32_t(":", telemetryRecord->minutes);
    addTelemetryDataInt32_t(":", telemetryRecord->seconds);

    addTelemetryDataInt32_t("\tP", getPilotMode());

    addTelemetryDataInt32_t("\tPc", telemetryRecord->cruisePulseWidth);
    addTelemetryDataString("", telemetryRecord->isPropulsionOn ? "+" : "-");

    addTelemetryDataInt32_t("\tPl", telemetryRecord->motorLeftPulseWidth);
    addTelemetryDataInt32_t("\tPr", telemetryRecord->motorRightPulseWidth);

    addTelemetryDataFloat("\tL", telemetryRecord->lat / 10000000.0);
    addTelemetryDataFloat(",", telemetryRecord->lon / 10000000.0);
    addTelemetryDataString("", getSimulationOnOffState() ? "s" : "r");

    addTelemetryDataInt32_t("\tCm", telemetryRecord->lastMagnetomerCalibrationState);
    addTelemetryDataInt32_t("\tCa", telemetryRecord->lastAccelerometerCalibrationState);
    addTelemetryDataInt32_t("\tCx", telemetryRecord->maxCountOfReadingsWithBadCompassCalibration);
    
    addTelemetryDataInt32_t("\tM", telemetryRecord->lastMagneticHeading);
    addTelemetryDataInt32_t("\tT", telemetryRecord->lastTrueHeading);
    addTelemetryDataInt32_t("\tB", telemetryRecord->bearing);

    addTelemetryDataInt32_t("\tIb", telemetryRecord->batteryCurrentDrawMa);
    addTelemetryDataInt32_t("\tVb", telemetryRecord->batteryVoltageMv);
    addTelemetryDataInt32_t("\tPb", telemetryRecord->optimalPowerBudgetTargetW);
    addTelemetryDataInt32_t("\tPu", telemetryRecord->currentlyUsedPowerW);

    addTelemetryDataInt32_t("\tIs", telemetryRecord->solarCurrentMa);
    addTelemetryDataInt32_t("\tVs", telemetryRecord->solarVoltageMv);
    addTelemetryDataInt32_t("\tIl", telemetryRecord->leftMotorCurrentDrawMilliamps);
    addTelemetryDataInt32_t("\tIr", telemetryRecord->rightMotorCurrentDrawMilliamps);

    addTelemetryDataInt32_t("\tEs", telemetryRecord->totalSpentEnergyMilliwattHours);
    addTelemetryDataInt32_t("\tEr", telemetryRecord->totalReceivedEnergyMilliwattHours);

    // addTelemetryDataInt32_t("\tPi", telemetryRecord->pitch);
    // addTelemetryDataInt32_t("\tRo", telemetryRecord->roll);

    addTelemetryDataInt32_t("\tTb", telemetryRecord->mainUnderwaterBatteryTemperatureC);
    addTelemetryDataInt32_t("\tTo", telemetryRecord->onboardTemperatureC);
    addTelemetryDataInt32_t("\tHo", telemetryRecord->onboardHumidity);
    addTelemetryDataInt32_t("\tPo", telemetryRecord->onboardPressurePa);

    addTelemetryDataInt32_t("\tRf", telemetryRecord->memHeapFree);

    addTelemetryDataInt32_t("\tSq", telemetryRecord->satCommLastSignalQuality);
    addTelemetryDataInt32_t("\tSs", telemetryRecord->satCommCurrentState);
    addTelemetryDataInt32_t("\tSe", telemetryRecord->satCommLastError);
}