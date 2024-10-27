#include <math.h>
#include "energy.h"
#include "power.h"
#include "telemetry.h"

struct EnergyTrackingStorage {
    unsigned long lastCalculationTimeMillis = 0;
    float cumulativeEnergyForThisTrackingPeriodWattHours = 0;  
};

EnergyTrackingStorage totalSpentEnergySinceStart;
EnergyTrackingStorage totalReceivedEnergySinceStart;

void initEnergyTracking() {
}

void handleEnergyTracking() {
    unsigned long currentMillis = millis(); // don't want consistent millis as everywhere else, want actual clock value for better accuracy (?)

    calculateEnergy(&totalSpentEnergySinceStart, currentMillis, getLastBatteryVoltageMv(), getLastBatteryDrawCurrentMa());
    calculateEnergy(&totalReceivedEnergySinceStart, currentMillis, getLastSolarChargingVoltageMv(), getLastSolarChargingCurrentMa());
}

void calculateEnergy(struct EnergyTrackingStorage* trackingStorage, unsigned long currentMillis, int voltageMv, int currentMa) {
    currentMa = max(0, currentMa); // at night solar current can be reported negative; up to a zero

    // if lastCalculationTimeMillis > currentMillis, the clock must have rolled over to zero, let's correct
    // with some not super accurate but positive value, part of the actual elapsed
    int elapsedTimeSinceLastCalculationMs = trackingStorage->lastCalculationTimeMillis > currentMillis ?
        currentMillis : currentMillis - trackingStorage->lastCalculationTimeMillis;

    float currentPowerWatt = (voltageMv / 1000.0) * (currentMa / 1000.0);
    float energySinceLastCalculationWattSeconds = currentPowerWatt * elapsedTimeSinceLastCalculationMs / 1000;

    trackingStorage->cumulativeEnergyForThisTrackingPeriodWattHours += energySinceLastCalculationWattSeconds / 3600;
    trackingStorage->lastCalculationTimeMillis = currentMillis;
}

unsigned long getTotalSpentEnergyMilliwattHours() {
    return totalSpentEnergySinceStart.cumulativeEnergyForThisTrackingPeriodWattHours * 1000;
}

unsigned long getTotalReceivedEnergyMilliwattHours() {
    return totalReceivedEnergySinceStart.cumulativeEnergyForThisTrackingPeriodWattHours * 1000;
}