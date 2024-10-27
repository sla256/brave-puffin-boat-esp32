#include <Arduino.h>

#include "controls.h"
#include "debug.h"
#include "nav.h"
#include "pins.h"
#include "power.h"
#include "power_budget.h"
#include "propulsion.h"
#include "sensors.h"

// dynamically adjust propulsion baseline PWM no more frequently that this, for smoother motor & prop loads
const int powerAdjustmentCycleMs = 20*1000; 
unsigned long lastPowerAdjustmentMillis = 0;

 // will tell propulsion to adjust baseline PWM in these increments, every powerAdjustmentCycleMs seconds
const int pulseWidthAdjustmentIncrement = 1;

int batteryVoltageMv;
int batteryDrawCurrentMa;
int solarChargingCurrentMa;
int solarChargingVoltageMv;

void handlePower() {
    batteryDrawCurrentMa = getCurrentFromIna226(BatteryIna226);
    batteryVoltageMv = getVoltageFromIna226(BatteryIna226);
    solarChargingCurrentMa = getCurrentFromIna226(SolarIna226);
    solarChargingVoltageMv = getVoltageFromIna226(SolarIna226);

    // don't adjust power too often; account for potentiall millis rollover
    int currentMillis = getConsistentMillis();
    if (lastPowerAdjustmentMillis - currentMillis < powerAdjustmentCycleMs  &&  currentMillis > lastPowerAdjustmentMillis) {
        return;
    }

    // don't adjust power dynamically in these modes
    PilotMode pilotMode = getPilotMode();
    if (pilotMode == FixedPowerNoSteering  ||  pilotMode == RemoteControlled  ||  !isPropulsionOn()) {
        return;
    }

    if (lastPowerAdjustmentMillis - currentMillis >= powerAdjustmentCycleMs  ||  currentMillis < lastPowerAdjustmentMillis) {
        lastPowerAdjustmentMillis = currentMillis;
    }

    lastPowerAdjustmentMillis = currentMillis;

    int currentlyUsedPowerW = getLastBatterDrawPowerW();
    int desiredPowerTargetW = getOptimalPowerBudgetTargetW(); // this is based on last millis cycle - OK

    // a really simple, linear, incremental adjustments. no PID (yet) - will likely constantly over-correct?
    // v1 approach. might be OK as the rate of adjustment is slow, power measurement is noisy, solar env. changes
    adjustPropulsionDesiredCruisePower(currentlyUsedPowerW > desiredPowerTargetW ? 
        -pulseWidthAdjustmentIncrement : pulseWidthAdjustmentIncrement);
}

int getLastBatteryVoltageMv() {
    return batteryVoltageMv;
}

int getLastBatteryDrawCurrentMa() {
    return batteryDrawCurrentMa;
}

int getLastBatterDrawPowerW() {
    return batteryVoltageMv * batteryDrawCurrentMa / 1000000;
}

int getLastSolarChargingCurrentMa() {
    return solarChargingCurrentMa;
}

int getLastSolarChargingVoltageMv() {
    return solarChargingVoltageMv;
}
