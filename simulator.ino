#include "compass.h"
#include "controls.h"
#include "eeprom_m.h"
#include "nav.h"
#include "propulsion.h"
#include "route.h"
#include "simulator.h"
#include "telemetry.h"

#ifndef REAL_MISSION
NeoGPS::Location_t lastSimulatedLocation;
NeoGPS::Location_t nextRouteWaypoint;

unsigned long lastSimulationMillis;
unsigned long lastSimulationStateSaveMillis;
float lastSimulatedTrueHeadingDegrees;
bool isSimulationOn;

int simulationIntervalSec = 30;

void initSimulator(int simulationRate) {
    Serial.println("Simulator:");    

    simulationIntervalSec = simulationIntervalSec / simulationRate;

    isSimulationOn = readInt16FromEeprom(EEPROM_SIMULATOR_FLAG_ADDR);
    lastSimulatedLocation.lat(readInt32FromEeprom(EEPROM_SIMULATOR_LAT_ADDR));
    lastSimulatedLocation.lon(readInt32FromEeprom(EEPROM_SIMULATOR_LON_ADDR));

    Serial.println(isSimulationOn);
    Serial.println(lastSimulatedLocation.lat());
    Serial.println(lastSimulatedLocation.lon());
}

NeoGPS::Location_t* generateSimulatedGpsLocation() {
    // addTelemetryDataInt32_t("\tSh", lastSimulatedTrueHeadingDegrees);

    // update simulated location every X seconds depending on simulation rate
    unsigned long consistentMillis = getConsistentMillis();
    if (consistentMillis - lastSimulationMillis < 1000*simulationIntervalSec) {
        return &lastSimulatedLocation;
    }

    lastSimulationMillis = consistentMillis;

    // 0.27 m/s = 1 kph. comes to 8 meters of every 30 seconds - at normal simulation rate.
    // add 8 meters in the direction of desired waypoint
    lastSimulatedTrueHeadingDegrees = generateSimulatedTrueHeadingDegrees();
    lastSimulatedLocation.OffsetBy((8/1000.0)/NeoGPS::Location_t::EARTH_RADIUS_KM,
        lastSimulatedTrueHeadingDegrees*PI/180.0);

    // save simulation position in EEPROM every 10 minutes tp preserve across restart
    if (consistentMillis - lastSimulationStateSaveMillis > 1000*60*10) {
        lastSimulationStateSaveMillis = consistentMillis;
        persistSimulationState();
    }

    return &lastSimulatedLocation;
}

float generateSimulatedTrueHeadingDegrees() {
    Waypoint wp = getCurrentTargetRouteWaypoint();
    nextRouteWaypoint.lat(wp.lat);
    nextRouteWaypoint.lon(wp.lon);

    // would be an ideal heading perfectly towards the next real waypoint
    lastSimulatedTrueHeadingDegrees = lastSimulatedLocation.BearingTo(nextRouteWaypoint)*180/PI;

    return lastSimulatedTrueHeadingDegrees;
}

int getLastSimulatedMagneticHeading() {
    return convertTrueHeadingToMagneticViaMagneticVariation(lastSimulatedTrueHeadingDegrees,
        (int) lastSimulatedLocation.latF(), (int) lastSimulatedLocation.lonF());
}

bool getSimulationOnOffState() {
    return isSimulationOn;
}

void resetSimulationState(bool onOff) {
    isSimulationOn = onOff;
    lastSimulationMillis = 0;

    resetWaypointIndex();
    Waypoint wp = getCurrentTargetRouteWaypoint();

    lastSimulatedLocation.lat(wp.lat);
    lastSimulatedLocation.lon(wp.lon);

    persistSimulationState();
}

void persistSimulationState() {
    writeInt16ToEeprom(EEPROM_SIMULATOR_FLAG_ADDR, isSimulationOn);
    writeInt32ToEeprom(EEPROM_SIMULATOR_LAT_ADDR, lastSimulatedLocation.lat());
    writeInt32ToEeprom(EEPROM_SIMULATOR_LON_ADDR, lastSimulatedLocation.lon());
}
#else
void initSimulator(int simulationRate) {
}
bool getSimulationOnOffState() {
    return false;
}
#endif