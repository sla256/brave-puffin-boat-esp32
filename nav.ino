
#include <Arduino.h>

#include "controls.h"
#include "debug.h"
#include "eeprom_m.h"
#include "nav.h"
#include "route.h"
#include "telemetry.h"
#include "test.h"

#define CLOSE_TO_WAYPOINT_DISTANCE_METERS   3000 // defines proximity to waypoints where power budget will be more generous

int targetWaypointIndex = 0;
bool isNextWaypointReal = true;
NeoGPS::Location_t realTargetWaypoint(0, 0L);
NeoGPS::Location_t virtualTargetWaypoint(0, 0L);

int cachedLastTrueHeading;

unsigned long distanceToNextRealWaypointMeters;
unsigned long distanceToNextVirtualWaypointMeters;
unsigned long lastVirtualWaypointGeneratedTimeMillis;

void initNav() {
  targetWaypointIndex = readInt16FromEeprom(EEPROM_WAYPOINT_INDEX_ADDR);
  targetWaypointIndex = targetWaypointIndex < 0 ? 0 : targetWaypointIndex;
  Serial.print("Nav: ");
  Serial.println(targetWaypointIndex);
}

int cacheLastTrueHeading(int lastTrueHeading) {
  return cachedLastTrueHeading = lastTrueHeading;
}

int getCachedLastTrueHeading() {
  return cachedLastTrueHeading;
}

int getCurrentWaypointIndex() {
  return targetWaypointIndex;
}

unsigned long getDistanceToNextRealWaypointMeters() {
  return distanceToNextRealWaypointMeters;
}

NeoGPS::Location_t *getTargetWaypoint(gps_fix *lastGoodFix) {
  addTelemetryDataInt32_t("\tWn", targetWaypointIndex+1);  

  addNavTelemetry();

  if (!lastGoodFix->valid.location  ||
      getPilotMode() == AutonomousTesting  &&  !isTestingWaypointPresent(targetWaypointIndex)) {
    return NULL;
  }

  loadTargetWaypoints(&lastGoodFix->location);

  if (distanceToNextRealWaypointMeters < WAYPOINT_REACHED_MIN_DISTANCE_METERS) {
    // next real waypoint reached!
    targetWaypointIndex++;
    writeInt16ToEeprom(EEPROM_WAYPOINT_INDEX_ADDR, targetWaypointIndex);
    loadTargetWaypoints(&lastGoodFix->location);
  }

  return &virtualTargetWaypoint; // will be equal to next real when appropriate
}

void loadTargetWaypoints(NeoGPS::Location_t *currentLocation) {
  // loop between testing waypoints when in testing mode
  if (getPilotMode() == AutonomousTesting  &&  !isTestingWaypointPresent(targetWaypointIndex)) {
    resetWaypointIndex();
  }

  Waypoint wp = getCurrentTargetRouteWaypoint();
  realTargetWaypoint.lat(wp.lat);
  realTargetWaypoint.lon(wp.lon);

  distanceToNextRealWaypointMeters = distanceToNextVirtualWaypointMeters = currentLocation->DistanceKm(realTargetWaypoint)*1000;

  if (!isNextWaypointReal) {
    distanceToNextVirtualWaypointMeters = currentLocation->DistanceKm(virtualTargetWaypoint)*1000;
    if (distanceToNextVirtualWaypointMeters < WAYPOINT_REACHED_MIN_DISTANCE_METERS) {
      // next virtual waypoint reached
      isNextWaypointReal = true;
    }
  }

  // generate next virtual waypoint if we just reached a previous one, and distance to next
  // real one is bigger than virtual waypoint distance
  if (isNextWaypointReal  &&  distanceToNextRealWaypointMeters > VIRTUAL_WAYPOINT_DISTANCE_METERS) {
    generateNextVirtualWaypoint(currentLocation);
    isNextWaypointReal = false;
  }

  // or if still target virtual waypoint but was too long trying to reach it
  if (!isNextWaypointReal  &&  isTimedOutTryingToReachNextVirtualWaypoint()) {
    generateNextVirtualWaypoint(currentLocation);
  }

  // most likely real waypoint is too close to generate another virtual one, use real
  if (isNextWaypointReal) {
    virtualTargetWaypoint.lat(realTargetWaypoint.lat());
    virtualTargetWaypoint.lon(realTargetWaypoint.lon());
  }

  distanceToNextVirtualWaypointMeters = currentLocation->DistanceKm(virtualTargetWaypoint)*1000;
}

Waypoint getCurrentTargetRouteWaypoint() {
  return getRouteWaypoint(targetWaypointIndex);
}

// start in current location and produce another virtual waypoint N meters straight on course to the next real waypoint
void generateNextVirtualWaypoint(NeoGPS::Location_t *currentLocation) {
  virtualTargetWaypoint.lat(currentLocation->lat());
  virtualTargetWaypoint.lon(currentLocation->lon());
  
  float bearingToNextRealWaypoint = currentLocation->BearingTo(realTargetWaypoint);
  virtualTargetWaypoint.OffsetBy((VIRTUAL_WAYPOINT_DISTANCE_METERS/1000.0)/NeoGPS::Location_t::EARTH_RADIUS_KM, bearingToNextRealWaypoint);

  lastVirtualWaypointGeneratedTimeMillis = getConsistentMillis();
}

bool isTimedOutTryingToReachNextVirtualWaypoint() {
  unsigned long currentMillis = getConsistentMillis();

  if (currentMillis <= lastVirtualWaypointGeneratedTimeMillis) {
    return true;
  }

  // 1 kph speed ~= 16 meters/minute
  const unsigned long timeoutMillis = VIRTUAL_WAYPOINT_DISTANCE_METERS * 60L * 1000 / 16;

  return currentMillis - lastVirtualWaypointGeneratedTimeMillis > timeoutMillis;
}

void resetWaypointIndex() {
  targetWaypointIndex = 0;
  isNextWaypointReal = true;
  writeInt16ToEeprom(EEPROM_WAYPOINT_INDEX_ADDR, targetWaypointIndex);
}

bool isCloseToNextRealWaypoint() {
  return distanceToNextRealWaypointMeters != 0  &&  distanceToNextRealWaypointMeters < CLOSE_TO_WAYPOINT_DISTANCE_METERS;
}

void setIsNextWaypointRealFlag(bool flag) {
  isNextWaypointReal = flag;
}

void addNavTelemetry() {
  addTelemetryDataFloat("\tT", realTargetWaypoint.latF());
  addTelemetryDataFloat(",", realTargetWaypoint.lonF());

  addTelemetryDataFloat("\tV", virtualTargetWaypoint.latF());
  addTelemetryDataFloat(",", virtualTargetWaypoint.lonF());

  addTelemetryDataInt32_t("\tDr", distanceToNextRealWaypointMeters);  
  addTelemetryDataString("", isNextWaypointReal ? "+" : "-");
  addTelemetryDataInt32_t("\tDv", distanceToNextVirtualWaypointMeters);  
}