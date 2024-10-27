#include "nav.h"
#include "test.h"

// explicitly set waypotins, used only in AutonomousTesting mode
#define MAX_WAYPOINTS 10
int32_t latTestingWaypoints[MAX_WAYPOINTS + 1];
int32_t lonTestingWaypoints[MAX_WAYPOINTS + 1];

void setWaypointsForRoundtrip(int roundtripDistanceMeters, NeoGPS::Location_t *currentLocation) {
  // straight on current heading, roundtripDistanceMeters away
  NeoGPS::Location_t roundTripDestination(*currentLocation);
  int cachedLastTrueHeading = getCachedLastTrueHeading();
  roundTripDestination.OffsetBy((roundtripDistanceMeters/1000.0)/NeoGPS::Location_t::EARTH_RADIUS_KM, cachedLastTrueHeading*PI/180.0);

  // 50 meters back, on heading opposite to current
  int oppositeHeading = cachedLastTrueHeading < 180 ? cachedLastTrueHeading + 180 : cachedLastTrueHeading - 180;
  NeoGPS::Location_t backedUpCurrentLocation(*currentLocation);
  backedUpCurrentLocation.OffsetBy((50/1000.0)/NeoGPS::Location_t::EARTH_RADIUS_KM, oppositeHeading*PI/180.0);

  setCourseWaypointAtGivenGpsLocation(0, currentLocation);
  setCourseWaypointAtGivenGpsLocation(1, &roundTripDestination);
  setCourseWaypointAtGivenGpsLocation(2, &backedUpCurrentLocation);

  setIsNextWaypointRealFlag(true);
}

// waypointIndex starts at 0
bool setCourseWaypointAtGivenGpsLocation(int waypointIndex, NeoGPS::Location_t *location) {
  if (waypointIndex > MAX_WAYPOINTS) {
    return false;
  }

  latTestingWaypoints[waypointIndex] = location->lat();
  lonTestingWaypoints[waypointIndex] = location->lon();

  Serial.print(latTestingWaypoints[waypointIndex]);
  Serial.print(",");
  Serial.println(lonTestingWaypoints[waypointIndex]);

  if (waypointIndex < MAX_WAYPOINTS) {
    latTestingWaypoints[waypointIndex+1] = lonTestingWaypoints[waypointIndex+1] = 0;
  }

  setIsNextWaypointRealFlag(true);

  return true;
}

bool isTestingWaypointPresent(int waypointIndex) {
    if (waypointIndex >= MAX_WAYPOINTS) {
        return false;
    }

    return latTestingWaypoints[waypointIndex] != 0  &&  lonTestingWaypoints[waypointIndex] != 0;
}

int32_t getTestingWaypointLat(int waypointIndex) {
    if (!isTestingWaypointPresent(waypointIndex)) {
        return 0;
    }

    return latTestingWaypoints[waypointIndex];
}

int32_t getTestingWaypointLon(int waypointIndex) {
    if (!isTestingWaypointPresent(waypointIndex)) {
        return 0;
    }

    return lonTestingWaypoints[waypointIndex];
}
