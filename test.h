#include <Arduino.h>
#include <NMEAGPS.h>

bool setCourseWaypointAtGivenGpsLocation(int waypointIndex, NeoGPS::Location_t *location);
void setWaypointsForRoundtrip(int roundtripDistanceMeters, NeoGPS::Location_t *currentLocation);

bool isTestingWaypointPresent(int waypointIndex);
int32_t getTestingWaypointLat(int waypointIndex);
int32_t getTestingWaypointLon(int waypointIndex);
