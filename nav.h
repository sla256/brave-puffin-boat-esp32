#include <NMEAGPS.h>
#include "route.h"

#define WAYPOINT_REACHED_MIN_DISTANCE_METERS 25
#define VIRTUAL_WAYPOINT_DISTANCE_METERS 400

void initNav();
NeoGPS::Location_t *getTargetWaypoint(gps_fix *lastGoodFix);
int cacheLastTrueHeading(int lastTrueHeading);
int getCachedLastTrueHeading();
bool isCloseToNextRealWaypoint();
void resetWaypointIndex();
void setIsNextWaypointRealFlag(bool flag);
Waypoint getCurrentTargetRouteWaypoint();
int getCurrentWaypointIndex();
unsigned long  getDistanceToNextRealWaypointMeters();