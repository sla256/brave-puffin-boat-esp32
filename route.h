#include <Arduino.h>

#ifndef ROUTE_H
#define ROUTE_H

struct Waypoint {
  int32_t lat;
  int32_t lon; 
};

void initRoute();
Waypoint getAutonomousMissionRouteWaypoint(int waypointIndex);
Waypoint getRouteWaypoint(int waypointIndex);

#endif