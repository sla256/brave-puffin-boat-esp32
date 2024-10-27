#include "controls.h"
#include "route.h"
#include "test.h"

// TODO final route
const Waypoint routeWaypoints [] = {
  {41.889272*10000000,-69.943068*10000000}, // start: marconi beach, 0km from prev,km total, days total
  {42.69553*10000000,-65.47453*10000000}, // south of nova scotia outbound, 378.275883763757km from prev,378.275883763757km total,15.5747416790546 days total
  {43.418218*10000000,-59.870919*10000000}, // south of sable island outbound, 462.222851311118km from prev,840.498735074875km total,25.2043844147029 days total
  {46.11451*10000000,-52.84976*10000000}, // south of labrador outbound, 629.856126656706km from prev,1470.35486173158km total,38.3263870533843 days total
  {51*10000000,-16*10000000}, // east line target center point, 2735.8336115513km from prev,4206.18847328288km total,95.3229206273697 days total
  {45*10000000,-30*10000000}, // north of azores, 1233.90202972453km from prev,5440.09050300741km total,121.029212913297 days total
  {44*10000000,-53*10000000}, // south of labrador inbound, 1821.37260535993km from prev,7261.46310836734km total,158.974475524963 days total
  {43.353556*10000000,-59.870704*10000000}, // south of sable island inbound, 557.040051183708km from prev,7818.50315955105km total,170.57947659129 days total
  {42.602859*10000000,-65.473474*10000000}, // south of nova scotian inbound, 463.280617808607km from prev,8281.78377735966km total,180.231156128969 days total
  {41.872847*10000000,-69.912847*10000000}, // marconi beach approach, 374.325926797375km from prev,8656.10970415703km total,188.029612937248 days total
  {41.869916*10000000,-69.949104*10000000}, // marconi beach finish, 3.01974563744647km from prev,8659.12944979448km total,188.092524304695 days total
};

const int waypointsCount = sizeof(routeWaypoints)/sizeof(Waypoint);

void initRoute() {
  Serial.print(waypointsCount);
  Serial.println(F(" waypoints"));
  for(int i=0; i < waypointsCount; i++) {
    Waypoint waypoint = getAutonomousMissionRouteWaypoint(i);
    Serial.print(i);
    Serial.print(":");
    Serial.print(waypoint.lat);
    Serial.print(",");
    Serial.println(waypoint.lon);
  }
}

Waypoint getAutonomousMissionRouteWaypoint(int waypointIndex) {
  // if index is out of bounds, alternate between last and previous to last waypoints, forever
  if (waypointIndex >= waypointsCount) {
    waypointIndex = waypointsCount - 1 - waypointIndex%2; // external waypoint index will keep incrementing
  }
  
  return routeWaypoints[waypointIndex];
}

// either from the autonomous mission rout or testing waypoints
Waypoint getRouteWaypoint(int waypointIndex) {
  if (getPilotMode() == Autonomous) {
    return getAutonomousMissionRouteWaypoint(waypointIndex);
  } else {
    Waypoint wp;
    wp.lat = getTestingWaypointLat(waypointIndex);
    wp.lon = getTestingWaypointLon(waypointIndex);
    return wp;
  }
}