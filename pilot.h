#include <NMEAGPS.h>

int getAutoPilotBearingDegrees();
void adjustAutoPilotBearing(int bearingCorrectionDegrees);

int getNewAutonomousBearingDegrees(NeoGPS::Location_t* currentLocation, NeoGPS::Location_t* targetWaypoint);
int getLastAutonomousBearingDegrees();

int calculateCourseCorrection(int bearing);
