#include "controls.h"
#include "nav.h"
#include "pid.h"

struct PidControllerStorage pidCourseCorrection;
int previousBearingDegrees;

int autoPilotBearingDegrees;
int lastAutonomousBearingDegrees;

// input in degrees. output is in abstract correction signal units:
// negative (0 to -180) for left turn, positive (0 to +180 for the right turn)
// the bigger the output signal, the harder the turn should be
int calculateCourseCorrection(int bearingToTarget) {
    int cachedLastTrueHeading = getCachedLastTrueHeading();

    // to make application of pid output easier for efficient (avoiding 180+ degree) turns,
    // remap current heading relative to the desired bearing on -180 to 180 scale.
    // i.e. if actual bearing is 170 and heading is 190. on the new scale the desired bearing is always zero,
    // the remapped heading will become +20 (indicating desired 20 degree course correction to the right)
    // if actual bearing is 80 and heading is 350, the remapped heading becomes -90,
    // indicating 90 degree desired left course correction
    int courseDelta = cachedLastTrueHeading - bearingToTarget;
    int remappedHeading = courseDelta < -180 ?
    (courseDelta + 360) : (courseDelta > 180 ? courseDelta - 360 : courseDelta);

    // apply PID controller to optimize turning: apply more correction over time if needed, avoid overshoots, etc.
    long pidOutput = computePidOutput(&pidCourseCorrection, 0, remappedHeading, previousBearingDegrees != bearingToTarget);
    previousBearingDegrees = bearingToTarget;

    // PID can return bigger values than a negative / positive 180, cap here
    int courseCorrection = pidOutput > 180 ? 180 : (pidOutput < -180 ? -180 : pidOutput);

    return courseCorrection;
}

int getAutoPilotBearingDegrees() {
  return autoPilotBearingDegrees;
}

int getNewAutonomousBearingDegrees(NeoGPS::Location_t* currentLocation, NeoGPS::Location_t* targetWaypoint) {
    return lastAutonomousBearingDegrees = currentLocation->BearingTo(NeoGPS::Location_t(*targetWaypoint))*180/PI;
}

int getLastAutonomousBearingDegrees() {
    PilotMode pilotMode = getPilotMode();
    return (pilotMode == Autonomous || pilotMode == AutonomousTesting) ?
      lastAutonomousBearingDegrees : autoPilotBearingDegrees;
}

// negative or positive adjustment
void adjustAutoPilotBearing(int courseCorrectionDegrees) {
  int newBearing = autoPilotBearingDegrees + courseCorrectionDegrees;
  
  newBearing = newBearing >= 360 ? newBearing - 360 : newBearing;
  newBearing = newBearing < 0 ? newBearing + 360 : newBearing;

  autoPilotBearingDegrees = newBearing;
}

