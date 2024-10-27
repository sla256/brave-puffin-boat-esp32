#ifndef CONTROLS_H
#define CONTROLS_H

// TODO
// mission prep:
// update route table with final waypoints
// reflash with REAL_MISSION undefined
// check compass calibration
// send G via serial to reset waypoints and start the mission
// send s stop motors
// reflash with REAL_MISSION defined
// reset chip and read off route waypoints
// check reed switch

// #define REAL_MISSION

enum PilotMode {
    Autonomous,             // follows preprogrammed route waypoints for the actual mission
    AutonomousTesting,      // same as autonomous except waypoints can be given during testing
    AutoPilot,              // follows given true geo bearing under prescribed power
    FixedPowerNoSteering,   // both motors at equal, adjustable power; no dynamic power management
    DynamicPowerNoSteering, // both motors at equal, automatically managed power
    RemoteControlled        // listens for RC inputs for steering and throttle
};

void initControls();
void handleControls();
PilotMode getPilotMode();
bool isPropulsionOn();

#endif