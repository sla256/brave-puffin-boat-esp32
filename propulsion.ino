#include <ESP32Servo.h>
#include <math.h>

#include "controls.h"
#include "nav.h"
#include "pilot.h"
#include "pins.h"
#include "power.h"
#include "propulsion.h"
#include "rc.h"
#include "simulator.h"
#include "telemetry.h"

// pulse widths are in microseconds
// absolute min (stop) and max (don't want to ever exceed to not draw too much current)
const int motorStopPulseWidth = 800;    // must start at this level to initialize ESC properly
const int motorEcoPulseWidth = 1215;    // minimal power at max efficiency, barebones propulsion and steering, can stall
const int motorDecentPulseWidth = 1370; // good baseline power (~30-45W), used when getting close to real waypoints
const int motorMaxPulseWidth = 1600;    // absolute max to not overheat.

// when propulsion is on, this will determine the starting point of the dynamically adjusted,
// more optimal "desired cruise" target
// it can be == to Eco or Decent, depending on battery voltage and proximity to real waypoints
// actual cruise pulse width can go up from here if there extra solar power, but can't drop below this level
int motorBaselinePulseWidth = motorEcoPulseWidth; // default value for boot only; dynamic later

// max reduce/increase for each motor during turns
const int motorTurnMaxPulseWidthIncrease = motorMaxPulseWidth / 10;

// this will be dynamically adjusted based on the drawn vs. charging current,
// and then potentially further adjusted based on the voltage
int motorDesiredCruisePulseWidth = motorDecentPulseWidth;

Servo motorLeft;
Servo motorRight;

struct PropulsionState propulsionState;

void initPropulsion() {
    Serial.println("PROP");
    pinMode(PIN_MOTOR_L, OUTPUT);
    pinMode(PIN_MOTOR_R, OUTPUT);

    motorLeft.attach(PIN_MOTOR_L);
    motorRight.attach(PIN_MOTOR_R);

    motorLeft.write(motorStopPulseWidth);
    motorRight.write(motorStopPulseWidth);
}

void handlePropulsion(gps_fix *lastGoodFix) {
    // power signals for left and right motors range from motorStopPulseWidth to motorMaxPulseWidth
    // max pulse will only be applied to one motor during turns
    // otherwise the power is normally capped at cruise max pulse
    // which also depends on battery charge level

    propulsionState.cruisePulseWidth = getAdjustedCruisePulseWidth();

    // in stopped state as a starting point, have to override below when all conditions are met
    propulsionState.motorLeftPulseWidth = motorStopPulseWidth;
    propulsionState.motorRightPulseWidth = motorStopPulseWidth;

    NeoGPS::Location_t* waypoint = getTargetWaypoint(lastGoodFix); // this may return next real or virtual waypoint

    // calculate desired bearing if next waypoint is known
    int bearingToTarget;
    PilotMode pilotMode = getPilotMode();
    if (pilotMode == AutoPilot) {
        bearingToTarget = getAutoPilotBearingDegrees();
    }

    if (waypoint != NULL  && (pilotMode == Autonomous  ||  pilotMode == AutonomousTesting)) {
        bearingToTarget = getNewAutonomousBearingDegrees(&lastGoodFix->location, waypoint);
    }

    // apply non zero power only if there is a known next waypoint,
    // which also will require having a recent GPS fix available
    // and if propulsion is on
    // and if battery voltage is sufficient to run motors
    if ((waypoint != NULL  ||  pilotMode != Autonomous) && isPropulsionOn()  &&
        propulsionState.cruisePulseWidth != motorStopPulseWidth) {
        // correction is negative (0 to -180) for left turn, positive for the right turn; PID based
        int courseCorrection = calculateCourseCorrection(bearingToTarget);

#ifndef REAL_MISSION
        if (pilotMode == FixedPowerNoSteering  ||  pilotMode == DynamicPowerNoSteering  ||
            getSimulationOnOffState()) {
            courseCorrection = 0;
        } else if (pilotMode == RemoteControlled) {
            courseCorrection = getLastSteeringInput();
        }
#endif

        // map course correction to the pulse width increase proportionally, capping at max turn pulse increase
        // -motorTurnMaxPulseWidthIncrease to zero for left turns
        // zero to +motorTurnMaxPulseWidthIncrease for right turns
        // zero for staying course, i.e. both motors will run at cruise speed
        int pulseWidthCourseCorrection = map(courseCorrection, -180, 180,
            -motorTurnMaxPulseWidthIncrease, motorTurnMaxPulseWidthIncrease);

        // decrease or increase each motor's pulse based on the required correction
        // note if left motor's pulse is decreased relative to cruise, then rright's pulse will increase,
        // and vice versa
        propulsionState.motorLeftPulseWidth = propulsionState.cruisePulseWidth + pulseWidthCourseCorrection;
        propulsionState.motorRightPulseWidth = propulsionState.cruisePulseWidth - pulseWidthCourseCorrection;
    }

    // if for some result specific L/R PWM drops below the min, override to the min; o/w keep
    propulsionState.motorLeftPulseWidth = max(propulsionState.motorLeftPulseWidth, motorStopPulseWidth);
    propulsionState.motorRightPulseWidth = max(propulsionState.motorRightPulseWidth, motorStopPulseWidth);

    // similary, cap max PWM
    propulsionState.motorLeftPulseWidth = min(propulsionState.motorLeftPulseWidth, motorMaxPulseWidth);
    propulsionState.motorRightPulseWidth = min(propulsionState.motorRightPulseWidth, motorMaxPulseWidth);

    motorLeft.write(propulsionState.motorLeftPulseWidth);
    motorRight.write(propulsionState.motorRightPulseWidth);
}

// returns "currently best" pulse width, adjusted based on battery charge
//
// cuts off propulsion if battery voltage is <= BAT_VOLTAGE_INSUFFICIENT_FOR_PROPULSION_MV
// estimated run time of the navigation board on 25% battery without any charging is several days
int getAdjustedCruisePulseWidth() {
    int batteryVoltageMv = getLastBatteryVoltageMv();

#ifndef REAL_MISSION    
    if (getPilotMode() == RemoteControlled) {
        int pulseWidthIncrementFromThrottleInput = map(getLastThrottleInput(), 0, 100,
            0, motorMaxPulseWidth - motorStopPulseWidth);
        return motorStopPulseWidth + pulseWidthIncrementFromThrottleInput;
    }
#endif

    // enter or sustain cut off mode. enter if Vbat <= cut off voltage, exit when gets above 10.7
    if (batteryVoltageMv <= BAT_VOLTAGE_INSUFFICIENT_FOR_PROPULSION_MV  ||
        propulsionState.isPropulsionInCutOffMode  &&  
        batteryVoltageMv < BAT_VOLTAGE_SUFFICIENT_FOR_PROPULSION_MV) {

        propulsionState.isPropulsionInCutOffMode = true;
        motorBaselinePulseWidth = motorEcoPulseWidth;

        return motorStopPulseWidth;
    }

    // exit cut off mode?
    if (batteryVoltageMv >= BAT_VOLTAGE_SUFFICIENT_FOR_PROPULSION_MV) {
        propulsionState.isPropulsionInCutOffMode = false;
        motorBaselinePulseWidth = motorEcoPulseWidth;
    }

    // maintain at least decent baseline power regardless of the battert condition
    // if reasonably close to a real waypoint, to help with hitting it despite water / wind conditions
    if (isCloseToNextRealWaypoint()) {
        motorBaselinePulseWidth = max(motorBaselinePulseWidth, motorDecentPulseWidth);
    }

    // if no exception rules above happened, return currently desired value (see below)
    return motorDesiredCruisePulseWidth;
}

// incrementally +/- adjusts propulsion's desired power level
// for example, as battery status and chaging conditions change, this will be
// called to tell propulsion "feel free to use more / less power".
//
// has absolute min and max power values which won't be exceeded
void adjustPropulsionDesiredCruisePower(int pulseWidthAdjustment) {
    motorDesiredCruisePulseWidth += pulseWidthAdjustment;

    // set the target to be at least the baseline, if tries to go lower
    motorDesiredCruisePulseWidth = max(motorDesiredCruisePulseWidth, motorBaselinePulseWidth);

     // always cap the targe PWM at max, if tries to go higher
    motorDesiredCruisePulseWidth = min(motorDesiredCruisePulseWidth, motorMaxPulseWidth);
}

struct PropulsionState* getPropulsionState() {
    return &propulsionState;
}