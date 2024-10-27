#ifndef PROPULSION_H
#define PROPULSION_H

#include <NMEAGPS.h>

struct PropulsionState {
    // when propulsion is on, this will determine the minimum pulse width, before additional dynamical adjustments
    // it can move up and down depending on battery voltage
    // actual current / cruise pulse width can go up from here if there extra solar power, but can't drop below this level
    int motorBaselinePulseWidth = 1000;

    // dynamically adjusted propulsion basline / pulse width, which will be used as a basis for left / right
    // motor inputs
    int cruisePulseWidth;

    int motorLeftPulseWidth;
    int motorRightPulseWidth;

    // if true, propulsion is off because of low battery voltage
    // enter cut off mode when battery voltage reaches lower value of the cut off band (e.g. 10.5V)
    // then wait until voltage comes back to the upper value (e.g. 10.7V) to exit cut off mode
    // this is to avoid frequent propulsion on / off during initial charging phase
    bool isPropulsionInCutOffMode = false;
};

void initPropulsion();
void adjustPropulsionDesiredCruisePower(int pulseWidthAdjustment);
void handlePropulsion(gps_fix *lastGoodFix);
struct PropulsionState* getPropulsionState();

#endif