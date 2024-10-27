#include <Arduino.h>

#include "controls.h"
#include "debug.h"
#include "pins.h"
#include "rc.h"

int lastSteeringInput;
int lastThrottleInput;

// when RF is off, default pulse width would be within mid +/- this value range
const int dsm2SignalMinPulseWidthThreshold = 30;

const int minSteeringPulseWidth = 1255; // all the way to the right
const int maxSteeringPulseWidth = 1455; // all left

const int midSteeringPulseWidth = (maxSteeringPulseWidth-minSteeringPulseWidth)/2; // middle when there is signal
const int midSteeringPulseWidthMin = midSteeringPulseWidth - dsm2SignalMinPulseWidthThreshold;
const int midSteeringPulseWidthMax = midSteeringPulseWidth + dsm2SignalMinPulseWidthThreshold;

const int minThrottlePulseWidth = 1558;
const int maxThrottlePulseWidth = 1758;

const int midThrottlePulseWidth = (minThrottlePulseWidth-maxThrottlePulseWidth)/2;
const int midThrottlePulseWidthMin = midThrottlePulseWidth - dsm2SignalMinPulseWidthThreshold;
const int midThrottlePulseWidthMax = midThrottlePulseWidth + dsm2SignalMinPulseWidthThreshold;

const unsigned long pulseInWaitTimeoutMicroSeconds = 25*1000;

#ifndef REAL_MISSION
void initRc() {
    pinMode(PIN_DSM2_INPUT_STEERING, INPUT);
    pinMode(PIN_DSM2_INPUT_THROTTLE, INPUT);
}

void handleRc() {
    handleDsm2RemoteInputs();
}

void handleDsm2RemoteInputs() {
    lastSteeringInput = pulseIn(PIN_DSM2_INPUT_STEERING, HIGH, pulseInWaitTimeoutMicroSeconds);
    lastThrottleInput = pulseIn(PIN_DSM2_INPUT_THROTTLE, HIGH, pulseInWaitTimeoutMicroSeconds);
}

// -100 all the way to the left, 100 all right; 0 is no signal / middle
int getLastSteeringInput() {
    if (lastSteeringInput > midSteeringPulseWidthMin  &&  lastSteeringInput < midSteeringPulseWidthMax) {
        return 0; // roughly somewhere in the middle, assume no input
    }
    
    return map(lastSteeringInput, minSteeringPulseWidth,  maxSteeringPulseWidth, 100, -100);
}

int getLastThrottleInput() {
    if (lastThrottleInput > midThrottlePulseWidthMin  &&  lastThrottleInput < midThrottlePulseWidthMin) {
        return 0; // roughly somewhere in the middle, assume no input
    }
    
    return map(lastThrottleInput, minThrottlePulseWidth,  maxThrottlePulseWidth, -100, 100);
}
#else
void initRc() {
}

void handleRc() {
}
#endif