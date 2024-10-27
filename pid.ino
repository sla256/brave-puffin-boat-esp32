#include "pid.h"
#include "telemetry.h"

// basic PID controller implementation
float pidKp = 12;     // proportional term, controls speed of response; 2020 boat P=5.25
float pidKi = 0.00075;  // integral term, adds more action as error persists over time; 2020 boat I=0.00225
float pidKd = 0;       // derivative term, limits overshoots / oscillitations; 2020 boat D=15

// returns good, smooth yet effective correction value to get current value of some variable (input) to the 
// desired target value (setpoint) in the most optimal way
// set newSetpoint to true if the target value (setpoint) has changed compared to the last one,
// e.g. new bearing / steering input, etc.
long computePidOutput(struct PidControllerStorage *pidStorage, int setpoint, int input, bool newSetpoint) {
  unsigned long currentMillis = getConsistentMillis();

  if (newSetpoint) {
    pidStorage->previousError = 0;
    pidStorage->cumulativeError = 0;
  }

  // if lastComputeTimeMillis > currentMillis, the clock must have rolled over to zero, let's correct
  // with some not super accurate but positive value, part of the actual elapsed
  unsigned long elapsedTime = pidStorage->lastComputeTimeMillis > currentMillis ?
    pidStorage->lastComputeTimeMillis - currentMillis : currentMillis - pidStorage->lastComputeTimeMillis;

  int error = setpoint - input;
  float errorRate = (error - pidStorage->previousError) / (float) elapsedTime;
  pidStorage->cumulativeError += error * elapsedTime;

#ifdef PID_TUNING
  addTelemetryDataFloat("\tKp", pidKp);
  addTelemetryDataFloat("\tKi", pidKi);
  addTelemetryDataFloat("\tKd", pidKd);
  addTelemetryDataInt32_t("\tIn", input);
  addTelemetryDataInt32_t("\tEp", pidStorage->previousError);
  addTelemetryDataInt32_t("\tEr", error);
  addTelemetryDataFloat("\tER", errorRate);
  addTelemetryDataInt32_t("\tCe", pidStorage->cumulativeError);
#endif

  pidStorage->lastComputeTimeMillis = currentMillis;
  pidStorage->previousError = error;

  return pidKp*error + pidKi*pidStorage->cumulativeError + pidKd*errorRate;
}

// manual PID controller tuning procedure from https://pidexplained.com/how-to-tune-a-pid-controller/
//
// Start by setting the Integral and Derivative values to 0. Then increase the proportional until the controller starts to become 
// unstable and oscillate. Once the proportional value that causes the controller to oscillate is found, take this value and divide 
// it in half. This will be the starting P value.
//
// Once the proportional value is found, we can start to tune the integral. Always start with small steps when adjusting a PID controller, 
// and give time between each adjustment to see how the controller reacts. Increase the integral gain in small increments, and with 
// each adjustment, change the set point to see how the controller reacts. The goal of tuning the integral value, is to achieve an 
// adequate controller response or reaction time (after the initial response from the proportional is set).
// 
// Now that you have a stable PI controller, start by increasing the derivative value slowly, changing the set point, and allowing time 
// for the controller to stabilize. The purpose of the D value, is to monitor the ramp rate of the process value, and prevent it from 
// overshooting the set point. Continue to change the set point and increase the derivative until the overshoot has been dampened to an 
// acceptable level. If the controller starts to react in a negative way (unexpected changes in the output, poor control, or oscillation) 
// lessen the D value until the controller is stable again.


void resetPidFactorsForTuning() {
  pidKi = pidKd = 0;
  pidKp = 12;
}

void tunePidKp(float tuningAmount) {
  pidKp += tuningAmount;
}

void tunePidKi(float tuningAmount) {
  pidKi += tuningAmount;
}

void tunePidKd(float tuningAmount) {
  pidKd += tuningAmount;
}