#ifndef PID_H
#define PID_H

#define PID_TUNING

struct PidControllerStorage {
  unsigned long lastComputeTimeMillis;
  int previousError;
  long cumulativeError;
};

long computePidOutput(struct PidControllerStorage* pidStorage, int setpoint, int input, bool newSetpoint);

void tunePidKp(float tuningAmount);
void tunePidKi(float tuningAmount);
void tunePidKd(float tuningAmount);
void resetPidFactorsForTuning();

#endif