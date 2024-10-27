#include <Arduino.h>
#include <NMEAGPS.h>

void initSimulator(int simulationRate);

bool getSimulationOnOffState();
void resetSimulationState(bool onOff);
int getLastSimulatedMagneticHeading();

NeoGPS::Location_t* generateSimulatedGpsLocation();