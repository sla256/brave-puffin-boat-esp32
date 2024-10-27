#ifndef SENSORS_H
#define SENSORS_H

#include "ina219.h"

void initSensors();

enum Ina219SensorSelector {
    LeftMotor,
    RightMotor
};

int getCurrentFromIna219(Ina219SensorSelector sensorSelector); // in mA
int getVoltageFromIna219(Ina219SensorSelector sensorSelector); // in mV


enum Ina226SensorSelector {
    BatteryIna226,
    SolarIna226
};

int getCurrentFromIna226(Ina226SensorSelector sensorSelector); // in mA
int getVoltageFromIna226(Ina226SensorSelector sensorSelector); // in mV


enum Bme280SensorSelector {
    OnboardBme280
    // ExternalBme280
};


enum Tmp36SensorSelector {
    MainUnderwaterBattery
};

int getTemperatureFromBme280(Bme280SensorSelector sensorSelector); // temp in C
int getHumidityFromBme280(Bme280SensorSelector sensorSelector);
int getPressureFromBme280(Bme280SensorSelector sensorSelector); // millibars; 1013.25 mbar - sea level

int getTemperatureFromTmp36(Tmp36SensorSelector sensorSelector); // temp in C

float getVoltageFromAdc(uint8_t channel);

#endif