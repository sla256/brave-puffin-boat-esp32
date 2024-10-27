#ifndef COMPASS_H
#define COMPASS_H

#include <Arduino.h>

void initCompass();
int getCompassMagneticHeading();
int getCachedLastMagneticHeading();
int convertMagneticHeadingToTrueViaMagneticVariation(int magneticHeadingDegrees, int latitude, int longitude);
int convertTrueHeadingToMagneticViaMagneticVariation(int trueHeadingDegrees, int latitude, int longitude);
void enableCompassAutoCalibration();
void disableCompassAutoCalibration();
void eraseCompassCalibrationProfile();
int8_t getLastPitch();
int8_t getLastRoll();
uint8_t getLastMagnetomerCalibrationState();
uint8_t getLastAccelerometerCalibrationState();
uint8_t getLastSystemCalibrationState();
void saveCompassCalibrationProfile();
int adjustHeading(int headingDegrees, int correction);
int getMaxCountOfReadingsWithBadCompassCalibration();

#endif