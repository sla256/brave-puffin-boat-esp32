// https://robot-electronics.co.uk/files/cmps14.pdf

#define HEADING_16_BIT_REGISTER     2     // Register to read 16 bit heading
#define CALIBRATION_STATE_REGISTER  0x1E

#define CALIBRATION_COMM1_REGISTER  0xF0
#define CALIBRATION_COMM2_REGISTER  0xF1
#define CALIBRATION_COMM3_REGISTER  0xF2

#define TIMEOUT_CYCLES 10000

#include <Arduino.h>
#include <Wire.h>

#include "compass.h"
// #include "debug.h"
#include "magvar.h"

const uint16_t cmps14Address = 0x60;
uint16_t cmpsI2cAddress = cmps14Address;

int8_t pitch, roll;
unsigned int heading;
uint8_t calibrationState;
uint8_t readBuffer[4];
uint8_t lastStatus;

bool isAutoCalibrationModeEnabled = false;
int countOfReadingsWithBadCompassCalibration; // count of continuous readings where reported calibration is zero
int maxCountOfReadingsWithBadCompassCalibration;

int cachedLastMagneticHeading = 0;

void initCompass() {
  Serial.print("CMPS: ");
  bool i2cJoinSuccess = Wire.begin();
  Serial.println(i2cJoinSuccess);
  Serial.println(Wire.getClock());
  disableCompassAutoCalibration();
  // enableCompassAutoCalibration();
}

void enableCompassAutoCalibration() {
  setCalibrationConfiguration(B10010111);
  isAutoCalibrationModeEnabled = true;
}

void disableCompassAutoCalibration() {
  setCalibrationConfiguration(B10000000);
  isAutoCalibrationModeEnabled = false;
}

void eraseCompassCalibrationProfile() {
  writeToCommandRegister(0xE0);
  writeToCommandRegister(0xE5);
  writeToCommandRegister(0xE2);
  delay(300);
}

void saveCompassCalibrationProfile() {
  writeToCommandRegister(0xF0);
  writeToCommandRegister(0xF5);
  writeToCommandRegister(0xF6);
}

void setCalibrationConfiguration(uint8_t calibrationConfigByte) {
  writeToCommandRegister(0x98);
  writeToCommandRegister(0x95);
  writeToCommandRegister(0x99);
  writeToCommandRegister(calibrationConfigByte);
}

uint8_t writeToCommandRegister(uint8_t commandToWrite) {
  Wire.beginTransmission(cmpsI2cAddress);
  Wire.write(0x00);
  delay(20);
  Wire.write(commandToWrite);
  delay(20);
  lastStatus = Wire.endTransmission();
  
  return lastStatus;
}

bool readRegisters(uint8_t registerToStartReadingFrom, uint8_t* readIntoBuffer, uint8_t byteCountToRead) {
  Wire.beginTransmission(cmpsI2cAddress);
  Wire.write(registerToStartReadingFrom);

  lastStatus = Wire.endTransmission(false);
  if (lastStatus != 0) {
    return false;
  }

  Wire.requestFrom(cmpsI2cAddress, byteCountToRead, true);
  if (!waitForWireAvailability(byteCountToRead)) {
    return false;
  }

  for(int i = 0; i < byteCountToRead; i++) {
    readIntoBuffer[i] = Wire.read();
  }

  return true;
}

void printI2cErrorsIfAny(char *step) {
  if (lastStatus != 7  &&  lastStatus != 0) {
    Serial.print(step);
    Serial.print(lastStatus);
  }
}

bool waitForWireAvailability(int byteCountToRead) {
  int timeout = TIMEOUT_CYCLES;
  while(Wire.available() < byteCountToRead && timeout-- > 0);

  return timeout > 0;
}

// takes about 5ms at 80MHz
int getCompassMagneticHeading() {
  long threeReadings =
    getTiltCompensatedMagneticHeadingInDegreesInternal() +
    getTiltCompensatedMagneticHeadingInDegreesInternal() +
    getTiltCompensatedMagneticHeadingInDegreesInternal();

  manageAutoCalibrationState();

  return cachedLastMagneticHeading = threeReadings / 3;
}

int getCachedLastMagneticHeading() {
  return cachedLastMagneticHeading;
}

void manageAutoCalibrationState() {
  // software samples compass ~30 times/sec. if reported calibration has been zero for more than a 
  // minute, and auto calibration is off, enable it
  if (!isAutoCalibrationModeEnabled  &&  countOfReadingsWithBadCompassCalibration > 60*30) {
    eraseCompassCalibrationProfile();
    enableCompassAutoCalibration();
  }

  // if auto calibration is on, and non zero calibration is achieved, save calibration eraseCompassCalibrationProfile
  // and disable autocalibration
  if (isAutoCalibrationModeEnabled  &&  getLastMagnetomerCalibrationState() > 0) {
    saveCompassCalibrationProfile();
    disableCompassAutoCalibration();
  }
}

int getMaxCountOfReadingsWithBadCompassCalibration() {
  return maxCountOfReadingsWithBadCompassCalibration;
}

int getTiltCompensatedMagneticHeadingInDegreesInternal() {
  if (!readRegisters(HEADING_16_BIT_REGISTER, readBuffer, 4)) {
    return 0;
  }

  heading = (readBuffer[0] << 8) + readBuffer[1];
  heading = heading / 10;

  pitch = readBuffer[2];
  roll = readBuffer[3];
  
  readCalibrationState();

  countOfReadingsWithBadCompassCalibration = getLastMagnetomerCalibrationState() == 0 ?
    (countOfReadingsWithBadCompassCalibration + 1) : 0;
  
  if (maxCountOfReadingsWithBadCompassCalibration < countOfReadingsWithBadCompassCalibration) {
    maxCountOfReadingsWithBadCompassCalibration = countOfReadingsWithBadCompassCalibration;
  }

  // sensor is mounted 90 degrees counter clockwise, adjust
  return heading >= 270 ? heading - 270 : heading + 90;
}

// lat and long should be rounded, degrees only, signed whole numbers e.g. 42,-71
int convertMagneticHeadingToTrueViaMagneticVariation(int magneticHeadingDegrees, int latitude, int longitude) {
  return adjustHeading(magneticHeadingDegrees, getMagVarByLatLong(latitude, longitude));
}

int convertTrueHeadingToMagneticViaMagneticVariation(int trueHeadingDegrees, int latitude, int longitude) {
  return adjustHeading(trueHeadingDegrees, -getMagVarByLatLong(latitude, longitude));
}

int adjustHeading(int headingDegrees, int correction) {
  int adjustedHeading = headingDegrees + correction;

  if (adjustedHeading < 0) {
    adjustedHeading += 360;
  }
  
  if (adjustedHeading >= 360) {
    adjustedHeading -= 360;
  }

  return adjustedHeading;
}


// +/- 90
// roll and pitch are reversed because of sensor's orientation
int8_t getLastPitch() {
  return roll;
}

// +/- 90
int8_t getLastRoll() {
  return pitch;
}

uint8_t getLastMagnetomerCalibrationState() {
  return calibrationState & B00000011;
}

uint8_t getLastAccelerometerCalibrationState() {
  return calibrationState & B00001100 >> 2;
}

uint8_t getLastSystemCalibrationState() {
  return (calibrationState & B11000000) >> 6;
}

uint8_t readCalibrationState() {
  readRegisters(CALIBRATION_STATE_REGISTER, readBuffer, 1);
  return calibrationState = readBuffer[0];
}
