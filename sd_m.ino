#include <Arduino.h>

#include "debug.h"
#include "pins.h"
#include "eeprom_m.h"
#include "sd_m.h"
#include "telemetry.h"

#ifndef USE_SD_CARD

void initSd() {
}

void saveLogRecordToSdCard(char *data) {
}

#else

const int maxLogRecordsPerFile = 5000;
bool sdInitStatus;
int logFileNumber; // increments every maxLogRecordsPerFile
int logRecordCount;

// log file name is a unique combination of "run number" (increments on every boot)
// and log file number
char fileName[13];

void initSd() {
  mountCard();
  Serial.print("SD card: ");
  Serial.println(sdInitStatus);

  writeInt16ToEeprom(EEPROM_LOG_FILE_NUMBER_ADDR, 0);
  unmountCard();
}

void saveLogRecordToSdCard(char *data) {
  mountCard();
  if (!sdInitStatus) {
    // Serial.println("SD write aborted");
    unmountCard();
    return; 
  }

  logRecordCount++;

  if (logRecordCount > maxLogRecordsPerFile) {
    logRecordCount = 0;
    logFileNumber++;
    writeInt16ToEeprom(EEPROM_LOG_FILE_NUMBER_ADDR, logFileNumber);
  }

  snprintf(fileName, 12, "/%d-%d.csv", currentRunNumber(), logFileNumber);
  
  int writtenBytes = writeToFile(fileName, data);
  if (writtenBytes <= 0) {
    // Serial.println("SD write failed");
  }

  unmountCard();
}

void mountCard() {
  // safe to call repeatedly if card is already mounted
  // https://github.com/espressif/arduino-esp32/blob/master/libraries/SD/src/SD.cpp
  sdInitStatus = SD.begin(PIN_SD_CS);
  if (!sdInitStatus) {
    // Serial.println("Failed to mount SD card");
  }
}

void unmountCard() {
  // safe to call repeatedly if card is not mounted
  // https://github.com/espressif/arduino-esp32/blob/master/libraries/SD/src/SD.cpp
  
  // however memory will leak if called repeatedly to mount / unmount, until this is fixed:
  // https://github.com/espressif/arduino-esp32/issues/2897
  SD.end(); 
}

int writeToFile(char* fileName, char *data) {
  File file = SD.open(fileName, FILE_APPEND);
  if (!file) {
    return 0;
  } 
  
  int writtenBytes = file.print(currentRunNumber());
  writtenBytes += file.print(",");
  
  writtenBytes += file.print(logFileNumber);
  writtenBytes += file.print(",");

  writtenBytes += file.print(logRecordCount);
  writtenBytes += file.print(",");
  
  writtenBytes += file.println(data);
  file.close();

  return writtenBytes;
}

#endif