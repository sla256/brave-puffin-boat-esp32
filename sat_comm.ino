#include <IridiumSBD.h> // see http://librarymanager/All#IridiumSBDI2C  / https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library
#include "sat_comm.h"
#include "pins.h"

enum SatCommState {
  SatCommOff,
  SatCommTurningOn,
  SatCommInitializing,
  SatCommSelfTesting,
  SatCommConnecting,
  SatCommSending,
  SatCommTurningOff
};

const int satCommAtTimeoutSec = 10;
const int satCommLoopDelayMs = 10*1000;
const int satCommDelayBetweenSuccessfulCommSessionsMs = 4*60*60*1000; // 3*60*1000 for testing
const int satCommOneStepExecutionMaxTimeMs = 90*1000;
const int satCommDelayBetweenRetryingFailedCommSessionsMs = 15*60*1000;

SatCommState satCommState = SatCommOff;
SatTrackingRecord* lastSetSatTrackingRecord;
SatTrackingRecord emptyTrackingRecord;

int satCommLastSignalQuality = -1;
int satCommLastError = ISBD_SUCCESS;

// this should initiate sat comm session shortly (step exec max time) after chip's restart,
// instead of waiting the regular, much longer delay between comm sessions time.
// this is to: 1) communicate position even if the chip crashes more often than regular comm sessions occur; but
// 2) to not start sat comm session right away after restart, in case sat code is the root cause of restarts.
unsigned long lastStateChangeMillis = -satCommDelayBetweenSuccessfulCommSessionsMs + satCommOneStepExecutionMaxTimeMs;

unsigned long lastSatCommLoopCallMillis = 0;

SatCommRunningCoreSelection satCommCoreSelection;

#define IridiumSerial Serial2
IridiumSBD modem(IridiumSerial);

TaskHandle_t core0SatCommTaskHandle;

// Arduino's code (main loop) runs on core 1 by default.
// Satellite comm code can use either core 1 or the other spare core 0 to handle delay-prone satellite communications.
// In both cases, sat comm logic is handled via a state machine - in case either core needs to run other tasks
// IMEI 300434064534220 for 2024 modem
void initSatComm(SatCommRunningCoreSelection coreSelector) {
    pinMode(PIN_SAT_SLEEP, OUTPUT);
    digitalWrite(PIN_SAT_SLEEP, LOW);

    lastSetSatTrackingRecord = &emptyTrackingRecord;

    Serial.print("SAT_COMM ");
    Serial.println(sizeof(SatTrackingRecord));

    modem.adjustATTimeout(satCommAtTimeoutSec); // default is 30

    satCommCoreSelection = coreSelector;

    if (satCommCoreSelection == SatCommRunOnSpareCore0) {
        xTaskCreatePinnedToCore(core0SatCommTask, "core0SatCommTask", 10000, NULL, 1, &core0SatCommTaskHandle, 0);
    }
}

void handleSatComm() {
    // no-op if sat comm is running on spare core 0
    if (satCommCoreSelection == SatCommRunOnSpareCore0) {
        return;
    }

    handleCurrentSatCommState(millis());
}

// this core task should never return, otherwise the core/chip will panic
// wakes up every satCommLoopDelayMs ms to check the state, do the comm work if it's time, etc.
void core0SatCommTask(void *) {
    // esp_task_wdt_delete(NULL);
    // esp_task_wdt_deinit();

    while(true) {
        handleCurrentSatCommState(millis());
        delay(satCommLoopDelayMs);
    }
}

void handleCurrentSatCommState(unsigned long currentMillis) {
    // no-op if less than satCommLoopDelayMs time passed since last call
    if (currentMillis - lastSatCommLoopCallMillis < satCommLoopDelayMs) {
        return;
    }

    lastSatCommLoopCallMillis = currentMillis;
    unsigned long timeLapsedSinceLastStateChangeMs = currentMillis - lastStateChangeMillis;
    SatCommState newSatCommState = satCommState;
    char imei[16];
    int error = -1;

    switch (satCommState)
    {
        case SatCommOff:
            digitalWrite(PIN_SAT_SLEEP, LOW);
            if (timeLapsedSinceLastStateChangeMs > satCommDelayBetweenSuccessfulCommSessionsMs  ||  timeLapsedSinceLastStateChangeMs < 0) {
                newSatCommState = SatCommTurningOn;    
            }
            if (satCommLastError != ISBD_SUCCESS  &&  timeLapsedSinceLastStateChangeMs > satCommDelayBetweenRetryingFailedCommSessionsMs) {
                newSatCommState = SatCommTurningOn;
            }
            break;

        case SatCommTurningOn:
            digitalWrite(PIN_SAT_SLEEP, HIGH);
            IridiumSerial.begin(19200, SERIAL_8N1, PIN_SAT_RX, PIN_SAT_TX);
            newSatCommState = SatCommInitializing;
            break;

        case SatCommInitializing:
            error = modem.begin();
            newSatCommState = (error == ISBD_SUCCESS ? SatCommSelfTesting : satCommState);
            break;

        case SatCommSelfTesting:
            error = modem.getIMEI(imei, sizeof(imei));
            newSatCommState = (error == ISBD_SUCCESS ? SatCommConnecting : satCommState);
            break;

        case SatCommConnecting:
            error = modem.getSignalQuality(satCommLastSignalQuality); // quality on the scale of 0 to 5; 2+ is recommended for sending
            if (error == ISBD_SUCCESS) {
                if (satCommLastSignalQuality > 1) {
                    newSatCommState = SatCommSending;
                } else {
                    error = ISBD_NO_NETWORK;
                }
            }
            break;

        case SatCommSending:
            error = modem.sendSBDBinary((const uint8_t*)lastSetSatTrackingRecord, sizeof(SatTrackingRecord));
            newSatCommState = (error == ISBD_SUCCESS ? SatCommTurningOff : satCommState);
            break;

        case SatCommTurningOff:
            digitalWrite(PIN_SAT_SLEEP, LOW);
            newSatCommState = SatCommOff;
            break;
    }

    if (satCommState != newSatCommState) {
        lastStateChangeMillis = currentMillis;
        satCommState = newSatCommState;
        timeLapsedSinceLastStateChangeMs = 0;
    }

    if (satCommState != SatCommOff  &&  timeLapsedSinceLastStateChangeMs > satCommOneStepExecutionMaxTimeMs) {
        Serial.println("Sat comm timed out in current step, aborting & turning off");
        satCommState = SatCommTurningOff;
    }

    if (error != -1) {
        satCommLastError = error;
    }
}

// beware of cross-core race conditions - the incoming record can be modified in 
// core 1 (main loop) while being used by core 0 (sat comm loop)
void satCommSetTrackingRecordForNextCommSession(SatTrackingRecord* _trackingRecord) {
    lastSetSatTrackingRecord = _trackingRecord;
}

int satCommGetLastSignalQuality() {
    return satCommLastSignalQuality;
}

int satCommGetCurrentState() {
    return satCommState;
}

int satCommGetLastError() {
    return satCommLastError;
}

void startSatCommSession() {
    lastStateChangeMillis = -satCommDelayBetweenSuccessfulCommSessionsMs;
}
