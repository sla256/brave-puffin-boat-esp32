#include <Arduino.h>

#include "controls.h"
#include "comm_bt.h"
#include "compass.h"
#include "debug.h"
#include "display.h"
#include "eeprom_m.h"
#include "gps.h"
#include "nav.h"
#include "pid.h"
#include "pilot.h"
#include "pins.h"
#include "propulsion.h"
#include "rc.h"
#include "sat_comm.h"
#include "simulator.h"
#include "test.h"

#define TIMEOUT_TO_SWITCH_RC_MODE_OFF_IF_IDLE_MS    2*60000

bool isPropulsionOnOff; // false is off
PilotMode pilotMode = Autonomous;

unsigned long lastMillisWhenDisplayControlWasNotTouched;
unsigned long lastMillisWhenDisplayWasTurnedOn;
unsigned long lastMillisWhenRcInputWasPresent;

unsigned long lastMillisWhenMissionSwitchWasFlipped;
bool wasMissionSwitchOnDuringLastSample;
int missionSwitchFlipCount;

const int missionSwitchFlipCountThreshold = 3;
const int missionSwitchFlipMinDurationMs = 1000;
const int missionSwitchFlipMaxDurationMs = 4000;
const int missionSwitchDebounceTimeMs = 300;

const int timeToHavePinTouchedToConsiderActivatedMillis = 200;
const int keepDisplayOnTimeMillis = 10*1000;
const int touchReadingThreshold = 80;

const int manualPulseWidthAdjustmentIncrement = 10;

int roundtripDistanceMeters = 0;

int rcThrottleInputFromPreviousCycle;

void initControls() {
    isPropulsionOnOff = readInt16FromEeprom(EEPROM_PROPULSION_STATE_ADDR);
}

PilotMode getPilotMode() {
    return pilotMode;
}

bool isPropulsionOn() {
  return isPropulsionOnOff;
}

void handleControls() {
    unsigned long currentMillis = getConsistentMillis();

    if (missionSwitchFlipCount == missionSwitchFlipCountThreshold) {
        Serial.println("Mission control switch activated");
        missionSwitchFlipCount = 0;
        setPropulsionOnOf(!isPropulsionOn());
    }


#ifndef REAL_MISSION
    handleDebugControls(currentMillis); 
    handleSerialInputs();
    handleBtInputs();
    handleRcInput();
#endif
}

#ifndef REAL_MISSION
void handleSerialInputs() {
    if (Serial.available() <= 0) return;
    
    processControlInput((char) Serial.read());

}

void handleBtInputs() {
    if (!isBtOn()) return;

    BT_SERIAL_TYPE* btSerial = getBtSerial();

    if (btSerial == NULL  ||  !isBtConnected()  ||  btSerial->available() <= 0) return;

    processControlInput((char) btSerial->read());
}

void handleRcInput() {
    int currentRcThrottleInput = getLastThrottleInput();
    if (currentRcThrottleInput > 50  &&  rcThrottleInputFromPreviousCycle > 50) {
        lastMillisWhenRcInputWasPresent = getConsistentMillis();
        roundtripDistanceMeters = 0;
        pilotMode = RemoteControlled;
    }

    // if in RC mode yet there is no RC input - drop into Autonomous mode but also
    // cut off propulsion to avoid the runaway
    if (pilotMode == RemoteControlled
        &&  getConsistentMillis() - lastMillisWhenRcInputWasPresent > TIMEOUT_TO_SWITCH_RC_MODE_OFF_IF_IDLE_MS) {
        roundtripDistanceMeters = 0;
        pilotMode = Autonomous;
        setPropulsionOnOf(false);
    }

    rcThrottleInputFromPreviousCycle = currentRcThrottleInput;
}

void processControlInput(char input) {
    switch (input)
    {
#ifndef REAL_MISSION
        case 'A': // intentionally disable built-in TWDT and hang the chip forever, to test external watchdog monitor
            // disableLoopWDT();
            Serial.println("Hanging...");
            while(true);
            break;
        case 'B': // test programmatic restart
            Serial.println("Restarting...");
            ESP.restart();
            break;
#endif
        case 'a':
            startSatCommSession();
            break;
        case 'b': // adjust desired bearing 1 degree right
            adjustAutoPilotBearing(1);
            break;  
        case 'c':
            enableCompassAutoCalibration();
            break;
        case 'd':
            adjustPropulsionDesiredCruisePower(-manualPulseWidthAdjustmentIncrement);
            break;
        case 'e':
            eraseCompassCalibrationProfile();
            break;
        case 'F':
            tunePidKd(1);
            break;
        case 'f':
            tunePidKd(-1);
            break;
        case 'g': // resume propulsion: do not reset waypoint
            setPropulsionOnOf(true);
            break;
        case 'G': // begin autonomous mission from the first waypoint
            pilotMode = Autonomous;
            resetWaypointIndex();
            setPropulsionOnOf(true);
            break;
        case 'h': // set King's beach as one and only waypoint, restart mission
            {
                resetWaypointIndex();
                NeoGPS::Location_t homeWaypoint(42.467600, -70.920050); // King's beach
                setCourseWaypointAtGivenGpsLocation(0, &homeWaypoint);
                break;
            }
        case 'I':
            tunePidKi(0.00025);
            break;
        case 'i':
            tunePidKi(-0.00025);
            break;
        case 'l': // adjust auto pilot bearing 10 degrees left
            adjustAutoPilotBearing(-10);
            break;
        case 'O':
            tunePidKp(0.25);
            break;
        case 'o':
            tunePidKp(-0.25);
            break;
        case 'p': // cycle through pilot modes
            roundtripDistanceMeters = 0;
            pilotMode = (PilotMode) (pilotMode == RemoteControlled ? Autonomous : pilotMode+1);
            Serial.println(pilotMode);
            break;
        case 'r': // adjust auto pilot bearing 10 deg right
            adjustAutoPilotBearing(10);
            break;
        case 's': // stop propulsion, do not reset waypoint
            setPropulsionOnOf(false);
            roundtripDistanceMeters = 0;
            break;
        case 'S': // toggle simulator on/off
            resetSimulationState(!getSimulationOnOffState());
            break;
        case 't': // cycle through roundtrip distances 100m..2000m in 100 increments
            {
                gps_fix* lastGoodFix = getLastGoodFix();
                if (!lastGoodFix->valid.location) {
                Serial.println("No fix");
                    btPrintln("No fix");
                    return;
                }

                roundtripDistanceMeters = roundtripDistanceMeters >= 2000 ? 100 : roundtripDistanceMeters + 100;
                Serial.println(roundtripDistanceMeters);
                btPrintln(roundtripDistanceMeters);

                setWaypointsForRoundtrip(roundtripDistanceMeters, &lastGoodFix->location);
                break;
            }
        case 'u':
            adjustPropulsionDesiredCruisePower(manualPulseWidthAdjustmentIncrement);
            break;
        case 'v': // adjust auto pilot bearing 1 deg left
            adjustAutoPilotBearing(-1);
            break;
        case 'x': // disable compass auto calibration, and make it store current calibration
            disableCompassAutoCalibration();
            saveCompassCalibrationProfile();
            break;
        case 'z':
            resetPidFactorsForTuning();
            break;
    }
}
#endif

void setPropulsionOnOf(bool propulsionOn) {
  isPropulsionOnOff = propulsionOn;
  setIsNextWaypointRealFlag(true);
  writeInt16ToEeprom(EEPROM_PROPULSION_STATE_ADDR, propulsionOn);
}

#ifndef REAL_MISSION
void handleDebugControls(unsigned long currentMillis) {
    if (touchRead(PIN_TOUCH_DISPLAY) < touchReadingThreshold) {
        if (currentMillis - lastMillisWhenDisplayControlWasNotTouched > timeToHavePinTouchedToConsiderActivatedMillis) {
            lastMillisWhenDisplayWasTurnedOn = currentMillis;
        }
    } else {
        lastMillisWhenDisplayControlWasNotTouched = currentMillis;
    }

    if (currentMillis - lastMillisWhenDisplayWasTurnedOn < keepDisplayOnTimeMillis) {
        turnDisplayOn();
    } else {
        turnDisplayOff();
    }
}
#endif