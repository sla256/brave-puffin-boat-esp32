#include "heartbeat.h"
#include "pins.h"

bool heartbeatSignalOn = false;

void initHeartbeat() {
    Serial.println("HEARTBEAT");
    pinMode(PIN_HEARTBEAT_SIGNAL, OUTPUT);
}

// alternates setting HIGH/LOW voltage on a dedicated pin
// should be called in the main loop, which cycles about once every 100ms
// the idea is for the external watchdog chip to follow this signal, and if it stops
// pulsating, reset this main ESP32 chip vis RST pin
void handleHeartbeat() {
    heartbeatSignalOn = !heartbeatSignalOn;
    digitalWrite(PIN_HEARTBEAT_SIGNAL, heartbeatSignalOn ? HIGH : LOW);
}