#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#include "comm_wifi.h"
#include "controls.h"
#include "secrets.h"
#include "telemetry.h"

#ifdef REAL_MISSION
#define DISABLE_WIFI
#endif

#ifndef DISABLE_WIFI

// define in secrets.h:
// #define WIFI_PSW "..."
// #define WIFI_SSID "..."
// #define TELEMETRY_API_KEY "..."
// #define TELEMETRY_API_URL "..."

#define TELEMETRY_API_HEADER "key"

enum WifiCommState {
  WifiCommOff,
  WifiCommTurningOn,
  WifiCommConnecting,
  WifiCommConnected,
  WifiCommDisconnecting,
  WifiCommTurningOff
};

WifiCommState wifiCommState = WifiCommOff;

const unsigned long wifiCommSleepBetweenSessionsMs = 5*60*1000L;
const unsigned long wifiCommDelayBetweenStepsMs = 1000L;

unsigned long previousWifiCommSessionStartedMs = 0;
unsigned long currentWifiCommSessionStartedMs = 0;

uint8_t currentWifiCommStepAttempt = 0;       // to keep track of attempts within some states
unsigned long lastWifiCommStepStartedMs = 0;  // to introduce delay between steps
const uint8_t maxWifiCommStepAttempts = 15;

// only for logging / debugging
#define WIFI_STATE_STORAGE_SIZE 4
uint8_t wifiStateStorage[WIFI_STATE_STORAGE_SIZE];

void saveLastCommState(WifiCommState wifiCommState);

void initWifi() {
}

int handleWifiComm(unsigned long currentMillis) {
  if (!checkIfTimeToDoNextWifiSessionStep(currentMillis)) {
    return wifiCommState;
  }
  
  lastWifiCommStepStartedMs = currentMillis;
  
  Serial.print(currentMillis);
  Serial.print(": Wifi in state ");
  Serial.println(wifiCommState);

  switch(wifiCommState) {
    case WifiCommOff:
      WiFi.mode(WIFI_STA);
      wifiCommState = WifiCommTurningOn;
      currentWifiCommSessionStartedMs = currentMillis;
      break;

    case WifiCommTurningOn:
      // WiFi.setAutoConnect(true);
      WiFi.begin(WIFI_SSID, WIFI_PSW);
      wifiCommState = WifiCommConnecting;
      currentWifiCommStepAttempt = 0;
      break;

    case WifiCommConnecting:
      if (++currentWifiCommStepAttempt > maxWifiCommStepAttempts) {
        Serial.println("Wifi connect aborted");
        wifiCommState = WifiCommDisconnecting;
      } else if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Wifi connected");
        wifiCommState = WifiCommConnected;
      }
      break;

    case WifiCommConnected:
      sendLastTelemetryViaWifi(getLastTelemetryDataInJson());
      wifiCommState = WifiCommDisconnecting;
      break;
    
    case WifiCommDisconnecting:
      WiFi.disconnect(true);
      wifiCommState = WifiCommTurningOff;
      break;

    case WifiCommTurningOff:
//      WiFi.mode(WIFI_MODE_NULL); // leads to a crash?
      wifiCommState = WifiCommOff;
      previousWifiCommSessionStartedMs = currentWifiCommSessionStartedMs;
      break;
  }

  saveLastCommState(wifiCommState);

  return wifiCommState;
}

void saveLastCommState(WifiCommState wifiCommState) {
  if (wifiStateStorage[WIFI_STATE_STORAGE_SIZE-1] == wifiCommState) {
    return;
  }

  for(int i = 0;  i< WIFI_STATE_STORAGE_SIZE - 1; i++) {
    wifiStateStorage[i] = wifiStateStorage[i + 1];
  }

  wifiStateStorage[WIFI_STATE_STORAGE_SIZE - 1] = wifiCommState;
}

void sendLastTelemetryViaWifi(char* jsonPayload) {
  HTTPClient http;
  
  // ESP32's default task watchdog is enabled for Arduino, if a high priority
  // main loop takes more than 5 seconds to run, the chip will be rebooted automatically.
  // occasionally WiFi / HTTP failures can do that, try to set client timeouts low
  // (see CONFIG_ESP_TASK_WDT_TIMEOUT_S defaulting to 5 seconds in ESP32 core stack)
  http.setTimeout(1000); // timeout for the TCP connection, ms
  http.setConnectTimeout(1000); // timeout (ms) for establishing a connection to the server

  Serial.println(http.begin(TELEMETRY_API_URL));
  http.addHeader("Content-Type", "application/json");
  http.addHeader(TELEMETRY_API_HEADER, TELEMETRY_API_KEY);
  Serial.println(jsonPayload);
  Serial.println(http.POST(jsonPayload));
  
  http.end();  
}

bool checkIfTimeToDoNextWifiSessionStep(unsigned long currentMillis) {
  if (currentMillis < previousWifiCommSessionStartedMs) {
    // if millis rolled over between sessions, reset previous start to current millis to wait another no-comm cycle
    previousWifiCommSessionStartedMs = currentMillis;
    return false;
  }

  if (currentMillis - previousWifiCommSessionStartedMs < wifiCommSleepBetweenSessionsMs) {
    return false; // sleep more between more sessions
  }

  if (currentMillis < lastWifiCommStepStartedMs) {
    // if millis rolled over between session steps, reset last step start to current and wait a bit more
    lastWifiCommStepStartedMs = currentMillis;
    return false;
  }
  
  if (currentMillis - lastWifiCommStepStartedMs < wifiCommDelayBetweenStepsMs) {
    return false; // sleep more between steps
  }

  return true;
}

#else

void initWifi() {
  WiFi.mode(WIFI_OFF);
}

int handleWifiComm(unsigned long currentMillis) {
  return 0;
}

#endif
