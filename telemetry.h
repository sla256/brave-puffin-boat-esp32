#include <Arduino.h>
#include "telemetry_record.h"

void initTelemetry();
void handleTelemetry();
int16_t currentRunNumber();
void startTelemetryRecord();
void addTelemetryDataFloat(const char *prefix, float value);
void addTelemetryDataInt32_t(const char *prefix, int32_t value);
void addTelemetryDataString(const char *prefix, const char *value);
void saveTelemetry();
char* getLastTelemetryDataInJson();
unsigned long getConsistentMillis();
bool isTimeToSaveTelemetry();

// allows to use the same millis across all components, set at the beginning of a given loop() iteration
void setConsistentMillis(unsigned long currentMillis);

unsigned long getLastSaveTelemetryMillis();

TelemetryRecord* getCurrentTelemetryRecord();