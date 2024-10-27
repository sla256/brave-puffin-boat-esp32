#include <Arduino.h>
#include "debug.h"
#include "watchdog.h"

int forcedChipRestartBeginMillis;
int forcedRestartTriggerThresholdMs;

// ESP's built-in task watchdog for some reason stopped rebooting the chip in the panic handler...
// So now relying on the external watchdog, and periodic programmic restarts
// 
// https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/main.cpp
// https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-misc.c
// https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/system/wdts.html
// TWDT timeout is 5 seconds by default
// important to enable after all of setup is done - otherwise can crash in a loop during setup
// Serial.println(esp_task_wdt_deinit());
// esp_task_wdt_config_t twdt_config = {
//       .timeout_ms = esp32TaskWatchdogTimeoutSec*1000,
//       .idle_core_mask = 3, 
//       .trigger_panic = true
//   };
// Serial.println(esp_task_wdt_reconfigure(&twdt_config));


void initRestarter(int secondsToRestart) {
    Serial.println("WATCHDOG");
    disableLoopWDT();
    forcedChipRestartBeginMillis = millis();
    forcedRestartTriggerThresholdMs =  secondsToRestart*1000;
}

void handleForcedRestart() {
    if ((millis() - forcedChipRestartBeginMillis) < forcedRestartTriggerThresholdMs) return;

    ESP.restart();
}