// 2024 battery pack is 72 li-ion Samsung INR18650-33G cells, 24 parallel in 3 series
// Per https://lygte-info.dk/review/batteries2012/Samsung%20INR18650-33G%203150mAh%20(Blue)%20UK.html 
// cell's measured energy is ~11.3 Wh at a small discharge current.
// Means 2024 pack's total maximum energy is 72 * 11.3 = 813 Wh
// Depleting the pack to 25% charge to 10.5V means 813 x 0.75 = 610 Wh energy to recover
// during the day. 2024 solar panel is 175 W nominally, in practice produces 130 W at peak, and a lot less on average.
// Let's assume 70W average during the day; 610 Wh / 70 W = 8.7 hours of charging
// This is a conservative estimate: 610 Wh is quite a bit of power, allowing 50 W for 12 hours.
// That said overcast days may result in less than fully charged battery, which in turn can lead to
// full depletion overnight, forcing the stop of propulsion

#define BAT_VOLTAGE_INSUFFICIENT_FOR_PROPULSION_MV  10400 // 10.5V ~75% depleted; propulsion will be cut off when reached
#define BAT_VOLTAGE_SUFFICIENT_FOR_PROPULSION_MV    10800 // used for upper boundary cut off mode; propulsion will resume when this is reached

void handlePower();

int getLastBatteryDrawCurrentMa();
int getLastSolarChargingCurrentMa();

int getLastBatteryVoltageMv();
int getLastSolarChargingVoltageMv();

int getLastBatterDrawPowerW();