#define gpsPort gpsSerial
#define GPS_FIX_DATE
#define GPS_PORT_NAME "gpsSerial"
#define DEBUG_PORT Serial

#include <NMEAGPS.h>

void initGps(void (*workHandlerPtr)(gps_fix *lastGoodFix));
void gpsLoop();
gps_fix *getLastGoodFix();
bool isGpsAvailable();