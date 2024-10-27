#define USE_SD_CARD

#ifdef USE_SD_CARD
#include <SD.h>
#endif

void initSd();
void saveLogRecordToSdCard(char *data);