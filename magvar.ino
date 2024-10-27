#include <Arduino.h>

// These are arrays of magnetic variation variables aligned to a coarse grid of lat/long areas in North Atlantic.
// First array covers latitudes from 30 (inclusive) to 39 (inclusive) degrees, second is 40-49, third is 50-59.
// Each column represents magnetic variation in the given latitude range and 1 degree of western longitude,
// from -79 (inclusive, Florida's east coast) to -4 (inclusive, Glasgow) degrees.
// All of the magnetic variation values are negative because North Atlantic happens to have western magnetic declination.
// Produced by re-processing https://github.com/jafrado/magdec/blob/master/data/igrfgridData-World-2019.csv
// which in turn was extracted from the world magnetic model https://www.ngdc.noaa.gov/geomag/WMM/
// Stored in PROGMEM to cut RAM usage (~0.5K data)

#define MAG_VAR_GRID_LONG_START  -79  // western most longitude covered
#define MAG_VAR_GRID_LONG_END    -4   // eastern most
#define COUNT_OF_LONGITUDE_VARS   (MAG_VAR_GRID_LONG_END - MAG_VAR_GRID_LONG_START + 1)

#define MAG_VAR_GRID_LAT_START  30 // 30 degrees N is the southern-most latitude covered
#define MAG_VAR_GRID_LAT_END    59 // 59 degrees N is the northern-most latitude covered

// for latitudes [30-39] approx. NYC to Jackonsville, FL
const PROGMEM int8_t magVars1 [COUNT_OF_LONGITUDE_VARS] = 
  {-8,-9,-9,-10,-11,-11,-12,-12,-12,-13,    -13,-14,-14,-14,-14,-15,-15,-15,-15,-15,  -15,-15,-15,-15,-15,-15,-15,-15,-15,-15,  -15,-14,-14,-14,-14,-14,-13,-13,-13,-12,  -12,-12,-12,-11,-11,-11,-10,-10,-10,-9,   -9,-9,-8,-8,-7,-7,-7,-6,-6,-6,        -5,-5,-4,-4,-4,-3,-3,-3,-2,-2,  -2,-1,-1,-1,0,0};

// for latitudes [40-49] approx north of Quebec to NYC
const PROGMEM int8_t magVars2 [COUNT_OF_LONGITUDE_VARS] = 
  {-10,-11,-12,-12,-13,-13,-14,-14,-15,-15, -16,-16,-16,-16,-17,-17,-17,-17,-17,-17,  -17,-17,-17,-17,-17,-17,-17,-17,-16,-16,  -16,-16,-16,-15,-15,-15,-14,-14,-14,-14,  -13,-13,-13,-12,-12,-12,-11,-11,-10,-10,  -10,-9,-9,-8,-8,-8,-7,-7,-7,-6,       -6,-5,-5,-5,-4,-4,-3,-3,-3,-2,  -2,-2,-1,-1,-1,0};

// [50-59] approximately south most to north most tips of UK
const PROGMEM int8_t magVars3 [COUNT_OF_LONGITUDE_VARS] = 
  {-14,-15,-16,-16,-17,-18,-18,-19,-19,-19, -20,-20,-20,-20,-21,-21,-21,-21,-21,-21,  -21,-21,-21,-21,-21,-20,-20,-20,-20,-20,  -20,-19,-19,-19,-19,-18,-18,-18,-17,-17,  -16,-16,-16,-15,-15,-14,-14,-14,-13,-13,  -12,-12,-11,-11,-11,-10,-10,-9,-9,-8, -8,-8,-7,-7,-6,-6,-5,-5,-5,-4,  -4,-3,-3,-2,-2,-2};


// cache to avoid PROGMEM extraction & array index computation
int lastQueriedLat = 0;
int lastQueriedLong = 0;
int8_t lastExtractedMagVar = -14; // default to Boston's declination

// lat and long should be rounded, degrees only, signed whole numbers e.g. 42,-71
int8_t getMagVarByLatLong(int latitude, int longitude) {
  if (lastQueriedLat == latitude  &&  lastQueriedLong == longitude) {
    return lastExtractedMagVar;
  }

  // limit to the max/min supported coordinates
  latitude = latitude > MAG_VAR_GRID_LAT_END  ? MAG_VAR_GRID_LAT_END : latitude;
  latitude = latitude < MAG_VAR_GRID_LAT_START ? MAG_VAR_GRID_LAT_START : latitude;
  longitude = longitude > MAG_VAR_GRID_LONG_END ? MAG_VAR_GRID_LONG_END : longitude;
  longitude = longitude < MAG_VAR_GRID_LONG_START ?  MAG_VAR_GRID_LONG_START : longitude;

  lastQueriedLat = latitude;
  lastQueriedLong = longitude;

  // -79 is zero, -78 is one, etc. -4 corresponds to tjhe ...? confefe?
  int longitudeArrayIndex = longitude - MAG_VAR_GRID_LONG_START;
  int8_t magVar;
  
  if (latitude < MAG_VAR_GRID_LAT_START + 10) {
    // must be in [30-39] range
    magVar = pgm_read_byte_near(magVars1+longitudeArrayIndex);
  } else if (latitude > MAG_VAR_GRID_LAT_END - 10) {
    // must be in [50-59] range
    magVar = pgm_read_byte_near(magVars3+longitudeArrayIndex);
  } else {
    // must be in [40-49] range
    magVar = pgm_read_byte_near(magVars2+longitudeArrayIndex);
  }

  lastExtractedMagVar = magVar;

  return magVar;
}
