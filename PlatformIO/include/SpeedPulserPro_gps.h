#ifndef GPS_H
#define GPS_H

#include "SpeedPulserPro_config.h"


void parseGPS();
static void printFloat(float val, bool valid, int len, int prec);


// Persistent GPS update rate (Hz), default 1Hz
extern uint8_t gpsUpdateRateHz;

// Set GPS update rate (Hz: 1, 5, 10, 16). Returns true if sent, false if invalid.
bool setGPSUpdateRate(uint8_t rateHz, String &responseMsg);

// Get the current GPS update frequency (updates per second, float)
float getGPSUpdateFrequency();

#endif // GPS_H
