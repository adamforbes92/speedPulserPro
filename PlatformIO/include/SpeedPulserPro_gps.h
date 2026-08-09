#ifndef SPEEDPULSERPRO_GPS_H
#define SPEEDPULSERPRO_GPS_H

#include "SpeedPulserPro_config.h"


void parseGPS();
static void printFloat(float val, bool valid, int len, int prec);

// Initialize GPS serial at default baud on startup (9600). Returns selected baud.
unsigned long initGPS();

// GPS update rate (Hz), default 1Hz
extern uint8_t gpsUpdateRateHz;

// Set GPS update rate (Hz: 1, 5, 10, 16). Returns true if sent, false if invalid.
bool setGPSUpdateRate(uint8_t rateHz, String &responseMsg);

// Returns true (and clears the flag) when parseGPS has queued an auto rate apply.
bool gpsAutoRateApplyPending();

// Get the current GPS update frequency (updates per second, float)
float getGPSUpdateFrequency();

// Seconds remaining until the auto rate apply fires.
// -1 = no pending auto-apply, 0 = ready/firing, >0 = countdown.
int gpsAutoApplySecondsRemaining();

#endif // SPEEDPULSERPRO_GPS_H
