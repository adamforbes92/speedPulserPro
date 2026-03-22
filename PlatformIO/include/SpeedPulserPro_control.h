#ifndef SPEEDPULSERPRO_CONTROL_H
#define SPEEDPULSERPRO_CONTROL_H

#include "Arduino.h"

// Input signal handlers
void incomingHz();
void incomingMotorSpeed();

// RPM output setup and frequency control
void setupTimer();
void setFrequencyRPM(long frequencyHz);

// Motor calibration lookup
uint16_t findClosestMatch(uint16_t val);

// Speed offset helpers
void normaliseSpeedOffsetCurve();
int16_t getCurveOffsetForSpeed(uint16_t speedKph);
uint16_t applyConfiguredSpeedOffset(uint16_t speedKph);

// Filter buffer management
void resetHallMedianFilter();
void resetRPMMedianFilter();

#endif
