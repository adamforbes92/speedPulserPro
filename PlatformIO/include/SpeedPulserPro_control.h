#ifndef SPEEDPULSERPRO_CONTROL_H
#define SPEEDPULSERPRO_CONTROL_H

#include "Arduino.h"

// ===== Motor PWM (mirrors SpeedPulser) =====
// Hardware PWM runs at 12-bit (4096 levels) for fine low-speed granularity, while the
// calibration tables (motorPerformance[]) stay in their captured legacy 10-bit type.
// DUTY_SCALE_SHIFT bridges the two; PWM_DUTY_MAX is full-scale (100% = ~4.0 V).
#define PWM_RESOLUTION   12
#define CAL_RESOLUTION   10
#define DUTY_SCALE_SHIFT (PWM_RESOLUTION - CAL_RESOLUTION)
#define PWM_DUTY_MAX     ((1u << PWM_RESOLUTION) - 1)

// Input signal handlers
void incomingHz();
void incomingMotorSpeed();

// Closed-loop motor feedback (PID)
void feedbackPulse();                                       // motor feedback ISR
void resetPid();                                            // clear PID accumulators
float updateMeasuredFreq();                                 // return feedback Hz / measuredSpeed
int16_t applyFeedbackTrim(uint16_t targetSpeed, uint16_t baseDuty); // PID duty correction

// RPM output setup and frequency control
void setupTimer();
void setFrequencyRPM(long frequencyHz);

// Motor calibration lookup
uint16_t findClosestMatch(uint16_t val);
uint32_t speedToPwmDuty(uint16_t speedKph); // speed -> interpolated 12-bit hardware duty

// Speed offset helpers
void normaliseSpeedOffsetCurve();
int16_t getCurveOffsetForSpeed(uint16_t speedKph);
uint16_t applyConfiguredSpeedOffset(uint16_t speedKph);

// Filter buffer management
void resetHallMedianFilter();
void resetRPMMedianFilter();

#endif
