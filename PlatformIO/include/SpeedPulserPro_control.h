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

// ===== V_ADJ "DAC" (buck voltage control, V4 board) =====
// A separate LEDC timer/channel drives GPIO33 as a PWM->RC analogue setpoint into
// the buck FB node. CH0/TIMER0 = motor and CH1/TIMER1 = RPM (see globals.h), so
// V_ADJ takes CH2/TIMER2. 20 kHz sits far above the FB injection RC corner, so the
// residual ripple on the motor rail is negligible.
#define LEDC_CHANNEL_VADJ  LEDC_CHANNEL_2
#define LEDC_TIMER_VADJ    LEDC_TIMER_2
#define VADJ_PWM_FREQUENCY 20000  // Hz
#define VADJ_RESOLUTION    10     // bits (0..1023) — smooth enough for an analogue voltage setpoint
#define VADJ_DUTY_MAX      ((1u << VADJ_RESOLUTION) - 1)

// Input signal handlers
void incomingHz();
void incomingMotorSpeed();
void incomingVR(); // variable-reluctance speed input (counted like the hall input)

// Windowed input-frequency readers — read-and-clear the ISR accumulators and return
// the averaged frequency in Hz, or <0 when no fresh edges arrived (hold last value).
float readHallHz();
float readVRHz();
float readRPMHz();
void resetHallPulseCounter(); // full reset (incl. edge reference) on timeout/test transitions
void resetVRPulseCounter();
void resetRPMPulseCounter();

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

// V4 mid-ranging voltage control (fast PWM tracks RPM, slow loop schedules volts)
int16_t applyMidRangingControl(uint16_t targetSpeed); // returns 12-bit motor duty
void resetMidRanging();                                // clear mid-ranging PID + voltage-trim state

// Speed offset helpers
void normaliseSpeedOffsetCurve();
int16_t getCurveOffsetForSpeed(uint16_t speedKph);
uint16_t applyConfiguredSpeedOffset(uint16_t speedKph);

// Filter buffer management
void resetHallMedianFilter();
void resetRPMMedianFilter();

#endif
