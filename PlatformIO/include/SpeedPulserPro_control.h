#ifndef SPEEDPULSERPRO_CONTROL_H
#define SPEEDPULSERPRO_CONTROL_H

#include "Arduino.h"
#include "SpeedPulserPro_board.h"

// ===== Motor PWM (mirrors SpeedPulser) =====
// Hardware PWM runs at 12-bit (4096 levels) for fine low-speed granularity, while the
// calibration tables (motorPerformance[]) stay in their captured legacy 10-bit type.
// DUTY_SCALE_SHIFT bridges the two; PWM_DUTY_MAX is full-scale (100% = ~4.0 V).
#define PWM_RESOLUTION   12
#define CAL_RESOLUTION   10
#define DUTY_SCALE_SHIFT (PWM_RESOLUTION - CAL_RESOLUTION)
#define PWM_DUTY_MAX     ((1u << PWM_RESOLUTION) - 1)

#if SPP_HAS_VOLTAGE_CTRL
// ===== V_ADJ "DAC" (buck voltage control, S3 board) =====
// A separate LEDC timer/channel drives V_ADJ as a PWM->RC analogue setpoint into
// the buck FB node. CH0/TIMER0 = motor (see globals.h); the RPM output is a
// GPTimer square wave, not LEDC. V_ADJ keeps CH2/TIMER2 so it never collides with
// anything else. 20 kHz sits far above the FB injection RC corners, so the
// residual ripple on the motor rail is negligible.
#define LEDC_CHANNEL_VADJ  LEDC_CHANNEL_2
#define LEDC_TIMER_VADJ    LEDC_TIMER_2
#define VADJ_PWM_FREQUENCY 20000  // Hz
#define VADJ_RESOLUTION    10     // bits (0..1023) — smooth enough for an analogue voltage setpoint
#define VADJ_DUTY_MAX      ((1u << VADJ_RESOLUTION) - 1)
#endif

// Input signal handlers
void incomingHz();
void incomingMotorSpeed();

// Windowed input-frequency readers — read-and-clear the ISR accumulators and return
// the averaged frequency in Hz, or <0 when no fresh edges arrived (hold last value).
float readHallHz();
float readRPMHz();
void resetHallPulseCounter(); // full reset (incl. edge reference) on timeout/test transitions
void resetRPMPulseCounter();

#if SPP_HAS_VR
// Variable-reluctance speed input (S3 board only) — counted like the hall input
void incomingVR();
float readVRHz();
void resetVRPulseCounter();
#endif

// Closed-loop motor feedback (PID)
void feedbackPulse();                                       // motor feedback ISR
void resetPid();                                            // clear PID accumulators
float updateMeasuredFreq();                                 // measure + publish motor tacho frequency (Hz) — 100 ms cadence
int16_t applyFeedbackTrim(uint16_t targetSpeed, uint16_t baseDuty); // PID duty correction (uses this tick's measurement)
float speedToTachoHz(float speed);                          // tacho Hz expected at a dial speed (drive table, else legacy linear)

// RPM output setup and frequency control
void setupTimer();
void setFrequencyRPM(float frequencyHz);

// Motor calibration lookup
uint16_t findClosestMatch(uint16_t val);
uint32_t speedToPwmDuty(uint16_t speedKph); // speed -> interpolated 12-bit hardware duty

#if SPP_HAS_VOLTAGE_CTRL
void syncFeedbackMaxFreq();               // re-derive feedbackMaxFreq (Hz at maxSpeed) from the drive table
// Last ~1 s of tacho: mean, spread ((max-min)/mean, %) and drift (newer half-second
// mean minus older half-second mean, Hz).
bool feedbackHzWindow(float &meanHz, float &spreadPct, float &driftHz);
// "Steady" = the MEAN has stopped moving. Cogging at low speed is a wobble around a
// constant mean (e.g. 23.1-24.7 Hz, ~7 % p-p) and averages out of each half, so it
// shows as spread but not drift; a motor still settling after a drive change shows
// as drift. The spread limit only rejects a motor that is stalling / stuttering.
#define CAL_MAX_DRIFT_PCT   2.0f          // max |drift| as % of the mean...
#define CAL_MAX_DRIFT_HZ    0.75f         // ...or in Hz, whichever is looser (low speeds)
#define CAL_MAX_SPREAD_PCT  20.0f         // max (max-min)/mean...
#define CAL_MAX_SPREAD_HZ   4.0f          // ...or in Hz, whichever is looser
inline bool feedbackHzSteady(float meanHz, float spreadPct, float driftHz)
{
  const float spreadHz = spreadPct * meanHz / 100.0f;
  const bool driftOk = fabsf(driftHz) <= CAL_MAX_DRIFT_HZ ||
                       fabsf(driftHz) * 100.0f / meanHz <= CAL_MAX_DRIFT_PCT;
  const bool spreadOk = spreadHz <= CAL_MAX_SPREAD_HZ || spreadPct <= CAL_MAX_SPREAD_PCT;
  return driftOk && spreadOk;
}

// S3 board: open-loop drive from the calibration table + optional slow feedback trim
uint16_t applyDriveControl(uint16_t targetSpeed); // returns the raw PWM duty applied
void resetDriveControl();                 // drop the feedback trim
extern float driveTrim;                   // feedback trim on the calibrated drive (fraction, ±vcTrimLimit)
#endif

// Speed offset helpers
void normaliseSpeedOffsetCurve();
int16_t getCurveOffsetForSpeed(uint16_t speedKph);
uint16_t applyConfiguredSpeedOffset(uint16_t speedKph);

// Filter buffer management
void resetHallMedianFilter();
void resetRPMMedianFilter();

#endif
