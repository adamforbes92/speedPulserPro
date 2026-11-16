#ifndef SPEEDPULSERPRO_CONTROL_H
#define SPEEDPULSERPRO_CONTROL_H

#include "Arduino.h"
#include "SpeedPulserPro_board.h"

// ===== Motor PWM (mirrors SpeedPulser) =====
// Hardware PWM runs at 12-bit (4096 levels) for fine low-speed granularity, while the
// calibration tables (presetSpeedTable[]) stay in their captured legacy 10-bit type.
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

// ===== Speed sensor inputs (V5.06, mirrors SpeedPulser V5.06) =====
// Hall (and VR on the S3): each interrupt keeps its pulse times; the speed task
// averages them over inputSmoothingMs. Speed (km/h) = Hz x 3600 / pulses per km.
#define INPUT_TIMEOUT_MS 1500          // no pulse for this long = stopped...
#define INPUT_TIMEOUT_MAX_MS 5000      // ...longer for a slow sensor (V5.09), up to this:
#define INPUT_TIMEOUT_SLOW_KPH 10.0f   //    INPUT_TIMEOUT_SLOW_PERIODS pulse periods at this speed
#define INPUT_TIMEOUT_SLOW_PERIODS 2.5f
#define KPH_TO_MPH 0.621371f
void incomingHz();                     // hall speed interrupt
float measureHallFrequency();          // hall input (Hz), averaged; 0 = stopped
#if SPP_HAS_VR
void incomingVR();                     // VR speed interrupt (S3)
float measureVRFrequency();            // VR input (Hz), averaged; 0 = stopped
#endif
void updateInputGlitchFilters();       // call when a pulses-per-km setting or maxSpeed changes
uint16_t toNeedleTarget(float speedKph); // km/h -> offset -> dial units (what the needle shows)

// Engine RPM input (windowed): read-and-clear, Hz or <0 when no fresh edges arrived.
void incomingMotorSpeed();
float readRPMHz();
void resetRPMPulseCounter();

// Motor tacho (S3 board)
void feedbackPulse();                  // motor tacho interrupt
float updateMeasuredFreq();            // measure + publish the tacho frequency (Hz), 100 ms cadence
bool tachoSignalLost();                // S3: the tacho went silent while the motor should turn (feedback dropped)
float speedToTachoHz(float speed);     // tacho Hz expected at a dial speed (drive table; 0 without one)

// RPM output setup and frequency control
void setupTimer();
void setFrequencyRPM(float frequencyHz);

// Motor calibration lookup
uint16_t findClosestMatch(uint16_t val);
uint32_t speedToPwmDuty(uint16_t speedKph); // speed -> interpolated 12-bit hardware duty

#if SPP_HAS_VOLTAGE_CTRL
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
extern float driveTrim;                   // feedback trim on the calibrated drive (fraction, ±trimLimit)
#endif

// RPM filter buffer
void resetRPMMedianFilter();

#endif
