#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_motorCal.h"
#include "SpeedPulserPro_calBuilder.h"
#include "SpeedPulserPro_voltage.h"
#include "SpeedPulserPro_driveCal.h"
#include "Arduino.h"
#include "hal/gpio_ll.h"

void normaliseSpeedOffsetCurve()
{
  for (uint8_t index = 0; index < SPEED_OFFSET_CURVE_POINTS; index++)
  {
    speedOffsetCurveOffsets[index] = constrain(speedOffsetCurveOffsets[index], -20, 20);
  }
}

int16_t getCurveOffsetForSpeed(uint16_t speedKph)
{
  if (speedKph <= 50)
  {
    return speedOffsetCurveOffsets[0];
  }
  if (speedKph <= 100)
  {
    return speedOffsetCurveOffsets[1];
  }
  if (speedKph <= 150)
  {
    return speedOffsetCurveOffsets[2];
  }
  if (speedKph <= 200)
  {
    return speedOffsetCurveOffsets[3];
  }
  return speedOffsetCurveOffsets[4];
}

uint16_t applyConfiguredSpeedOffset(uint16_t speedKph)
{
  normaliseSpeedOffsetCurve();

  int32_t correctedSpeed = (int32_t)speedKph;
  int16_t offsetToApply = 0;

  if (useSpeedOffsetCurve)
  {
    offsetToApply = getCurveOffsetForSpeed(speedKph);
  }
  else if (useGlobalSpeedOffset)
  {
    offsetToApply = speedOffsetPositive ? (int16_t)speedOffset : -(int16_t)speedOffset;
  }

  currentSpeedOffset = offsetToApply;
  correctedSpeed += offsetToApply;

  if (correctedSpeed < 0)
  {
    correctedSpeed = 0;
  }

  if (correctedSpeed > 400)
  {
    correctedSpeed = 400;
  }

  return (uint16_t)correctedSpeed;
}

void resetHallMedianFilter()
{
  rawCount = 0;
  samples.clear();
}

void resetRPMMedianFilter()
{
  rawCountRPM = 0;
  filteredRPMHz = 0.0f;
  samplesRPM.clear();
}

// File-scope (not a function-local static): a function-local static with a
// runtime initializer would emit a __cxa_guard_acquire on first use, which takes
// a FreeRTOS mutex with a timeout — illegal in an ISR and asserts in queue.c.
//
// Windowed frequency capture — shared by the hall, VR and RPM inputs. Deriving Hz
// from a single edge-to-edge period turns tone-wheel / tooth-spacing jitter straight
// into a jumpy reading, so instead each ISR accumulates the summed edge intervals
// and the interval count. The task reads-and-clears the pair once per window, so
// freq = count / summedInterval is a true average over the window — exactly how the
// closed-loop tacho (feedbackPulse) already works.
struct PulseWindow
{
  volatile uint32_t accumUs;    // summed edge-to-edge intervals (us) this window
  volatile uint32_t count;      // number of intervals summed into accumUs
  volatile uint32_t lastEdgeUs; // micros() of the previous accepted edge (kept across windows)
  volatile uint32_t minIntervalUs; // edges closer than this are rejected (see pulseMinIntervalUs)
};

// Glitch rejection. Ignition-coil EMI couples in as sub-millisecond bursts, so edges
// closer together than half a period at the input's configured full scale (Max Hall
// Frequency / Max VR Frequency / Cluster Frequency Limit) are dropped. Up to V5.01 this
// was a fixed 3 ms (333 Hz) for every input, and since a rejected edge keeps the last
// good edge, a genuine input above 333 Hz (6/8-cyl RPM near redline) read HALF its
// frequency. Half a period at full scale means anything rejected is above 2x full
// scale, and the doubled interval it leaves still reads >= full scale, so the output
// sits at its ceiling instead of collapsing.
static const uint32_t PULSE_MIN_INTERVAL_DEFAULT_US = 3000; // until the first read sets it
static const uint32_t PULSE_MIN_INTERVAL_FLOOR_US = 1000;   // never accept edges above 1 kHz

static PulseWindow hallPulse = {0, 0, 0, PULSE_MIN_INTERVAL_DEFAULT_US}; // vehicle hall speed input
#if SPP_HAS_VR
static PulseWindow vrPulse = {0, 0, 0, PULSE_MIN_INTERVAL_DEFAULT_US};   // variable-reluctance speed input (S3 board)
#endif
static PulseWindow rpmPulse = {0, 0, 0, PULSE_MIN_INTERVAL_DEFAULT_US};  // engine RPM input

static uint32_t pulseMinIntervalUs(uint16_t fullScaleHz)
{
  if (fullScaleHz == 0)
    return PULSE_MIN_INTERVAL_DEFAULT_US;
  uint32_t us = 500000UL / fullScaleHz; // half a period at full scale
  return us < PULSE_MIN_INTERVAL_FLOOR_US ? PULSE_MIN_INTERVAL_FLOOR_US : us;
}

// Guards the read-and-clear of the pulse windows against the ISRs. portMUX is the
// FreeRTOS/ESP32 safe primitive; a global noInterrupts() would starve the WiFi radio.
static portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;

// Accumulate one edge into a window. Returns true when the edge was accepted (it
// advanced the average), false when it merely seeded the window or was rejected as
// a glitch/bounce. lastEdgeUs is kept on a glitch so the next real edge still
// measures from the last good edge.
static inline bool IRAM_ATTR pulseEdge(PulseWindow &w, uint32_t now)
{
  uint32_t last = w.lastEdgeUs;
  if (last == 0)
  {
    w.lastEdgeUs = now; // seed on the first pulse only
    return false;
  }
  uint32_t interval = now - last;
  if (interval < w.minIntervalUs)
    return false; // coil EMI / bounce: ignore, keep the last good edge
  w.accumUs += interval;
  w.count++;
  w.lastEdgeUs = now;
  return true;
}

// Read-and-clear a window, returning its averaged frequency in Hz, or -1 when no
// fresh edges arrived (caller should hold the previous value). lastEdgeUs is left
// intact so the next window's first interval bridges from this window's last edge.
// The glitch threshold is refreshed from the live full-scale setting on every read,
// so a slider change takes effect within one window.
static float readPulseHz(PulseWindow &w, uint16_t fullScaleHz)
{
  w.minIntervalUs = pulseMinIntervalUs(fullScaleHz);
  portENTER_CRITICAL(&pulseMux);
  uint32_t count = w.count;
  uint32_t accum = w.accumUs;
  w.count = 0;
  w.accumUs = 0;
  portEXIT_CRITICAL(&pulseMux);
  if (count >= 1 && accum > 0)
    return (float)count * 1000000.0f / (float)accum;
  return -1.0f;
}

// Full reset (including the edge reference) — used on timeout / test transitions so
// a stale lastEdgeUs can't inject one bogus huge-interval reading when input resumes.
static void resetPulseWindow(PulseWindow &w)
{
  portENTER_CRITICAL(&pulseMux);
  w.accumUs = 0;
  w.count = 0;
  w.lastEdgeUs = 0;
  portEXIT_CRITICAL(&pulseMux);
}

float readHallHz() { return readPulseHz(hallPulse, maxFreqHall); }
float readRPMHz() { return readPulseHz(rpmPulse, maxRPM); }
void resetHallPulseCounter() { resetPulseWindow(hallPulse); }
void resetRPMPulseCounter() { resetPulseWindow(rpmPulse); }
#if SPP_HAS_VR
float readVRHz() { return readPulseHz(vrPulse, maxFreqVR); }
void resetVRPulseCounter() { resetPulseWindow(vrPulse); }
#endif

// Interrupt handler for the vehicle hall speed input.
// IRAM_ATTR is required so a pulse arriving while the flash cache is disabled
// (EEPROM/LittleFS write) doesn't crash the chip.
void IRAM_ATTR incomingHz()
{
  // Ignore the vehicle hall input entirely while bench-testing or calibrating —
  // the motor is driven from the Speed Test / Cal controls, so an incoming signal
  // must not touch the counter, lastPulse or the LED counter until the test
  // (or cal) is turned off.
  if (testSpeedo || testCal)
    return;
  if (pulseEdge(hallPulse, micros()))
  {
    lastPulse = millis();
    ledCounter++;
  }
}

#if SPP_HAS_VR
// Interrupt handler for the variable-reluctance (VR) speed input. Counted exactly
// like the hall input — one clean digital edge per zero-crossing from the VR
// conditioner (C Out) — feeding the same windowed period->frequency->speed pipeline.
// IRAM_ATTR is required so a pulse arriving while the flash cache is disabled
// (EEPROM/LittleFS write) doesn't crash the chip.
void IRAM_ATTR incomingVR()
{
  // Ignore the VR input while bench-testing or calibrating (same rule as the hall).
  if (testSpeedo || testCal)
    return;
  if (pulseEdge(vrPulse, micros()))
    lastPulseVR = millis();
}
#endif // SPP_HAS_VR

// Interrupt handler for the engine RPM input.
// IRAM_ATTR is required so a pulse arriving while the flash cache is disabled
// (EEPROM/LittleFS write) doesn't crash the chip.
void IRAM_ATTR incomingMotorSpeed()
{
  if (pulseEdge(rpmPulse, micros()))
    lastPulseRPM = millis();
}

// ============================================================================
// Closed-Loop Motor Feedback (PID)
// ============================================================================
// The motor's own tacho pulse train (pinMotorFeedback) is measured period-based:
// we accumulate the exact micros() interval between edges as well as the edge
// count, so freq = edges / (summed interval time) has continuous resolution and
// averages out per-commutation jitter over the sample window.
// IRAM_ATTR is required so a pulse arriving while the flash cache is disabled
// (EEPROM/LittleFS write) doesn't crash the chip.
static volatile uint32_t feedbackAccumUs = 0;    // summed edge-to-edge intervals (us)
static volatile uint32_t feedbackLastEdgeUs = 0; // micros() of the previous edge
void IRAM_ATTR feedbackPulse()
{
  uint32_t now = micros();
  uint32_t last = feedbackLastEdgeUs;
  feedbackLastEdgeUs = now;
  if (last != 0)
  {
    feedbackAccumUs = feedbackAccumUs + (now - last);
  }
  feedbackCount = feedbackCount + 1;
}

// Full-scale hardware PWM duty — the feedback loop works in the 12-bit domain.
static inline uint16_t fbDutyMax()
{
  return (uint16_t)PWM_DUTY_MAX; // 4095
}

static float pidIntegral = 0.0f;
static float pidPrevError = 0.0f; // previous error, tacho Hz domain
static float measFreqFilt = 0.0f; // EMA-smoothed tacho frequency

// Guards the read-and-clear of the ISR accumulators. portMUX is the FreeRTOS/ESP32
// safe primitive; a global noInterrupts() would starve the WiFi radio.
static portMUX_TYPE feedbackMux = portMUX_INITIALIZER_UNLOCKED;

void resetPid()
{
  pidIntegral = 0.0f;
  pidPrevError = 0.0f;
  measFreqFilt = 0.0f;
  pidCorrection = 0;
}

// ===== Speed <-> tacho scale =====
// S3 board with a drive calibration: the captured marks map dial speed <-> tacho
// Hz directly (piecewise, so a dial that isn't linear near zero still reads
// right). Otherwise (DevKit, Voltage Control off, or no tacho in the marks) the
// legacy single scale: feedbackMaxFreq Hz at maxSpeed, straight through zero.
#if SPP_HAS_VOLTAGE_CTRL
static bool useDriveTable()
{
  return voltageControlEnable && driveCalHzValid();
}
#endif

float speedToTachoHz(float speed)
{
#if SPP_HAS_VOLTAGE_CTRL
  if (useDriveTable())
    return hzForSpeed(speed);
#endif
  const uint16_t scaleFreq = feedbackMaxFreq > 0 ? feedbackMaxFreq : 1;
  const uint16_t spdSpan = maxSpeed > 0 ? maxSpeed : 1;
  return speed * (float)scaleFreq / (float)spdSpan;
}

static float tachoHzToSpeed(float hz)
{
#if SPP_HAS_VOLTAGE_CTRL
  if (useDriveTable())
    return speedForHz(hz);
#endif
  const uint16_t scaleFreq = feedbackMaxFreq > 0 ? feedbackMaxFreq : 1;
  const uint16_t spdSpan = maxSpeed > 0 ? maxSpeed : 1;
  return hz * (float)spdSpan / (float)scaleFreq;
}

#if SPP_HAS_VOLTAGE_CTRL
// Keep the legacy feedbackMaxFreq (Hz at maxSpeed) in step with the drive table.
// It still normalises the legacy PID gains, so call after a capture, at boot and
// whenever maxSpeed changes.
void syncFeedbackMaxFreq()
{
  if (!driveCalHzValid())
    return;
  long hz = lroundf(hzForSpeed((float)(maxSpeed > 0 ? maxSpeed : 1)));
  feedbackMaxFreq = (uint16_t)constrain(hz, 1L, 2000L);
}

// ===== Capture window =====
// The last ~1 s of RAW tacho readings (one per 100 ms measurement). The capture
// averages over it and refuses a reading that is still moving, so a mark isn't
// taken mid-transition just after the drive was nudged.
#define FB_RING 10
static float fbRing[FB_RING] = {0};
static uint8_t fbRingIdx = 0;
static uint8_t fbRingCount = 0;

static void pushFeedbackSample(float hz)
{
  fbRing[fbRingIdx] = hz;
  fbRingIdx = (fbRingIdx + 1) % FB_RING;
  if (fbRingCount < FB_RING)
    fbRingCount++;
}

// Mean of the window, its spread ((max-min)/mean, %) and its drift (newer half
// mean - older half mean, Hz). False until the window is full or when the motor
// isn't turning.
bool feedbackHzWindow(float &meanHz, float &spreadPct, float &driftHz)
{
  meanHz = 0.0f;
  spreadPct = 0.0f;
  driftHz = 0.0f;
  if (fbRingCount < FB_RING)
    return false;
  float sum = 0.0f, older = 0.0f, lo = fbRing[0], hi = fbRing[0];
  for (uint8_t k = 0; k < FB_RING; k++)
  {
    const float v = fbRing[(fbRingIdx + k) % FB_RING]; // k = 0 is the oldest
    sum += v;
    if (k < FB_RING / 2) older += v;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  meanHz = sum / (float)FB_RING;
  if (meanHz < 1.0f)
    return false;
  spreadPct = (hi - lo) * 100.0f / meanHz;
  const float half = (float)(FB_RING / 2);
  driftHz = (sum - older) / half - older / half;
  return true;
}
#endif // SPP_HAS_VOLTAGE_CTRL

// Period-based tacho frequency (Hz), EMA-smoothed, published to measuredFreqHz /
// measuredSpeed. Called once per 100 ms by the speed task in EVERY mode, so the
// read-outs, the capture window and the feedback loops all share one fresh
// measurement.
float updateMeasuredFreq()
{
  const uint32_t FB_STALL_US = 300000UL; // no edge for 300 ms -> stopped
  const float FB_MEAS_ALPHA = 0.4f;      // EMA weight for the measured-Hz low-pass

  portENTER_CRITICAL(&feedbackMux);
  uint32_t pulses = feedbackCount;
  uint32_t accumUs = feedbackAccumUs;
  uint32_t lastEdge = feedbackLastEdgeUs;
  feedbackCount = 0;
  feedbackAccumUs = 0;
  portEXIT_CRITICAL(&feedbackMux);

  float measuredFreq;
  if (pulses >= 2 && accumUs > 0 &&
      (uint32_t)(micros() - lastEdge) < FB_STALL_US)
  {
    measuredFreq = (float)pulses * 1000000.0f / (float)accumUs;
  }
  else
  {
    measuredFreq = 0.0f;
  }
  measuredFreqRawHz = measuredFreq;
#if SPP_HAS_VOLTAGE_CTRL
  pushFeedbackSample(measuredFreq); // top-speed capture window
#endif

  if (measuredFreq > 0.0f)
  {
    feedbackAvailable = true; // a genuine tacho edge stream proves the feedback pin is wired
  }

  if (measuredFreq <= 0.0f)
  {
    measFreqFilt = 0.0f;
  }
  else
  {
    measFreqFilt += FB_MEAS_ALPHA * (measuredFreq - measFreqFilt);
  }
  measuredFreqHz = measFreqFilt;
  measuredSpeed = (uint16_t)lroundf(tachoHzToSpeed(measFreqFilt)); // cluster units, for UI/status
  return measFreqFilt;
}

// Trims the feed-forward duty so the measured motor speed tracks the requested
// speed. baseDuty and the return value are in the 12-bit hardware duty domain
// (0..PWM_DUTY_MAX). Works in the tacho FREQUENCY domain (Hz) to keep the error
// smooth. Sample at 100 ms; uses the measurement taken this tick (updateMeasuredFreq).
int16_t applyFeedbackTrim(uint16_t targetSpeed, uint16_t baseDuty)
{
  const float PID_PERIOD_S = 0.1f;   // 100 ms control interval
  const float deadbandHz = feedbackDeadband > 0.0f ? feedbackDeadband : 0.0f; // user-set anti-hunt band; 0 = off

  const float measFreq = measuredFreqHz;

  const uint16_t dutyMax = fbDutyMax();
  const uint16_t scaleFreq = feedbackMaxFreq > 0 ? feedbackMaxFreq : 1;
  const float targetFreq = speedToTachoHz((float)targetSpeed);

  // Below feedbackMinSpeed the motor can't run smoothly (stiction/cogging), so the
  // loop can only hunt. Drop to pure feed-forward and hold the PID reset.
  if (feedbackMinSpeed > 0 && targetSpeed < feedbackMinSpeed)
  {
    pidIntegral = 0.0f;
    pidPrevError = 0.0f;
    pidCorrection = 0;
    return (int16_t)constrain((int32_t)baseDuty, 0, (int32_t)dutyMax);
  }

  const float error = targetFreq - measFreq; // Hz

  // Full calibration duty range spans the full tacho range, so the UI gains keep
  // their authority whatever feedbackMaxFreq is.
  const float dutyPerHz = (float)dutyMax / (float)scaleFreq;

  // Deadband (anti-hunt): near the motor's break-free duty the response is sticky,
  // and chasing measurement jitter sets up a limit-cycle hunt. Inside the band we
  // silence P and D so they don't react to that jitter — but the INTEGRAL keeps
  // accumulating so the loop still trims out the last couple of Hz of steady-state
  // error and actually settles on the target (freezing it here used to leave a fixed
  // ~2 km/h offset). Outside the band the full PID runs as before.
  const bool inDeadband = fabsf(error) < deadbandHz;

  pidIntegral += error * PID_PERIOD_S;
  const float maxIntegral = (float)scaleFreq / (pidKi > 0.001f ? pidKi : 0.001f);
  pidIntegral = constrain(pidIntegral, -maxIntegral, maxIntegral);

  const float derivative = (error - pidPrevError) / PID_PERIOD_S;
  pidPrevError = error;

  const float pTerm = inDeadband ? 0.0f : pidKp * error;
  const float iTerm = pidKi * pidIntegral; // always active — nulls the steady-state offset
  const float dTerm = inDeadband ? 0.0f : pidKd * derivative;
  const float output = (pTerm + iTerm + dTerm) * dutyPerHz;
  pidCorrection = (int16_t)constrain(output, -(float)dutyMax, (float)dutyMax);

  int32_t corrected = (int32_t)baseDuty + pidCorrection;
  // Allow headroom above the calibrated full-scale so the loop can compensate for
  // load / supply sag, but stay inside the 12-bit hardware range.
  corrected = constrain(corrected, 0, (int32_t)PWM_DUTY_MAX);

#if enableDebug && debugFB
  static uint8_t fbLogDiv = 0;
  if (++fbLogDiv >= 10)
  {
    fbLogDiv = 0;
    DEBUG_FB("tgtF=%.1f measF=%.1f/%u Hz meas=%u kph err=%+.1f corr=%+d base=%u applied=%d",
             targetFreq, measFreq, scaleFreq, measuredSpeed, error,
             pidCorrection, baseDuty, (int)corrected);
  }
#endif

  return (int16_t)corrected;
}

#if SPP_HAS_VOLTAGE_CTRL
// ===== Drive Control (S3 board) =====
// Open loop first: the drive for a target speed comes straight from the
// calibration table (SpeedPulserPro_driveCal), so with a few marks captured the
// needle lands where it should without any feedback at all.
//
// With Feedback Enable on (and a tacho, and a table with Hz), ONE slow integrator
// nudges that drive by a few percent to hold the calibrated tacho Hz against load
// and temperature drift. It does nothing inside the deadband, nothing below the
// minimum feedback speed, and is capped at ±vcTrimLimit — so it can't fight the
// calibration or hunt the way the old fast PWM loop + slow voltage loop did.
float driveTrim = 0.0f;

void resetDriveControl()
{
  driveTrim = 0.0f;
}

// Called on the 100 ms control cadence. Returns the applied raw PWM duty.
uint16_t applyDriveControl(uint16_t targetSpeed)
{
  const float PERIOD_S = 0.1f;

  if (targetSpeed == 0)
  {
    resetDriveControl();
    pidCorrection = 0;
    return applyMotorDrive(0.0f);
  }

  // Cap at the top calibration mark: past it both the drive and the trim's target
  // Hz would be extrapolated, and the trim would happily chase an overspin.
  const uint16_t topSpeed = driveCalTopSpeed();
  if (topSpeed > 0 && targetSpeed > topSpeed)
    targetSpeed = topSpeed;

  const float ffDrive = driveForSpeed(targetSpeed);
  const bool closedLoop = feedbackEnable && feedbackAvailable && driveCalHzValid() &&
                          !(feedbackMinSpeed > 0 && targetSpeed < feedbackMinSpeed);
  float tgtHz = 0.0f;
  if (closedLoop)
  {
    tgtHz = hzForSpeed((float)targetSpeed);
    const float errHz = tgtHz - measuredFreqHz;
    const float deadbandHz = feedbackDeadband > 0.0f ? feedbackDeadband : 0.0f;
    if (tgtHz > 0.5f && fabsf(errHz) > deadbandHz)
    {
      driveTrim += vcTrimGain * (errHz / tgtHz) * PERIOD_S;
      driveTrim = constrain(driveTrim, -vcTrimLimit, vcTrimLimit);
    }
  }
  else
  {
    driveTrim = 0.0f;
  }

  const uint16_t duty = applyMotorDrive(ffDrive * (1.0f + driveTrim));
  pidCorrection = (int16_t)lroundf(driveTrim * 100.0f); // UI "trim", in %

#if enableDebug && debugFB
  static uint8_t logDiv = 0;
  if (++logDiv >= 10)
  {
    logDiv = 0;
    DEBUG_FB("drive: tgt=%u ff=%.2f V trim=%+.1f%% -> %.2f V (rail %.2f V x %.0f%%) | tgtF=%.1f measF=%.1f%s",
             targetSpeed, ffDrive, driveTrim * 100.0f, lastDriveV, lastRailCmdV,
             lastPwmFrac * 100.0f, tgtHz, measuredFreqHz, closedLoop ? "" : " (open loop)");
  }
#endif
  return duty;
}
#endif // SPP_HAS_VOLTAGE_CTRL

// ===== Speed -> PWM duty (interpolated, 12-bit) =====
// Anchor on the nearest calibration point (findClosestMatch, 0..384) then linearly
// interpolate toward the neighbour that brackets the requested speed, returning a
// hardware-domain duty (already scaled by DUTY_SCALE_SHIFT) with sub-count precision.
// Falls back to the anchor when there is nothing to interpolate (dead-zone, table
// ends, flat/zero neighbours) so presets and the motor start threshold are unchanged.
uint32_t speedToPwmDuty(uint16_t speedKph)
{
  // Custom (SpeedPulser) calibration: interpolate straight from the anchor
  // points so we use the full 12-bit duty range. The 385-entry table can only
  // represent duty up to 384<<DUTY_SCALE_SHIFT (~37.5%), which would otherwise
  // clamp a full-range custom calibration far short of its top speed.
  if (motorPerformanceVal == CUSTOM_CAL_ID && customCalValid)
  {
    return customSpeedToDuty12(speedKph);
  }

  const int32_t maxIdx = (int32_t)(sizeof motorPerformance / sizeof motorPerformance[0]) - 1; // 384

  const uint16_t dNear = findClosestMatch(speedKph);
  if (dNear == 0)
  {
    return 0; // below the motor's start threshold -> off
  }

  const int32_t vNear = (int32_t)motorPerformance[dNear];
  const uint32_t base = (uint32_t)dNear << DUTY_SCALE_SHIFT;
  if (vNear == (int32_t)speedKph)
  {
    return base; // exact hit — nothing to interpolate
  }

  // Step to the neighbouring calibration point on the side of the requested speed,
  // skipping flat entries so we interpolate across a real slope.
  const int32_t dir = (speedKph > vNear) ? +1 : -1;
  int32_t dOther = (int32_t)dNear + dir;
  while (dOther >= 0 && dOther <= maxIdx && motorPerformance[dOther] == (uint16_t)vNear)
  {
    dOther += dir;
  }
  if (dOther < 0 || dOther > maxIdx || motorPerformance[dOther] == 0)
  {
    return base; // no usable neighbour -> fall back to the anchor
  }

  const int32_t vOther = (int32_t)motorPerformance[dOther];
  const int32_t den = vOther - vNear;
  if (den == 0)
  {
    return base;
  }

  const int32_t num = ((int32_t)speedKph - vNear) * (dOther - (int32_t)dNear);
  int32_t pwm = (int32_t)base + ((num << DUTY_SCALE_SHIFT) / den);

  const int32_t maxPwm = maxIdx << DUTY_SCALE_SHIFT;
  if (pwm < 0) pwm = 0;
  if (pwm > maxPwm) pwm = maxPwm;
  return (uint32_t)pwm;
}

// ===== RPM (coil) output =====
// A 50 % square wave whose frequency tracks engine RPM, generated by a
// general-purpose hardware timer (GPTimer) whose alarm ISR toggles pinRPMOutput
// every half period.
//
// This replaced the LEDC (PWM) implementation. With Arduino-ESP32 3.3.x /
// ESP-IDF 5.5.2, ledc_update_duty() busy-waits inside a critical section
// (interrupts off) until the PREVIOUS duty update has latched, and on the ESP32
// a duty update only latches on the timer's next overflow. At tach rates (a few
// Hz during the needle sweep or cranking) that wait is longer than the 300 ms
// interrupt watchdog and the board panics/reboots. LEDC on the APB clock at
// 10-bit also can't go below ~76 Hz: ledc_set_freq() fails and ESP_LOGE's,
// and that printf blows the 2 KB TaskRPM stack (stack canary panic). The
// GPTimer path has no latch and no floor: a frequency change is just a new
// alarm period, and 0 Hz stops the timer with the pin held low. The motor PWM
// (CH0/TIMER0) and V_ADJ (CH2/TIMER2) stay on LEDC - they run at 10-20 kHz.
static constexpr uint32_t rpmTimerTickHz = 1000000; // 1 µs timer ticks

static hw_timer_t   *rpmTimer      = nullptr;
static volatile bool rpmOutputOn   = false; // timer running / alarm armed
static volatile bool rpmOutputHigh = false; // current pin level
static uint64_t      rpmLastHalfTicks = UINT64_MAX; // last half period applied (0 = off)

static void IRAM_ATTR onRpmTimerAlarm()
{
  if (!rpmOutputOn)
  {
    // Output was switched off while this alarm was in flight: park low.
    rpmOutputHigh = false;
    gpio_ll_set_level(&GPIO, pinRPMOutput, 0);
    return;
  }
  rpmOutputHigh = !rpmOutputHigh;
  gpio_ll_set_level(&GPIO, pinRPMOutput, rpmOutputHigh ? 1 : 0);
}

// Initialise the RPM output timer (frequency is updated by setFrequencyRPM)
void setupTimer()
{
  pinMode(pinRPMOutput, OUTPUT);
  digitalWrite(pinRPMOutput, LOW);

  rpmTimer = timerBegin(rpmTimerTickHz);
  if (rpmTimer == nullptr)
  {
    DEBUG("RPM timer: init failed - no RPM output");
    return;
  }
  timerAttachInterrupt(rpmTimer, &onRpmTimerAlarm);
  timerStop(rpmTimer); // idle (no alarm armed) until the first non-zero rate
  rpmOutputOn   = false;
  rpmOutputHigh = false;
  rpmLastHalfTicks = 0;
}

// Set RPM output frequency (0 = output off, pin low). Fractional Hz is kept: the
// half period is rounded to the nearest 1 µs tick, so a hall RPM input is
// reproduced to well under 0.1 % rather than in whole-Hz steps (1 Hz = 30 RPM on a
// 4-cyl). The timer is only re-armed when that tick count actually changes.
void setFrequencyRPM(float frequencyHz)
{
  uint64_t halfPeriodTicks = 0; // 0 = output off
  if (frequencyHz > 0.0f)
  {
    halfPeriodTicks = (uint64_t)((float)rpmTimerTickHz / (2.0f * frequencyHz) + 0.5f);
    if (halfPeriodTicks < 2)
    {
      halfPeriodTicks = 2;
    }
  }
  if (rpmTimer == nullptr || halfPeriodTicks == rpmLastHalfTicks)
  {
    return; // no timer, or nothing changed
  }
  rpmLastHalfTicks = halfPeriodTicks;

  if (halfPeriodTicks == 0)
  {
    rpmOutputOn = false; // an alarm already in flight will park the pin low
    timerStop(rpmTimer);
    rpmOutputHigh = false;
    digitalWrite(pinRPMOutput, LOW);
    return;
  }

  if (!rpmOutputOn)
  {
    timerWrite(rpmTimer, 0);
    timerAlarm(rpmTimer, halfPeriodTicks, true, 0); // auto-reload: alarm = half period
    rpmOutputOn = true;
    timerStart(rpmTimer);
    return;
  }

  // Already running: re-arm with the new half period. If the counter is
  // already past it (rate went up), restart the half cycle so the next edge
  // comes within one new half period rather than after a 64-bit wrap.
  timerAlarm(rpmTimer, halfPeriodTicks, true, 0);
  if (timerRead(rpmTimer) >= halfPeriodTicks)
  {
    timerWrite(rpmTimer, 0);
  }
}

// ===== Speed Matching Function =====
// motorPerformance[] maps duty index (0..384) -> speed (kph). Leading entries are
// 0 (motor won't turn) and the remainder is non-decreasing. Return the duty whose
// calibrated speed is nearest to `val`; on an equal-distance tie prefer the lower
// duty. Below the motor's lowest calibrated speed we are in the dead-zone, so
// return 0 (motor off) — this is what the caller relies on for the start threshold.
uint16_t findClosestMatch(uint16_t val)
{
  const uint16_t n = sizeof motorPerformance / sizeof motorPerformance[0];

  uint16_t bestIdx = 0;
  uint32_t bestErr = UINT32_MAX;
  uint16_t minSpeed = 0;
  bool haveCal = false;

  for (uint16_t i = 0; i < n; i++)
  {
    const uint16_t spd = motorPerformance[i];
    if (spd == 0)
    {
      continue; // skip dead-zone / uncalibrated entries
    }
    if (!haveCal)
    { // first non-zero entry = lowest calibrated speed
      minSpeed = spd;
      haveCal = true;
    }

    const uint32_t err = (val > spd) ? (uint32_t)(val - spd) : (uint32_t)(spd - val);
    if (err < bestErr)
    { // strict '<' keeps the lower duty on ties
      bestErr = err;
      bestIdx = i;
    }
  }

  // Dead-zone: no calibration, or requested speed at/below the lowest achievable
  // speed -> motor off (preserves the legacy start-threshold behaviour).
  if (!haveCal || val <= minSpeed)
  {
    return 0;
  }
  return bestIdx;
}
