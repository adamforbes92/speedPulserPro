#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_motorCal.h"
#include "SpeedPulserPro_calBuilder.h"
#include "Arduino.h"

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
  filteredRPM = 0;
  samplesRPM.clear();
}

// File-scope (not a function-local static): a function-local static with a
// runtime initializer would emit a __cxa_guard_acquire on first use, which takes
// a FreeRTOS mutex with a timeout — illegal in an ISR and asserts in queue.c.
static volatile unsigned long incomingPreviousMicros = 0;   // hall input, seeded on first pulse
static volatile unsigned long motorSpeedPreviousMicros = 0; // rpm input, seeded on first pulse

// Interrupt handler for incoming frequency (RPM) reading.
// IRAM_ATTR is required so a pulse arriving while the flash cache is disabled
// (EEPROM/LittleFS write) doesn't crash the chip.
void IRAM_ATTR incomingHz()
{
  // Ignore the vehicle hall input entirely while bench-testing or calibrating —
  // the motor is driven from the Speed Test / Cal controls, so an incoming signal
  // must not touch dutyCycleIncoming, lastPulse or the LED counter until the test
  // (or cal) is turned off.
  if (testSpeedo || testCal)
    return;

  unsigned long presentMicros = micros();
  unsigned long previousMicros = incomingPreviousMicros;
  if (previousMicros == 0)
  {
    incomingPreviousMicros = presentMicros; // seed on the first pulse only
    return;
  }
  unsigned long revolutionTime = presentMicros - previousMicros;
  if (revolutionTime < 1000UL)
    return;
  dutyCycleIncoming = (60000000UL / revolutionTime) / 60;
  incomingPreviousMicros = presentMicros;
  lastPulse = millis();
  ledCounter++;
}

// Interrupt handler for motor speed feedback reading.
// IRAM_ATTR is required so a pulse arriving while the flash cache is disabled
// (EEPROM/LittleFS write) doesn't crash the chip.
void IRAM_ATTR incomingMotorSpeed()
{
  unsigned long presentMicros = micros();
  unsigned long previousMicros = motorSpeedPreviousMicros;
  if (previousMicros == 0)
  {
    motorSpeedPreviousMicros = presentMicros; // seed on the first pulse only
    return;
  }
  unsigned long revolutionTime = presentMicros - previousMicros;
  if (revolutionTime < 1000UL)
    return;
  dutyCycleMotor = (60000000UL / revolutionTime) / 60;
  motorSpeedPreviousMicros = presentMicros;
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

// Period-based tacho frequency (Hz), EMA-smoothed, published to measuredFreqHz /
// measuredSpeed. Kept separate from the PID so the readout stays live in open-loop
// modes too. Call on a ~100 ms cadence.
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

  const uint16_t scaleFreq = feedbackMaxFreq > 0 ? feedbackMaxFreq : 1;
  const uint16_t spdSpan = maxSpeed > 0 ? maxSpeed : 1;

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
  measuredSpeed = (uint16_t)((measFreqFilt * (float)spdSpan) / (float)scaleFreq);
  return measFreqFilt;
}

// Trims the feed-forward duty so the measured motor speed tracks the requested
// speed. baseDuty and the return value are in the 12-bit hardware duty domain
// (0..PWM_DUTY_MAX). Works in the tacho FREQUENCY domain (Hz) to keep the error
// smooth. Sample at 100 ms.
int16_t applyFeedbackTrim(uint16_t targetSpeed, uint16_t baseDuty)
{
  const float PID_PERIOD_S = 0.1f;   // 100 ms control interval
  const float deadbandHz = feedbackDeadband > 0.0f ? feedbackDeadband : 0.0f; // user-set anti-hunt band; 0 = off

  const float measFreq = updateMeasuredFreq();

  const uint16_t dutyMax = fbDutyMax();
  const uint16_t scaleFreq = feedbackMaxFreq > 0 ? feedbackMaxFreq : 1;
  const uint16_t spdSpan = maxSpeed > 0 ? maxSpeed : 1;
  const float targetFreq = (float)targetSpeed * (float)scaleFreq / (float)spdSpan;

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

// Initialize RPM LEDC channel (frequency is updated by setFrequencyRPM)
void setupTimer()
{
  ledc_timer_config_t rpm_ledc_timer = {};
  rpm_ledc_timer.speed_mode = LEDC_MODE;
  rpm_ledc_timer.timer_num = LEDC_RPM_TIMER;
  rpm_ledc_timer.freq_hz = 10;
  rpm_ledc_timer.duty_resolution = LEDC_RESOLUTION;
  rpm_ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&rpm_ledc_timer);

  ledc_channel_config_t rpm_ledc_channel = {};
  rpm_ledc_channel.gpio_num = (gpio_num_t)pinRPMOutput;
  rpm_ledc_channel.speed_mode = LEDC_MODE;
  rpm_ledc_channel.channel = LEDC_RPM_CHANNEL;
  rpm_ledc_channel.timer_sel = LEDC_RPM_TIMER;
  rpm_ledc_channel.duty = 0;
  rpm_ledc_channel.hpoint = 0;
  ledc_channel_config(&rpm_ledc_channel);

  setFrequencyRPM(0);
}

// Set RPM output frequency
void setFrequencyRPM(long frequencyHz)
{
  if (frequencyHz > 0)
  {
    ledc_set_freq(LEDC_MODE, LEDC_RPM_TIMER, (uint32_t)frequencyHz);
    ledc_set_duty(LEDC_MODE, LEDC_RPM_CHANNEL, LEDC_RPM_DUTY_50);
    ledc_update_duty(LEDC_MODE, LEDC_RPM_CHANNEL);
  }
  else
  {
    ledc_set_duty(LEDC_MODE, LEDC_RPM_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_RPM_CHANNEL);
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
