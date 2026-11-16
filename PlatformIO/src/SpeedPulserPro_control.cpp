#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_motorCal.h"
#include "SpeedPulserPro_calBuilder.h"
#include "SpeedPulserPro_voltage.h"
#include "SpeedPulserPro_driveCal.h"
#include "Arduino.h"
#include "hal/gpio_ll.h"

// ===== Speed offset + units (V5.06, mirrors SpeedPulser V5.06) =====
// Global: one fixed km/h added to every speed. Curve: an offset at 25 / 75 / 125 /
// 175 / 225 km/h, straight lines between them, flat beyond the ends (so the needle
// never jumps as the speed crosses from one band into the next; up to V5.05 it
// stepped at 50 / 100 / 150 / 200).
static const float CURVE_POINT_KPH[SPEED_OFFSET_CURVE_POINTS] = {25.0f, 75.0f, 125.0f, 175.0f, 225.0f};

static float curveOffsetAt(float kph)
{
  const int16_t *off = speedOffsetCurveOffsets;
  if (kph <= CURVE_POINT_KPH[0])
    return off[0];
  for (uint8_t i = 1; i < SPEED_OFFSET_CURVE_POINTS; i++)
  {
    if (kph <= CURVE_POINT_KPH[i])
    {
      const float t = (kph - CURVE_POINT_KPH[i - 1]) / (CURVE_POINT_KPH[i] - CURVE_POINT_KPH[i - 1]);
      return off[i - 1] + t * (float)(off[i] - off[i - 1]);
    }
  }
  return off[SPEED_OFFSET_CURVE_POINTS - 1];
}

// km/h -> offset -> mph if the dial reads mph, rounded once at the end (V5.05 and
// before truncated twice, so the needle read ~0.5 mph low on average).
uint16_t toNeedleTarget(float speedKph)
{
  if (speedKph <= 0.0f)
  {
    speedOffsetApplied = 0.0f;
    return 0; // standing still: no offset, needle at rest
  }
  const float offset = useSpeedOffsetCurve ? curveOffsetAt(speedKph) : (float)globalSpeedOffsetKph;
  speedOffsetApplied = offset;
  float target = speedKph + offset;
  if (target < 0.0f)
    target = 0.0f;
  if (clusterInMph)
    target *= KPH_TO_MPH;
  long rounded = lroundf(target);
  if (rounded > 400)
    rounded = 400;
  return (uint16_t)rounded;
}

void resetRPMMedianFilter()
{
  rawCountRPM = 0;
  filteredRPMHz = 0.0f;
  samplesRPM.clear();
}

// ============================================================================
// Speed sensor inputs: hall (and VR on the S3) — V5.06, mirrors SpeedPulser V5.06
// ============================================================================
// Each interrupt keeps the time of the last INPUT_EDGE_HISTORY pulses; the speed
// task works the speed out from them: walk back from the newest pulse until the
// pulses cover inputSmoothingMs (or the history runs out), frequency = intervals /
// time. Between pulses the reading holds, but no longer than the car could still be
// moving that fast (with no pulse for `age` it can't be faster than 1.5 / age), so a
// stopping car reads down smoothly, and 0 after the input's timeout (V5.09, mirrors
// SpeedPulser: INPUT_TIMEOUT_MS, longer for a slow sensor so it doesn't flicker to 0
// between pulses). Once stopped the history is forgotten: micros() wraps every 71.6
// minutes, and a parked car's last pulses would otherwise look new for a moment.
//
// The interrupts take inputMux too (V5.09): the tasks run on the other core, where
// the task's critical section alone doesn't keep the interrupt out.
//
// Glitch filter: a pulse closer to the last one than half a period at Max Speed
// can't be real (ignition noise) and is dropped, keeping the last good pulse. The
// limit follows each input's own pulses-per-km setting.
//
// IRAM_ATTR: the interrupts must run while the flash cache is off (settings save).
#define INPUT_EDGE_HISTORY 64 // power of 2
struct PulseTimer
{
  volatile uint32_t edgeUs[INPUT_EDGE_HISTORY]; // micros() of the newest pulses
  volatile uint32_t edgeCount;                  // pulses accepted since boot
  volatile uint32_t glitchUs;                   // pulses closer than this are noise
  uint32_t timeoutUs;                           // no pulse for this long = stopped
};
static PulseTimer hallTimer = {{0}, 0, 2500, INPUT_TIMEOUT_MS * 1000UL};
#if SPP_HAS_VR
static PulseTimer vrTimer = {{0}, 0, 2500, INPUT_TIMEOUT_MS * 1000UL};
#endif
static portMUX_TYPE inputMux = portMUX_INITIALIZER_UNLOCKED;

static inline void IRAM_ATTR pulseTimerEdge(PulseTimer &timer, uint32_t now)
{
  portENTER_CRITICAL_ISR(&inputMux);
  const uint32_t n = timer.edgeCount;
  if (!n || (now - timer.edgeUs[(n - 1) & (INPUT_EDGE_HISTORY - 1)]) >= timer.glitchUs)
  {
    timer.edgeUs[n & (INPUT_EDGE_HISTORY - 1)] = now;
    timer.edgeCount = n + 1;
  } // else noise: keep the last good pulse
  portEXIT_CRITICAL_ISR(&inputMux);
}

// Stopped after INPUT_TIMEOUT_SLOW_PERIODS pulse periods at INPUT_TIMEOUT_SLOW_KPH,
// within INPUT_TIMEOUT_MS..INPUT_TIMEOUT_MAX_MS (a 100 pulses/km sensor gives one
// pulse every 1.8 s at 20 km/h, which the fixed 1.5 s read as stopped).
static uint32_t timeoutUsFor(float pulsesPerKm)
{
  const float slowPeriodMs = 3600000.0f / (INPUT_TIMEOUT_SLOW_KPH * pulsesPerKm);
  const float timeoutMs = constrain(INPUT_TIMEOUT_SLOW_PERIODS * slowPeriodMs, (float)INPUT_TIMEOUT_MS,
                                    (float)INPUT_TIMEOUT_MAX_MS);
  return (uint32_t)(timeoutMs * 1000.0f);
}

static uint32_t glitchUsFor(float pulsesPerKm)
{
  const float fullScaleHz = (float)maxSpeed * pulsesPerKm / 3600.0f;
  uint32_t us = fullScaleHz > 1.0f ? (uint32_t)(500000.0f / fullScaleHz) : 3000;
  return constrain(us, (uint32_t)INPUT_GLITCH_MIN_US, (uint32_t)INPUT_GLITCH_MAX_US);
}

void updateInputGlitchFilters()
{
  hallTimer.glitchUs = glitchUsFor(hallPulsesPerKm);
  hallTimer.timeoutUs = timeoutUsFor(hallPulsesPerKm);
#if SPP_HAS_VR
  vrTimer.glitchUs = glitchUsFor(vrPulsesPerKm);
  vrTimer.timeoutUs = timeoutUsFor(vrPulsesPerKm);
#endif
}

static float pulseTimerHz(PulseTimer &timer)
{
  uint32_t edges[INPUT_EDGE_HISTORY];
  portENTER_CRITICAL(&inputMux);
  const uint32_t n = timer.edgeCount;
  for (uint8_t i = 0; i < INPUT_EDGE_HISTORY; i++)
    edges[i] = timer.edgeUs[i];
  portEXIT_CRITICAL(&inputMux);
  if (n < 2)
    return 0.0f;
  const uint32_t timeoutUs = timer.timeoutUs;
  const uint32_t newest = edges[(n - 1) & (INPUT_EDGE_HISTORY - 1)];
  const uint32_t age = micros() - newest;
  if (age >= timeoutUs)
  {
    portENTER_CRITICAL(&inputMux);
    if (timer.edgeCount == n)
      timer.edgeCount = 0; // stopped: forget the old pulses (see the micros() note above)
    portEXIT_CRITICAL(&inputMux);
    return 0.0f;
  }
  const uint32_t wantedSpanUs = (uint32_t)inputSmoothingMs * 1000UL;
  const uint32_t maxIntervals = (n - 1 < INPUT_EDGE_HISTORY - 1) ? n - 1 : INPUT_EDGE_HISTORY - 1;
  uint32_t intervals = 0, spanUs = 0;
  for (uint32_t i = 1; i <= maxIntervals; i++)
  {
    const uint32_t span = newest - edges[(n - 1 - i) & (INPUT_EDGE_HISTORY - 1)];
    if (span - spanUs > timeoutUs)
      break; // the car was stopped before this pulse
    intervals = i;
    spanUs = span;
    if (spanUs >= wantedSpanUs)
      break;
  }
  if (intervals == 0 || spanUs == 0)
    return 0.0f;
  float hz = (float)intervals * 1000000.0f / (float)spanUs;
  if (hz * (float)age > 1500000.0f)
    hz = 1500000.0f / (float)age; // next pulse overdue: slowing down
  return hz;
}

float measureHallFrequency() { return pulseTimerHz(hallTimer); }
#if SPP_HAS_VR
float measureVRFrequency() { return pulseTimerHz(vrTimer); }
#endif

void IRAM_ATTR incomingHz()
{
  pulseTimerEdge(hallTimer, micros());
  ledCounter++;
}

#if SPP_HAS_VR
// The VR conditioner (MAX9924 C Out) gives one clean edge per zero-crossing.
void IRAM_ATTR incomingVR()
{
  pulseTimerEdge(vrTimer, micros());
}
#endif

// ===== Engine RPM input (tach output) =====
// Windowed frequency capture: the ISR sums the edge intervals and counts them, the
// RPM task reads-and-clears the pair once per window (freq = count / summed time).
// Edges closer than half a period at Cluster Frequency Limit are dropped (V5.02).
//
// File-scope (not a function-local static): a function-local static with a
// runtime initializer would emit a __cxa_guard_acquire on first use, which takes
// a FreeRTOS mutex with a timeout — illegal in an ISR and asserts in queue.c.
struct PulseWindow
{
  volatile uint32_t accumUs;       // summed edge-to-edge intervals (us) this window
  volatile uint32_t count;         // number of intervals summed into accumUs
  volatile uint32_t lastEdgeUs;    // micros() of the previous accepted edge (kept across windows)
  volatile uint32_t minIntervalUs; // edges closer than this are rejected
};
static const uint32_t PULSE_MIN_INTERVAL_DEFAULT_US = 3000; // until the first read sets it
static const uint32_t PULSE_MIN_INTERVAL_FLOOR_US = 1000;   // never accept edges above 1 kHz
static PulseWindow rpmPulse = {0, 0, 0, PULSE_MIN_INTERVAL_DEFAULT_US};

static uint32_t pulseMinIntervalUs(uint16_t fullScaleHz)
{
  if (fullScaleHz == 0)
    return PULSE_MIN_INTERVAL_DEFAULT_US;
  uint32_t us = 500000UL / fullScaleHz; // half a period at full scale
  return us < PULSE_MIN_INTERVAL_FLOOR_US ? PULSE_MIN_INTERVAL_FLOOR_US : us;
}

static portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;

static inline bool IRAM_ATTR pulseEdge(PulseWindow &w, uint32_t now)
{
  bool accepted = false;
  portENTER_CRITICAL_ISR(&pulseMux); // the RPM task reads-and-clears from the other core
  const uint32_t last = w.lastEdgeUs;
  if (last == 0)
  {
    w.lastEdgeUs = now; // seed on the first pulse only
  }
  else if (now - last >= w.minIntervalUs) // closer = coil EMI / bounce: keep the last good edge
  {
    w.accumUs += now - last;
    w.count++;
    w.lastEdgeUs = now;
    accepted = true;
  }
  portEXIT_CRITICAL_ISR(&pulseMux);
  return accepted;
}

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

static void resetPulseWindow(PulseWindow &w)
{
  portENTER_CRITICAL(&pulseMux);
  w.accumUs = 0;
  w.count = 0;
  w.lastEdgeUs = 0;
  portEXIT_CRITICAL(&pulseMux);
}

float readRPMHz() { return readPulseHz(rpmPulse, maxRPM); }
void resetRPMPulseCounter() { resetPulseWindow(rpmPulse); }

// Interrupt handler for the engine RPM input.
// IRAM_ATTR is required so a pulse arriving while the flash cache is disabled
// (EEPROM/LittleFS write) doesn't crash the chip.
void IRAM_ATTR incomingMotorSpeed()
{
  if (pulseEdge(rpmPulse, micros()))
    lastPulseRPM = millis();
}

// ============================================================================
// Closed-Loop Motor Feedback — tacho pulse timestamps (V5.05, from SpeedPulser V5.04)
// ============================================================================
// The motor tacho (pinMotorFeedback) is timed in software, on every board: this
// ISR keeps the micros() of the last FB_TS_N falling edges, and
// updateMeasuredFreq() works the frequency out from them.
//
// Up to V5.04 the ISR summed edge intervals per 100 ms window and needed two edges
// in a window, so below ~20 Hz windows read 0 Hz while the motor was still turning
// and the reading went 20 -> 0 -> 20 (SpeedPulser bench, 2026-11-15: 14 of 29
// windows read 0 at 15 Hz, all of them at 8.5 Hz). Timing the edges themselves has
// no such floor.
//
// Edges closer than FB_GLITCH_US are line noise (the top speed is ~254 Hz =
// 3.9 ms; the bench saw 0.02-0.6 ms pairs on the tacho line) and are dropped,
// keeping the earlier edge.
//
// IRAM_ATTR is required so a pulse arriving while the flash cache is disabled
// (EEPROM/LittleFS write) doesn't crash the chip.
#define FB_TS_N 32                // edge history (power of 2): 6 revolutions + 2
static volatile uint32_t fbTs[FB_TS_N]; // micros() of the newest edges
static volatile uint32_t fbSeq = 0;     // edges accepted since boot
// Guards the edge history between the ISR and the speed task (other core).
// portMUX is the FreeRTOS/ESP32 safe primitive; a global noInterrupts() would
// starve the WiFi radio.
static portMUX_TYPE feedbackMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR feedbackPulse()
{
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&feedbackMux);
  const uint32_t s = fbSeq;
  if (!s || (now - fbTs[(s - 1) & (FB_TS_N - 1)]) >= FB_GLITCH_US)
  {
    fbTs[s & (FB_TS_N - 1)] = now;
    fbSeq = s + 1;
  } // else noise: keep the earlier edge
  portEXIT_CRITICAL_ISR(&feedbackMux);
}

static float measFreqFilt = 0.0f; // EMA-smoothed tacho frequency


// ===== Speed <-> tacho scale =====
// S3 board: the drive calibration marks map dial speed <-> tacho Hz (piecewise, so
// a dial that isn't linear near zero still reads right). No table = 0. The DevKit
// has no motor feedback (V5.06: the legacy PID scale is gone).
float speedToTachoHz(float speed)
{
#if SPP_HAS_VOLTAGE_CTRL
  if (driveCalHzValid())
    return hzForSpeed(speed);
#endif
  (void)speed;
  return 0.0f;
}

static float tachoHzToSpeed(float hz)
{
#if SPP_HAS_VOLTAGE_CTRL
  if (driveCalHzValid())
    return speedForHz(hz);
#endif
  (void)hz;
  return 0.0f;
}

#if SPP_HAS_VOLTAGE_CTRL

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

// ===== Motor Speed Measurement (V5.05, from SpeedPulser V5.04) =====
// Motor frequency (Hz) from the edge timestamps, smoothed, published to
// measuredFreqHz/measuredSpeed. Called once per 100 ms by the speed task in EVERY
// mode, so the read-outs, the capture window and the feedback loops all share one
// fresh measurement.
//
// From the newest edge, walk back over whole revolutions (multiples of
// FB_PULSES_PER_REV intervals: the poles aren't evenly spaced, and a whole turn
// cancels that) until the span covers FB_SPAN_US, or the history runs out.
// freq = intervals / span. At 254 Hz that is 6 turns in ~120 ms; at 8 Hz one turn
// in ~0.6 s. Never averages across a gap longer than FB_STALL_US (the motor was
// stopped there). Between edges the reading holds, but no longer than the motor
// could plausibly be turning: with no edge for `age`, it can't be faster than
// 1.5 / age, so a stopping motor reads down smoothly, and 0 after FB_STALL_US.
float updateMeasuredFreq()
{

  uint32_t ts[FB_TS_N];
  portENTER_CRITICAL(&feedbackMux);
  const uint32_t seq = fbSeq;
  for (uint8_t i = 0; i < FB_TS_N; i++)
    ts[i] = fbTs[i];
  portEXIT_CRITICAL(&feedbackMux);
  const uint32_t nowUs = micros();

  float measuredFreq = 0.0f;
  uint32_t kUsed = 0;
  if (seq >= 2)
  {
    const uint32_t newest = ts[(seq - 1) & (FB_TS_N - 1)];
    const uint32_t age = nowUs - newest;
    if (age >= FB_STALL_US)
    {
      // Stopped: forget the old edges, or once micros() wraps (71.6 min) they look new.
      portENTER_CRITICAL(&feedbackMux);
      if (fbSeq == seq)
        fbSeq = 0;
      portEXIT_CRITICAL(&feedbackMux);
    }
    else
    {
      const uint32_t maxK = (seq - 1 < FB_TS_N - 1) ? seq - 1 : FB_TS_N - 1;
      uint32_t k = 0, span = 0, kRev = 0, spanRev = 0;
      for (uint32_t i = 1; i <= maxK; i++)
      {
        const uint32_t s = newest - ts[(seq - 1 - i) & (FB_TS_N - 1)];
        if (s - span > FB_STALL_US)
          break; // the motor was stopped before this edge
        k = i;
        span = s;
        if (k % FB_PULSES_PER_REV == 0)
        {
          kRev = k;
          spanRev = span;
          if (span >= FB_SPAN_US)
            break;
        }
      }
      if (kRev) // whole turns available: use them
      {
        k = kRev;
        span = spanRev;
      }
      if (k && span)
      {
        measuredFreq = (float)k * 1000000.0f / (float)span;
        if (measuredFreq * (float)age > 1500000.0f)
          measuredFreq = 1500000.0f / (float)age; // overdue edge: slowing down
        kUsed = k;
      }
    }
  }
  measuredFreqRawHz = measuredFreq;
#if SPP_HAS_VOLTAGE_CTRL
  pushFeedbackSample(measuredFreq); // capture window
#endif

  if (measuredFreq > 0.0f && kUsed >= 2)
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

#if SPP_HAS_VOLTAGE_CTRL
// ===== Drive Control (S3 board) =====
// Open loop first: the drive for a target speed comes straight from the
// calibration table (SpeedPulserPro_driveCal), so with a few marks captured the
// needle lands where it should without any feedback at all.
//
// With Feedback Enable on (and a tacho, and a table with Hz), ONE slow integrator
// nudges that drive by a few percent to hold the calibrated tacho Hz against load
// and temperature drift. It does nothing inside the deadband, nothing below the
// minimum feedback speed, and is capped at ±trimLimit — so it can't fight the
// calibration or hunt the way the old fast PWM loop + slow voltage loop did.
float driveTrim = 0.0f;

// V5.05 reference model (SpeedPulser V5.04): the tacho Hz the motor should be
// showing NOW, following the target the way the motor + measurement do (bench,
// 2026-11-15: 63 % of a step in ~0.35 s, 90 % in ~0.55 s, up or down, 10-160 km/h).
// The trim works on the error against this, and only once it has (nearly) reached
// the target: getting to a new speed is the calibration's job. Integrating the raw
// step error instead wound the trim up while the motor was still accelerating:
// 5-31 Hz overshoot on a speed-up, and on a big slow-down the needle dipped to ~0
// and took ~7 s to come back.
static float refHz = 0.0f;

// Tacho lost (V5.08, mirrors SpeedPulser): feedbackAvailable latches once a tacho is
// seen, so a wire that breaks later read as "the motor is stopped" and the trim
// wound up to +trimLimit in about a second (needle ~40 % high). If the tacho reads
// nothing for FB_LOST_TICKS while the target is above the motor's floor, the trim
// is dropped (open loop, straight from the calibration) and the page says the
// feedback is missing. A reading far from the model (noise) is never integrated.
static uint8_t tachoSilentTicks = 0;
static bool tachoLost = false;

bool tachoSignalLost() { return tachoLost; }

void resetDriveControl()
{
  driveTrim = 0.0f;
  refHz = 0.0f;
  tachoSilentTicks = 0;
}

// Called on the 100 ms control cadence. Returns the applied raw PWM duty.
uint16_t applyDriveControl(uint16_t targetSpeed)
{
  const float PERIOD_S = 0.1f;

  if (targetSpeed == 0)
  {
    resetDriveControl();
    return applyMotorDrive(0.0f);
  }

  // Cap at the top calibration mark: past it both the drive and the trim's target
  // Hz would be extrapolated, and the trim would happily chase an overspin.
  const uint16_t topSpeed = driveCalTopSpeed();
  if (topSpeed > 0 && targetSpeed > topSpeed)
    targetSpeed = topSpeed;

  const float ffDrive = driveForSpeed(targetSpeed);
  if (feedbackAvailable && measuredFreqRawHz <= 0.0f && hzForSpeed((float)targetSpeed) > FB_LOST_MIN_HZ)
  {
    if (tachoSilentTicks < 255)
      tachoSilentTicks++;
  }
  else
  {
    tachoSilentTicks = 0;
  }
  tachoLost = tachoSilentTicks >= FB_LOST_TICKS;
  const bool closedLoop = feedbackEnable && feedbackAvailable && !tachoLost && driveCalHzValid() &&
                          !(feedbackMinSpeed > 0 && targetSpeed < feedbackMinSpeed);
  float tgtHz = 0.0f;
  if (closedLoop)
  {
    tgtHz = hzForSpeed((float)targetSpeed);
    if (refHz <= 0.0f)
      refHz = measuredFreqHz; // loop just engaged: start the model where the motor is
    refHz += (tgtHz - refHz) * (PERIOD_S / (FB_REF_TAU_S + PERIOD_S));
    const float errHz = refHz - measuredFreqHz;
    const float deadbandHz = feedbackDeadband > 0.0f ? feedbackDeadband : 0.0f;
    const bool settled = fabsf(tgtHz - refHz) <= FB_REF_SETTLED * tgtHz;
    const bool plausible = fabsf(errHz) <= FB_PLAUSIBLE * refHz;
    if (tgtHz > 0.5f && settled && plausible && fabsf(errHz) > deadbandHz)
    {
      driveTrim += trimGain * (errHz / tgtHz) * PERIOD_S;
      driveTrim = constrain(driveTrim, -trimLimit, trimLimit);
    }
  }
  else
  {
    driveTrim = 0.0f;
    refHz = 0.0f;
  }

  // Never let the trim push the drive more than 5 % past the top proven mark:
  // past it nobody has seen where the needle goes (a pegged speedo is damaged), nor
  // past the most the rail can give.
  float drive = ffDrive * (1.0f + driveTrim);
  float driveCap = driveMaxV();
  if (topSpeed > 0 && driveForSpeed(topSpeed) * 1.05f < driveCap)
    driveCap = driveForSpeed(topSpeed) * 1.05f;
  if (drive > driveCap)
  {
    drive = driveCap;
    // Anti-windup (V5.09): the trim stops where the drive stops. It used to keep
    // integrating against the cap up to +trimLimit, then held the needle high for
    // seconds after the speed dropped.
    if (closedLoop && ffDrive > 0.0f && driveTrim > drive / ffDrive - 1.0f)
      driveTrim = drive / ffDrive - 1.0f;
  }
  const uint16_t duty = applyMotorDrive(drive);

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
  if (presetId == CUSTOM_CAL_ID && customCalValid)
  {
    return customSpeedToDuty12(speedKph);
  }

  const int32_t maxIdx = (int32_t)(sizeof presetSpeedTable / sizeof presetSpeedTable[0]) - 1; // 384

  const uint16_t dNear = findClosestMatch(speedKph);
  if (dNear == 0)
  {
    return 0; // below the motor's start threshold -> off
  }

  const int32_t vNear = (int32_t)presetSpeedTable[dNear];
  const uint32_t base = (uint32_t)dNear << DUTY_SCALE_SHIFT;
  if (vNear == (int32_t)speedKph)
  {
    return base; // exact hit — nothing to interpolate
  }

  // Step to the neighbouring calibration point on the side of the requested speed,
  // skipping flat entries so we interpolate across a real slope.
  const int32_t dir = (speedKph > vNear) ? +1 : -1;
  int32_t dOther = (int32_t)dNear + dir;
  while (dOther >= 0 && dOther <= maxIdx && presetSpeedTable[dOther] == (uint16_t)vNear)
  {
    dOther += dir;
  }
  if (dOther < 0 || dOther > maxIdx || presetSpeedTable[dOther] == 0)
  {
    return base; // no usable neighbour -> fall back to the anchor
  }

  const int32_t vOther = (int32_t)presetSpeedTable[dOther];
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
    // Floor: 250 us half period (2 kHz). The timer interrupts every half period, so
    // a runaway frequency must never turn into an interrupt storm that stalls the core.
    if (halfPeriodTicks < 250)
    {
      halfPeriodTicks = 250;
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
// presetSpeedTable[] maps duty index (0..384) -> speed (kph). Leading entries are
// 0 (motor won't turn) and the remainder is non-decreasing. Return the duty whose
// calibrated speed is nearest to `val`; on an equal-distance tie prefer the lower
// duty. Below the motor's lowest calibrated speed we are in the dead-zone, so
// return 0 (motor off) — this is what the caller relies on for the start threshold.
uint16_t findClosestMatch(uint16_t val)
{
  const uint16_t n = sizeof presetSpeedTable / sizeof presetSpeedTable[0];

  uint16_t bestIdx = 0;
  uint32_t bestErr = UINT32_MAX;
  uint16_t minSpeed = 0;
  bool haveCal = false;

  for (uint16_t i = 0; i < n; i++)
  {
    const uint16_t spd = presetSpeedTable[i];
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
