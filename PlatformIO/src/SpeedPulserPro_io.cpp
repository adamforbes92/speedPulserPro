#include "SpeedPulserPro_io.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_gps.h"
#include "SpeedPulserPro_voltage.h"
#include "SpeedPulserPro_driveCal.h"
#include "SpeedPulserPro_calBuilder.h"

// Drive the direction pin from the reverseDirection flag.
// Default (unticked) drives LOW = the correct direction; ticking Reverse drives HIGH.
// (Mirrors SpeedPulser V5.01. Up to V5.00 this was the other way round, so Reverse
// had to be ticked to spin the right way; the setting moved to a new EEP key so
// those units start unticked.)
void applyDirection()
{
  digitalWrite(pinMotorDirection, reverseDirection ? HIGH : LOW);
}

#if SPP_HAS_SPEED_BIAS
// S3 board: the speed-input pull-up / pull-down are two transistor switches on
// their own GPIOs (both held off by R44/R45 at reset). Only ONE may ever be on,
// so both are dropped before the chosen one is raised — the two 4.7k legs would
// otherwise sit across VCC.
void applySpeedInputBias()
{
  digitalWrite(pinSpeedPullUp, LOW);
  digitalWrite(pinSpeedPullDown, LOW);
  if (speedInputBias == SPEED_BIAS_PULL_UP)
    digitalWrite(pinSpeedPullUp, HIGH);
  else if (speedInputBias == SPEED_BIAS_PULL_DOWN)
    digitalWrite(pinSpeedPullDown, HIGH);
  DEBUG_IO("speed input bias: %s", speedInputBias == SPEED_BIAS_PULL_UP ? "pull-up (VCC)"
                                   : speedInputBias == SPEED_BIAS_PULL_DOWN ? "pull-down (GND)"
                                                                            : "none (floating)");
}
#endif

void basicInit()
{
  DEBUG_IO("Initialising SpeedPulser (%s)...", SPP_BOARD_NAME);

  DEBUG_IO("Reading EEPROM...");
  readEEP();
  DEBUG_IO("Read EEPROM!");

  DEBUG_IO("Setting up LED Output...");
  pinMode(pinOnboardLED, OUTPUT);
  digitalWrite(pinOnboardLED, ledOnboard);
  DEBUG_IO("Set up LED Output!");

#if SPP_HAS_SPEED_BIAS
  DEBUG_IO("Setting up Speed Input Bias (PU IO%d / PD IO%d)...", pinSpeedPullUp, pinSpeedPullDown);
  pinMode(pinSpeedPullUp, OUTPUT);
  pinMode(pinSpeedPullDown, OUTPUT);
  applySpeedInputBias(); // from the persisted setting; default = none
#endif

#if SPP_HAS_CAN_STANDBY
  // TCAN1044V STB: R18 already pulls it low, but drive it explicitly so a floating
  // GPIO can never leave the transceiver in standby.
  pinMode(pinCANStandby, OUTPUT);
  digitalWrite(pinCANStandby, LOW);
#endif

  DEBUG_IO("Setting up RPM Output...");
  pinMode(pinRPMOutput, OUTPUT);
  DEBUG_IO("Set up RPM Output!");

  DEBUG_IO("Setting up PWM (LEDc)...");
  // Configure LEDC timer for 10 kHz at 12-bit resolution (motor PWM)
  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_MODE;
  ledc_timer.timer_num = LEDC_TIMER;
  ledc_timer.freq_hz = LEDC_FREQUENCY;
  ledc_timer.duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;

  ledc_timer_config(&ledc_timer);

  // Configure LEDC channel
  ledc_channel_config_t ledc_channel = {};
  ledc_channel.gpio_num = (gpio_num_t)pinMotorOutput;
  ledc_channel.speed_mode = LEDC_MODE;
  ledc_channel.channel = LEDC_OUTPUT_CHANNEL;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.timer_sel = LEDC_TIMER;
  ledc_channel.duty = 0;
  ledc_channel.hpoint = 0;
  ledc_channel_config(&ledc_channel);

  DEBUG_IO("Set up PWM (LEDC)!");

  DEBUG_IO("Setting up Speed Interrupt...");
  // INPUT_PULLUP holds a disconnected/idle input steady so stray noise can't fake pulses.
  pinMode(pinGearboxHall, INPUT_PULLUP);
  pinMode(pinEngineRPMInput, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinGearboxHall), incomingHz, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinEngineRPMInput), incomingMotorSpeed, FALLING);
  DEBUG_IO("Set up Speed Interrupt!");

#if SPP_HAS_VR
  // Variable-reluctance (VR) speed input — counted like the hall input. The MAX9924
  // drives a clean digital edge (C Out) with R15 10k pull-up to 3.3V, so INPUT (no
  // internal pull) is used and we count the FALLING edge.
  DEBUG_IO("Setting up VR Speed Interrupt (GPIO%d)...", pinVRInput);
  pinMode(pinVRInput, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinVRInput), incomingVR, FALLING);
  DEBUG_IO("Set up VR Speed Interrupt!");
#endif

  // Motor tacho feedback for the closed-loop PID.
  // INPUT_PULLUP (not plain INPUT): on a legacy PCB with no feedback trace the pin
  // floats and picks up noise, faking a tacho signal that latches feedbackAvailable
  // and lets the PID wind up and peg the motor to full scale. The pull-up holds a
  // disconnected pin steady so feedback stays correctly "missing" (open-loop, UI N/A).
#if SPP_HAS_VOLTAGE_CTRL // the DevKit has no tacho wired (V5.09: it attached the interrupt anyway)
  DEBUG_IO("Setting up Feedback Interrupt (GPIO%d)...", pinMotorFeedback);
  pinMode(pinMotorFeedback, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinMotorFeedback), feedbackPulse, FALLING);
  DEBUG_IO("Set up Feedback Interrupt!");
#endif

  DEBUG_IO("Setting up Motor Direction (GPIO%d)...", pinMotorDirection);
  pinMode(pinMotorDirection, OUTPUT);
  applyDirection();  // normal = LOW, reverse = HIGH
  DEBUG_IO("Set up Motor Direction! (%s)", reverseDirection ? "REV" : "FWD");

  DEBUG_IO("Setting up CAN...");
  canInit();
  DEBUG_IO("Set up CAN!");

  DEBUG_GPS("Setting up GPS Module...");
  initGPS();
#if enableDebug && debugGPS
  DEBUG_GPS("Set up GPS Module! TinyGPS++ v%s", TinyGPSPlus::libraryVersion());
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
#endif

  DEBUG_IO("Initialised SpeedPulser!");
}

// setMotorDuty() takes a duty in the 10-bit calibration (0..384)
// the legacy calibration tables are in 10-bit so scale it to the 12-bit PWM
// hardware range, keeping stored calibrations voltage-identical with finer resolution.
void setMotorDuty(uint32_t duty)
{
  setMotorDutyRaw(duty << DUTY_SCALE_SHIFT);
}

// Write a duty straight to the 12-bit PWM hardware (0..PWM_DUTY_MAX). Used by
// the interpolated speed path and the needle sweep for sub-count precision.
// motorDutyApplied is kept in the full 12-bit domain so the live cal-curve marker
// and the calibration builder share one duty scale (mirrors SpeedPulser).
void setMotorDutyRaw(uint32_t pwmDuty)
{
  motorDutyApplied = (uint16_t)pwmDuty;
  ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_CHANNEL, pwmDuty);
  ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_CHANNEL);
}

// ===== Needle sweep =====
// Mirrors SpeedPulser V5.07. The speed needle follows the motor's SPEED, and
// speed isn't a straight line against drive or duty: nothing until the motor
// breaks away, then a steep low end, then a flatter top. Ramping the drive evenly
// made the needle leap off the stop, slow in the middle and rush at the end. So
// the sweep ramps the dial SPEED evenly and looks up the output for it through the
// calibration on every tick: the calibration is the measured inverse of that
// bent curve, so an even speed ramp gives an even needle.
//
// One climb: from the lowest calibrated speed at a steady pace, then the last
// SWEEP_EASE of the time slowing to a stop, so the weighted needle isn't flung at
// the top. It starts at full pace: the motor's own lag (~0.35 s) already eases the
// needle off the stop. At the top it waits for the motor to get there (S3 tacho)
// or a fixed hold. The way down is the climb played backwards; at the lowest
// speed the motor is cut and coasts the needle home.
//
// Range: only where the calibration has proven the needle goes, the lowest to the
// highest calibrated speed (S3: the drive marks; DevKit: the selected preset or
// the custom table). No calibration = no speed sweep (the tach still sweeps).
//
// The tach needle is an air-core coil driven by frequency, which IS a straight
// line, so it simply follows the same profile from 0 to Max RPM.
//
// Pace: the speed needle runs at Sweep pace (ms per dial km/h or mph) x
// SWEEP_RATE_REF / Speed needle rate. The tach takes as long as the speed needle
// would over Max Speed, x SWEEP_RATE_REF / Tach needle rate. All read live, so
// moving a slider retimes a sweep in progress.

// Position 0..1 along the climb at time 0..1: a steady rate, then a cosine
// slow-down to rest. The steady rate is set so the climb still ends at 1.
static const float SWEEP_STEADY_RATE = 1.0f / (1.0f - SWEEP_EASE / 2.0f);
static float sweepProfile(float t)
{
  const float easeFrom = 1.0f - SWEEP_EASE;
  if (t <= easeFrom)
    return SWEEP_STEADY_RATE * t;
  const float u = (t - easeFrom) / SWEEP_EASE; // 0..1 through the slow-down
  return SWEEP_STEADY_RATE * (easeFrom + SWEEP_EASE * (u / 2.0f + sinf(PI * u) / (2.0f * PI)));
}

// The calibrated dial range. false = no calibration to sweep through.
static bool sweepRange(uint16_t &lowSpeed, uint16_t &topSpeed)
{
#if SPP_HAS_VOLTAGE_CTRL
  if (driveCalCount == 0)
    return false;
  lowSpeed = driveCal[0].speed;
  topSpeed = driveCalTopSpeed();
#else
  topSpeed = 0;
  if (presetId == CUSTOM_CAL_ID && customCalValid)
    topSpeed = customCalPoints[customCalCount - 1].speed;
  else
    for (uint16_t s : presetSpeedTable)
      if (s > topSpeed)
        topSpeed = s;
  lowSpeed = 1; // the first speed the table gives any duty for
  while (lowSpeed < topSpeed && speedToPwmDuty(lowSpeed) == 0)
    lowSpeed++;
#endif
  if (lowSpeed >= topSpeed)
    lowSpeed = 0; // a single drive mark: its line runs through zero
  return topSpeed > 0;
}

// Drive the speed motor for a (fractional) dial speed through the calibration.
static void sweepSpeedOutput(float speed)
{
#if SPP_HAS_VOLTAGE_CTRL
  applyMotorDrive(driveForSpeed(speed));
#else
  // Preset tables work in whole speeds: blend the duty of the two either side.
  const uint16_t whole = (uint16_t)speed;
  const float dutyLow = (float)speedToPwmDuty(whole);
  const float dutyHigh = (float)speedToPwmDuty(whole + 1);
  setMotorDutyRaw((uint32_t)lroundf(dutyLow + (speed - (float)whole) * (dutyHigh - dutyLow)));
#endif
}

static void sweepRpmOutput(float hz)
{
  frequencyRPM = lroundf(hz);
  setFrequencyRPM(hz);
}

// The sweep runs with the tasks suspended, so on the S3 it keeps the 100 ms
// measurements going itself: the rail over-voltage check and the tacho.
static void sweepWait(uint32_t ms)
{
  vTaskDelay(pdMS_TO_TICKS(ms));
#if SPP_HAS_VOLTAGE_CTRL
  static uint32_t sinceMeasure = 0;
  sinceMeasure += ms;
  if (sinceMeasure >= 100)
  {
    sinceMeasure = 0;
    updateMotorRail();
    updateMeasuredFreq();
  }
#endif
}

// One leg for both needles, up (climb) or down (the climb backwards).
static void sweepLeg(bool up, bool sweepSpeed, uint16_t lowSpeed, uint16_t topSpeed)
{
  const float travel = (float)(topSpeed - lowSpeed);
  float tSpeed = sweepSpeed ? 0.0f : 1.0f; // no calibration: the speed needle stays put
  float tRpm = 0.0f;
  while (tSpeed < 1.0f || tRpm < 1.0f)
  {
    const float pace = needleSweepSpeed < 1 ? 1.0f : (float)needleSweepSpeed;
    const float speedRate = stepSpeed < 1.0f ? 1.0f : stepSpeed;
    const float rpmRate = stepRPM < 1.0f ? 1.0f : stepRPM;
    const float dialSpan = maxSpeed < 10 ? 200.0f : (float)maxSpeed;
    const float speedLegMs = SWEEP_STEADY_RATE * travel * pace * (SWEEP_RATE_REF / speedRate);
    const float rpmLegMs = SWEEP_STEADY_RATE * dialSpan * pace * (SWEEP_RATE_REF / rpmRate);
    if (tSpeed < 1.0f)
    {
      tSpeed += (float)SWEEP_TICK_MS / speedLegMs;
      if (tSpeed > 1.0f)
        tSpeed = 1.0f;
      sweepSpeedOutput((float)lowSpeed + travel * sweepProfile(up ? tSpeed : 1.0f - tSpeed));
    }
    if (tRpm < 1.0f)
    {
      tRpm += (float)SWEEP_TICK_MS / rpmLegMs;
      if (tRpm > 1.0f)
        tRpm = 1.0f;
      sweepRpmOutput((float)maxRPM * sweepProfile(up ? tRpm : 1.0f - tRpm));
    }
    sweepWait(SWEEP_TICK_MS);
  }
}

// At the top: wait until the speed needle is really there before turning round.
// S3 with a tacho: until the motor reaches the top mark's speed; else a fixed hold.
static void sweepHoldAtTop(uint16_t topSpeed)
{
#if SPP_HAS_VOLTAGE_CTRL
  const bool tacho = feedbackAvailable && driveCalHzValid();
  const float arrivedHz = tacho ? SWEEP_TOP_ARRIVED * hzForSpeed((float)topSpeed) : 0.0f;
#else
  const bool tacho = false;
  const float arrivedHz = 0.0f;
  (void)topSpeed;
#endif
  uint32_t waited = 0;
  while (waited < (tacho ? SWEEP_TOP_WAIT_MAX_MS : SWEEP_TOP_HOLD_MS))
  {
    sweepWait(SWEEP_TICK_MS);
    waited += SWEEP_TICK_MS;
    if (tacho && measuredFreqRawHz >= arrivedHz)
      break;
  }
  vTaskDelay(pdMS_TO_TICKS(SWEEP_TOP_DWELL_MS));
}

void needleSweep()
{
  uint16_t lowSpeed = 0, topSpeed = 0;
  const bool sweepSpeed = sweepRange(lowSpeed, topSpeed);
  DEBUG_IO("needle sweep: speed %s %u..%u, tach 0..%u Hz", sweepSpeed ? "" : "(no calibration)",
           lowSpeed, topSpeed, (unsigned)maxRPM);
  sweepLeg(true, sweepSpeed, lowSpeed, topSpeed);
  sweepHoldAtTop(topSpeed);
  sweepLeg(false, sweepSpeed, lowSpeed, topSpeed);

  // Both outputs fully off: the motor coasts the speed needle home.
  needleTargetSpeed = 0;
  motorDutyApplied = 0;
  sweepSpeedOutput(0.0f);
  sweepRpmOutput(0.0f);
}
