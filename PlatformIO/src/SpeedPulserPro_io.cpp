#include "SpeedPulserPro_io.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_gps.h"

// Drive the direction pin from the reverseDirection flag.
// Default (unticked) idles HIGH = normal; ticking Reverse pulls LOW.
void applyDirection()
{
  digitalWrite(pinMotorDirection, reverseDirection ? LOW : HIGH);
}

void basicInit()
{
  DEBUG_IO("Initialising SpeedPulser...");

  DEBUG_IO("Reading EEPROM...");
  readEEP();
  DEBUG_IO("Read EEPROM!");

  DEBUG_IO("Setting up LED Output...");
  pinMode(pinOnboardLED, OUTPUT);
  digitalWrite(pinOnboardLED, ledOnboard);
  DEBUG_IO("Set up LED Output!");

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
  ledc_channel.duty = (uint32_t)dutyCycle;
  ledc_channel.hpoint = 0;
  ledc_channel_config(&ledc_channel);
  ledc_fade_func_install(0); // install LEDC hardware fade ISR (required once before any fade calls)

  DEBUG_IO("Set up PWM (LEDC)!");

  DEBUG_IO("Setting up Speed Interrupt...");
  attachInterrupt(digitalPinToInterrupt(pinGearboxHall), incomingHz, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinEngineRPMInput), incomingMotorSpeed, FALLING);
  DEBUG_IO("Set up Speed Interrupt!");

  // Motor tacho feedback for the closed-loop PID.
  DEBUG_IO("Setting up Feedback Interrupt (GPIO%d)...", pinMotorFeedback);
  pinMode(pinMotorFeedback, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinMotorFeedback), feedbackPulse, FALLING);
  DEBUG_IO("Set up Feedback Interrupt!");

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
// appliedDutyCycle is kept in the full 12-bit domain so the live cal-curve marker
// and the calibration builder share one duty scale (mirrors SpeedPulser).
void setMotorDutyRaw(uint32_t pwmDuty)
{
  appliedDutyCycle = (uint16_t)pwmDuty;
  ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_CHANNEL, pwmDuty);
  ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_CHANNEL);
}

void testSpeed()
{
  if (testCal)
  {
    setMotorDutyRaw(tempDutyCycle);
#if serialDebug
    DEBUG_PRINTF("[IO] Test Duty: %d", tempDutyCycle);
#endif
  }

  if (!testCal && tempSpeed > 0)
  {
#if serialDebug
    DEBUG_PRINTF("[IO] Chosen Speed: %d", tempSpeed);
#endif
    uint16_t spd = applyConfiguredSpeedOffset((uint16_t)tempSpeed);
    spd = spd * speedMultiplier;
    if (convertToMPH)
    {
      spd = spd * mphFactor;
    }
    dutyCycle = findClosestMatch(spd);   // cal-domain duty (display)
    setMotorDutyRaw(speedToPwmDuty(spd)); // 12-bit hardware duty applied
#if serialDebug
    DEBUG_PRINTF("[IO]   Final Duty: %ld", dutyCycle);
    DEBUG_PRINTLN("");
#endif
  }
}

void needleSweep()
{
  // Startup needle exercise for a KNOWN two-dial system. Both dials share the same
  // physical endpoints: a common REST (0) and a common FULL-SCALE marking. We drive
  // each needle across its OWN dial using its calibration so it lands exactly on the
  // top-of-scale marking instead of slamming past it into the mechanical stop:
  //   Speed dial: 0 .. maxSpeed km/h  -> duty via speedToPwmDuty() (the cal curve)
  //   RPM  dial : 0 .. maxRPM Hz       -> full-scale tach deflection (linear)
  // The two needles are DIFFERENT actuators (BLDC motor vs air-core tach coil) and
  // do not track at the same rate, so each has its OWN traversal rate (stepSpeed /
  // stepRPM) between the shared endpoints. Durations are recomputed per tick from
  // the LIVE sliders, so dragging sweepSpeed / stepSpeed / stepRPM retimes an
  // in-progress sweep smoothly.
  const uint32_t kPollMs     = 10;                            // software-fade cadence
  const long     kSpan       = (maxSpeed < 10) ? 200 : maxSpeed; // sets sweep DURATION only
  const uint16_t kMaxSpeed   = (maxSpeed < 10) ? 10 : maxSpeed;  // speed-dial full scale (km/h)
  const long     kMaxRpmFreq = (long)maxRPM;                  // RPM-dial full scale (Hz)
  const float    kStepRef    = 17.0f;                         // reference step: keeps default speed timing unchanged

  // Move each needle position from its normalised progress (0..1) along its dial.
  auto driveNeedles = [&](float speedP, float rpmP)
  {
    setMotorDutyRaw(speedToPwmDuty((uint16_t)((float)kMaxSpeed * speedP)));
    frequencyRPM = (long)((float)kMaxRpmFreq * rpmP);
    setFrequencyRPM(frequencyRPM);
  };

  // Ramp UP: both needles from REST (0) to their FULL-SCALE marking (1).
  float speedP = 0.0f, rpmP = 0.0f;
  while (speedP < 1.0f || rpmP < 1.0f)
  {
    float sSpeed = (stepSpeed < 1.0f) ? 1.0f : stepSpeed;
    float sRpm   = (stepRPM   < 1.0f) ? 1.0f : stepRPM;
    uint32_t speedFullMs = (uint32_t)((float)sweepSpeed * (float)kSpan * (kStepRef / sSpeed));
    uint32_t rpmFullMs   = (uint32_t)((float)sweepSpeed * (float)kSpan * (kStepRef / sRpm));
    if (speedFullMs < 1) speedFullMs = 1;
    if (rpmFullMs   < 1) rpmFullMs   = 1;
    speedP += (float)kPollMs / (float)speedFullMs; if (speedP > 1.0f) speedP = 1.0f;
    rpmP   += (float)kPollMs / (float)rpmFullMs;   if (rpmP   > 1.0f) rpmP   = 1.0f;
    driveNeedles(speedP, rpmP);
    vTaskDelay(pdMS_TO_TICKS(kPollMs));
  }
  driveNeedles(1.0f, 1.0f); // pin both needles on their full-scale markings

  // Pause at full scale
  vTaskDelay(pdMS_TO_TICKS((uint32_t)sweepSpeed * 2));

  // Ramp DOWN — mirror of the ramp-up, back to the shared REST point.
  speedP = 0.0f; rpmP = 0.0f;
  while (speedP < 1.0f || rpmP < 1.0f)
  {
    float sSpeed = (stepSpeed < 1.0f) ? 1.0f : stepSpeed;
    float sRpm   = (stepRPM   < 1.0f) ? 1.0f : stepRPM;
    uint32_t speedFullMs = (uint32_t)((float)sweepSpeed * (float)kSpan * (kStepRef / sSpeed));
    uint32_t rpmFullMs   = (uint32_t)((float)sweepSpeed * (float)kSpan * (kStepRef / sRpm));
    if (speedFullMs < 1) speedFullMs = 1;
    if (rpmFullMs   < 1) rpmFullMs   = 1;
    speedP += (float)kPollMs / (float)speedFullMs; if (speedP > 1.0f) speedP = 1.0f;
    rpmP   += (float)kPollMs / (float)rpmFullMs;   if (rpmP   > 1.0f) rpmP   = 1.0f;
    driveNeedles(1.0f - speedP, 1.0f - rpmP);
    vTaskDelay(pdMS_TO_TICKS(kPollMs));
  }

  // Settle, then hard-zero both outputs
  vTaskDelay(pdMS_TO_TICKS((uint32_t)sweepSpeed * 2));
  dutyCycle = 0;
  appliedDutyCycle = 0;
  setMotorDuty(0);    // speed output fully off
  frequencyRPM = 0;
  setFrequencyRPM(0); // RPM output fully off
}
