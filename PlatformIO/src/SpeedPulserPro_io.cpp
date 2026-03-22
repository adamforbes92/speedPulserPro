#include "SpeedPulserPro_io.h"
#include "SpeedPulserPro_control.h"

void basicInit()
{
  DEBUG_PRINTLN("Initialising SpeedPulser...");

#if serialDebug
  DEBUG_PRINTLN("Reading EEPROM...");
#endif
  readEEP();
#if serialDebug
  DEBUG_PRINTLN("Read EEPROM!");
#endif

  DEBUG_PRINTLN("Setting up LED Output...");
  pinMode(pinOnboardLED, OUTPUT);
  digitalWrite(pinOnboardLED, ledOnboard);
  DEBUG_PRINTLN("Set up LED Output!");

  DEBUG_PRINTLN("Setting up Coil Output...");
  pinMode(pinCoil, OUTPUT);
  DEBUG_PRINTLN("Set up Coil Output!");

  DEBUG_PRINTLN("Setting up PWM (LEDC)...");
  // Configure LEDC timer for 10 kHz at 10-bit resolution
  ledc_timer_config_t ledc_timer;
  ledc_timer.speed_mode = LEDC_MODE;
  ledc_timer.timer_num = LEDC_TIMER;
  ledc_timer.freq_hz = LEDC_FREQUENCY;
  ledc_timer.duty_resolution = LEDC_RESOLUTION;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;

  ledc_timer_config(&ledc_timer);

  // Configure LEDC channel
  ledc_channel_config_t ledc_channel;
  ledc_channel.gpio_num = (gpio_num_t)pinMotorOutput;
  ledc_channel.speed_mode = LEDC_MODE;
  ledc_channel.channel = LEDC_OUTPUT_CHANNEL;
  ledc_channel.timer_sel = LEDC_TIMER;
  ledc_channel.duty = (uint32_t)dutyCycle;
  ledc_channel.hpoint = 0;
  ledc_channel_config(&ledc_channel);
  ledc_fade_func_install(0); // install LEDC hardware fade ISR (required once before any fade calls)

  DEBUG_PRINTLN("Set up PWM (LEDC)!");

  DEBUG_PRINTLN("Setting up Speed Interrupt...");
  attachInterrupt(digitalPinToInterrupt(pinSpeedInput), incomingHz, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinMotorInput), incomingMotorSpeed, FALLING);
  DEBUG_PRINTLN("Set up Speed Interrupt!");

  DEBUG_PRINTLN("Setting up CAN...");
  canInit();
  DEBUG_PRINTLN("Set up CAN!");

  DEBUG_PRINTLN("Setting up GPS Module...");
  ss.begin(baudGPS);
  DEBUG_PRINTLN("Set up GPS Module!");

  DEBUG_PRINTLN(TinyGPSPlus::libraryVersion());
  DEBUG_PRINTLN(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  DEBUG_PRINTLN(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  DEBUG_PRINTLN(F("----------------------------------------------------------------------------------------------------------------------------------------"));

  DEBUG_PRINTLN("Initialised SpeedPulser!");
}

void testSpeed()
{
  if (testCal)
  {
    ledcWrite(LEDC_OUTPUT_CHANNEL, tempDutyCycle);
#if serialDebug
    DEBUG_PRINTF("     Duty: %d", tempDutyCycle);
#endif
  }

  if (!testCal && tempSpeed > 0)
  {
#if serialDebug
    DEBUG_PRINTF("Chosen Speed: %d", tempSpeed);
#endif
    dutyCycle = applyConfiguredSpeedOffset((uint16_t)tempSpeed);
    dutyCycle = dutyCycle * speedMultiplier;
    if (convertToMPH)
    {
      dutyCycle = dutyCycle * mphFactor;
    }
    dutyCycle = findClosestMatch(dutyCycle);
    ledcWrite(LEDC_OUTPUT_CHANNEL, dutyCycle);
#if serialDebug
    DEBUG_PRINTF("  Final Duty: %d", dutyCycle);
    DEBUG_PRINTLN("");
#endif
  }
}

void needleSweep()
{
  // Maximum raw duty index in the calibration table (385 entries, 0-384)
  const uint16_t kMaxDuty = (sizeof motorPerformance / sizeof motorPerformance[0]) - 1; // 384
  const uint32_t kSweepPollMs = 10;

  // Fade durations: sweepSpeed (ms) is the primary rate controller.
  //   stepSpeed scales the Speed needle independently (10 = 1:1, higher = slower)
  //   stepRPM   scales the RPM   needle independently (10 = 1:1, higher = slower)
  // Matches legacy per-step delay: sweepSpeed × (step/10) × range.
  // e.g. defaults: 18ms × (10/10) × 200 steps = 3600 ms per direction.
  const uint32_t kFadeMsSpeed = (uint32_t)(stepSpeed * (float)sweepSpeed * (float)maxSpeed / 10.0f);
  const uint32_t kFadeMsRPMRaw = (uint32_t)(stepRPM * (float)sweepSpeed * (float)maxRPM / 10.0f);
  const uint32_t kFadeMsRPM = max<uint32_t>(kFadeMsRPMRaw, 1U);

  // Target RPM frequency at full deflection
  const long kMaxRpmFreq = (long)maxRPM;

  // ---- Ramp UP --------------------------------------------------------
  // Speed: LEDC hardware fade to full duty, rate controlled by Speed Step
  ledc_set_fade_with_time(LEDC_MODE, LEDC_OUTPUT_CHANNEL, kMaxDuty, kFadeMsSpeed);
  ledc_fade_start(LEDC_MODE, LEDC_OUTPUT_CHANNEL, LEDC_FADE_NO_WAIT);

  // RPM uses LEDC frequency with a fixed 50% duty cycle, so sweep the frequency directly.
  uint32_t rpmSweepStart = millis();
  while (millis() - rpmSweepStart < kFadeMsRPM)
  {
    uint32_t elapsed = millis() - rpmSweepStart;
    long currentFreq = (long)((kMaxRpmFreq * (long)elapsed) / (long)kFadeMsRPM);
    frequencyRPM = currentFreq;
    setFrequencyRPM(currentFreq);
    vTaskDelay(pdMS_TO_TICKS(kSweepPollMs));
  }
  frequencyRPM = kMaxRpmFreq;
  setFrequencyRPM(kMaxRpmFreq);

  // Pause at full deflection (sweepSpeed still controls the hold time)
  vTaskDelay(pdMS_TO_TICKS((uint32_t)sweepSpeed * 2));

  // ---- Ramp DOWN ------------------------------------------------------
  // Speed: LEDC hardware fade back to zero, rate controlled by Speed Step
  ledc_set_fade_with_time(LEDC_MODE, LEDC_OUTPUT_CHANNEL, 0, kFadeMsSpeed);
  ledc_fade_start(LEDC_MODE, LEDC_OUTPUT_CHANNEL, LEDC_FADE_NO_WAIT);

  rpmSweepStart = millis();
  while (millis() - rpmSweepStart < kFadeMsRPM)
  {
    uint32_t elapsed = millis() - rpmSweepStart;
    long currentFreq = kMaxRpmFreq - (long)((kMaxRpmFreq * (long)elapsed) / (long)kFadeMsRPM);
    frequencyRPM = currentFreq;
    setFrequencyRPM(currentFreq);
    vTaskDelay(pdMS_TO_TICKS(kSweepPollMs));
  }

  // Settle, then hard-zero both outputs
  vTaskDelay(pdMS_TO_TICKS((uint32_t)sweepSpeed * 2));
  dutyCycle = 0;
  ledcWrite(LEDC_OUTPUT_CHANNEL, 0);
  setFrequencyRPM(0);
  frequencyRPM = 0;
}
