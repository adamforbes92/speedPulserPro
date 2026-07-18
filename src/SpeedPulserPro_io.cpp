#include "SpeedPulserPro_io.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_gps.h"

void basicInit()
{
  DEBUG_PRINTLN("[IO] Initialising SpeedPulser...");

#if serialDebug
  DEBUG_PRINTLN("[IO] Reading EEPROM...");
#endif
  readEEP();
#if serialDebug
  DEBUG_PRINTLN("[IO] Read EEPROM!");
#endif

  DEBUG_PRINTLN("[IO] Setting up LED Output...");
  pinMode(pinOnboardLED, OUTPUT);
  digitalWrite(pinOnboardLED, ledOnboard);
  DEBUG_PRINTLN("[IO] Set up LED Output!");

  DEBUG_PRINTLN("[IO] Setting up Coil Output...");
  pinMode(pinCoil, OUTPUT);
  DEBUG_PRINTLN("[IO] Set up Coil Output!");

  DEBUG_PRINTLN("[IO] Setting up PWM (LEDC)...");
  // Configure LEDC timer for 10 kHz at 10-bit resolution
  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_MODE;
  ledc_timer.timer_num = LEDC_TIMER;
  ledc_timer.freq_hz = LEDC_FREQUENCY;
  ledc_timer.duty_resolution = LEDC_RESOLUTION;
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

  DEBUG_PRINTLN("[IO] Set up PWM (LEDC)!");

  DEBUG_PRINTLN("[IO] Setting up Speed Interrupt...");
  attachInterrupt(digitalPinToInterrupt(pinSpeedInput), incomingHz, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinMotorInput), incomingMotorSpeed, FALLING);
  DEBUG_PRINTLN("[IO] Set up Speed Interrupt!");

  DEBUG_PRINTLN("[IO] Setting up CAN...");
  canInit();
  DEBUG_PRINTLN("[IO] Set up CAN!");

#if serialDebugGPS
  Serial.println("[GPS] Setting up GPS Module...");
#endif
  initGPS();
#if serialDebugGPS
  Serial.println("[GPS] Set up GPS Module!");

  Serial.print("[GPS] TinyGPS++ version: ");
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
#endif

  DEBUG_PRINTLN("[IO] Initialised SpeedPulser!");
}

// Write the motor PWM duty via the ESP-IDF LEDC driver. The channel is set up
// with ledc_channel_config() above, so duty must be written the IDF way too.
// (Arduino-ESP32 3.x made ledcWrite() pin-based and it no longer recognises
// channels created outside its own ledcAttach() bookkeeping.)
void setMotorDuty(uint32_t duty)
{
  ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_CHANNEL, duty);
  ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_CHANNEL);
}

void testSpeed()
{
  if (testCal)
  {
    setMotorDuty(tempDutyCycle);
#if serialDebug
    DEBUG_PRINTF("[IO] Test Duty: %d", tempDutyCycle);
#endif
  }

  if (!testCal && tempSpeed > 0)
  {
#if serialDebug
    DEBUG_PRINTF("[IO] Chosen Speed: %d", tempSpeed);
#endif
    dutyCycle = applyConfiguredSpeedOffset((uint16_t)tempSpeed);
    dutyCycle = dutyCycle * speedMultiplier;
    if (convertToMPH)
    {
      dutyCycle = dutyCycle * mphFactor;
    }
    dutyCycle = findClosestMatch(dutyCycle);
    setMotorDuty(dutyCycle);
#if serialDebug
    DEBUG_PRINTF("[IO]   Final Duty: %d", dutyCycle);
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
  const uint32_t kFadeMsSpeedRaw = (uint32_t)(stepSpeed * (float)sweepSpeed * (float)maxSpeed / 10.0f);
  const uint32_t kFadeMsRPMRaw = (uint32_t)(stepRPM * (float)sweepSpeed * (float)maxRPM / 10.0f);
  const uint32_t kFadeMsSpeed = max<uint32_t>(kFadeMsSpeedRaw, 1U);
  const uint32_t kFadeMsRPM = max<uint32_t>(kFadeMsRPMRaw, 1U);

  // Target RPM frequency at full deflection
  const long kMaxRpmFreq = (long)maxRPM;
  // Full-deflection target for the speed needle in display units (kph)
  const long kMaxSpeedDisplay = (long)maxSpeed;

  // ---- Ramp UP --------------------------------------------------------
  if (linearSpeedSweep)
  {
    // Linearised: sweep the *displayed* speed linearly over time and look up
    // the duty for each target via the calibration table. RPM is driven from
    // the same tick loop so both needles ramp concurrently.
    const uint32_t kFadeMsMax = max(kFadeMsSpeed, kFadeMsRPM);
    uint32_t sweepStart = millis();
    while (millis() - sweepStart < kFadeMsMax)
    {
      uint32_t elapsed = millis() - sweepStart;

      if (elapsed < kFadeMsSpeed)
      {
        long targetSpeed = (kMaxSpeedDisplay * (long)elapsed) / (long)kFadeMsSpeed;
        uint16_t duty = findClosestMatch((uint16_t)targetSpeed);
        setMotorDuty(duty);
      }

      if (elapsed < kFadeMsRPM)
      {
        long currentFreq = (kMaxRpmFreq * (long)elapsed) / (long)kFadeMsRPM;
        frequencyRPM = currentFreq;
        setFrequencyRPM(currentFreq);
      }

      vTaskDelay(pdMS_TO_TICKS(kSweepPollMs));
    }
    setMotorDuty(kMaxDuty);
    frequencyRPM = kMaxRpmFreq;
    setFrequencyRPM(kMaxRpmFreq);
  }
  else
  {
    // Legacy: LEDC hardware fade ramps duty linearly (needle climbs non-linearly).
    // Non-blocking, so the RPM software loop below runs concurrently.
    ledc_set_fade_with_time(LEDC_MODE, LEDC_OUTPUT_CHANNEL, kMaxDuty, kFadeMsSpeed);
    ledc_fade_start(LEDC_MODE, LEDC_OUTPUT_CHANNEL, LEDC_FADE_NO_WAIT);

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
  }

  // Pause at full deflection (sweepSpeed still controls the hold time)
  vTaskDelay(pdMS_TO_TICKS((uint32_t)sweepSpeed * 2));

  // ---- Ramp DOWN ------------------------------------------------------
  if (linearSpeedSweep)
  {
    // Mirror of the ramp-up combined loop: both needles ramp concurrently.
    const uint32_t kFadeMsMax = max(kFadeMsSpeed, kFadeMsRPM);
    uint32_t sweepStart = millis();
    while (millis() - sweepStart < kFadeMsMax)
    {
      uint32_t elapsed = millis() - sweepStart;

      if (elapsed < kFadeMsSpeed)
      {
        long targetSpeed = kMaxSpeedDisplay - (kMaxSpeedDisplay * (long)elapsed) / (long)kFadeMsSpeed;
        if (targetSpeed < 0) targetSpeed = 0;
        uint16_t duty = findClosestMatch((uint16_t)targetSpeed);
        setMotorDuty(duty);
      }

      if (elapsed < kFadeMsRPM)
      {
        long currentFreq = kMaxRpmFreq - (kMaxRpmFreq * (long)elapsed) / (long)kFadeMsRPM;
        frequencyRPM = currentFreq;
        setFrequencyRPM(currentFreq);
      }

      vTaskDelay(pdMS_TO_TICKS(kSweepPollMs));
    }
    setMotorDuty(0);
  }
  else
  {
    // Legacy: non-blocking LEDC hardware fade back to zero, RPM loop runs concurrently.
    ledc_set_fade_with_time(LEDC_MODE, LEDC_OUTPUT_CHANNEL, 0, kFadeMsSpeed);
    ledc_fade_start(LEDC_MODE, LEDC_OUTPUT_CHANNEL, LEDC_FADE_NO_WAIT);

    uint32_t rpmSweepStart = millis();
    while (millis() - rpmSweepStart < kFadeMsRPM)
    {
      uint32_t elapsed = millis() - rpmSweepStart;
      long currentFreq = kMaxRpmFreq - (long)((kMaxRpmFreq * (long)elapsed) / (long)kFadeMsRPM);
      frequencyRPM = currentFreq;
      setFrequencyRPM(currentFreq);
      vTaskDelay(pdMS_TO_TICKS(kSweepPollMs));
    }
  }

  // Settle, then hard-zero both outputs
  vTaskDelay(pdMS_TO_TICKS((uint32_t)sweepSpeed * 2));
  dutyCycle = 0;
  setMotorDuty(0);
  setFrequencyRPM(0);
  frequencyRPM = 0;
}
