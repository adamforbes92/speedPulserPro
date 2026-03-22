#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_motorCal.h"
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

// Interrupt handler for incoming frequency (RPM) reading
void incomingHz()
{
  static unsigned long previousMicros = micros();
  unsigned long presentMicros = micros();
  unsigned long revolutionTime = presentMicros - previousMicros;
  if (revolutionTime < 1000UL)
    return;
  dutyCycleIncoming = (60000000UL / revolutionTime) / 60;
  previousMicros = presentMicros;
  lastPulse = millis();
  ledCounter++;
}

// Interrupt handler for motor speed feedback reading
void incomingMotorSpeed()
{
  static unsigned long previousMicros = micros();
  unsigned long presentMicros = micros();
  unsigned long revolutionTime = presentMicros - previousMicros;
  if (revolutionTime < 1000UL)
    return;
  dutyCycleMotor = (60000000UL / revolutionTime) / 60;
  previousMicros = presentMicros;
  lastPulseRPM = millis();
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
  rpm_ledc_channel.gpio_num = (gpio_num_t)pinCoil;
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

// Find closest motor calibration value for target duty cycle
uint16_t findClosestMatch(uint16_t val)
{
  uint16_t closest = 0;
  uint16_t closest2 = 0;
  uint16_t i = 0;
  bool speedTest = false;

  for (i = 0; i < sizeof motorPerformance / sizeof motorPerformance[0]; i++)
  {
    if (motorPerformance[i] > 0)
    {
      if (abs(val) > motorPerformance[i])
      {
        speedTest = true;
        i = (sizeof motorPerformance / sizeof motorPerformance[0]);
      }
    }
  }

  if (speedTest)
  {
    for (i = 0; i < sizeof motorPerformance / sizeof motorPerformance[0]; i++)
    {
      if (abs(val - closest) >= abs(val - motorPerformance[i]))
      {
        closest = motorPerformance[i];
      }
    }

    for (i = 0; i < sizeof motorPerformance / sizeof motorPerformance[0]; i++)
    {
      if (motorPerformance[i] == closest)
      {
        closest2 = i;
        i = (sizeof motorPerformance / sizeof motorPerformance[0]);
      }
    }

    if (closest2 >= 385)
    {
      return 0;
    }
    else
    {
      return closest2;
    }
  }
  else
  {
    return 0;
  }
}
