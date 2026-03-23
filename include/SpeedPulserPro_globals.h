#ifndef GLOBALS_H
#define GLOBALS_H

#include "Arduino.h"
#include "driver/ledc.h"
#include <RunningMedian.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ESPAsyncWebServer.h>
#include <driver/twai.h>

// Note: config.h is NOT included here to avoid circular includes
// config.h includes this file (globals.h), so we can't include it back
// config.h must be included BEFORE globals.h in translation units

// ============================================================================
// Global Object Declarations
// ============================================================================
// LEDC PWM Control
#define LEDC_TIMER           LEDC_TIMER_0
#define LEDC_MODE            LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_CHANNEL  LEDC_CHANNEL_0
#define LEDC_RESOLUTION      LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY       (10000)
#define LEDC_RPM_TIMER       LEDC_TIMER_1
#define LEDC_RPM_CHANNEL     LEDC_CHANNEL_1
#define LEDC_RPM_DUTY_50     (1U << ((uint32_t)LEDC_RESOLUTION - 1U))
#define SPEED_OFFSET_CURVE_POINTS 5
extern RunningMedian samples;
extern RunningMedian samplesRPM;
extern SoftwareSerial ss;
extern TinyGPSPlus gps;
extern Preferences pref;
extern AsyncWebServer server;
extern hw_timer_t* timer0;
extern TaskHandle_t taskCANRxHandle;

// ============================================================================
// RPM and PWM Variables
// ============================================================================
extern bool rpmTrigger;
extern long frequencyRPM;
extern long pwmFrequency;
extern long dutyCycle;
extern int pwmResolution;
extern unsigned long dutyCycleIncoming;
extern unsigned long dutyCycleMotor;

// ============================================================================
// Speed Variables
// ============================================================================
extern uint16_t vehicleRPMCAN;
extern uint16_t vehicleRPM;
extern uint16_t vehicleRPMHall;
extern uint16_t calcSpeed;
extern uint16_t vehicleSpeed;
extern uint16_t vehicleSpeedHall;
extern uint16_t vehicleSpeedCAN;
extern uint16_t vehicleSpeedGPS;
extern uint16_t hallSpeed;
extern uint16_t ecuSpeed;
extern uint16_t dsgSpeed;
extern uint16_t gpsSpeed;
extern uint16_t absSpeed;
extern uint16_t udsSpeed;

// ============================================================================
// Motor Performance and Limits
// ============================================================================
extern uint16_t motorPerformance[385];
extern bool updateMotorPerformance;
extern uint8_t motorPerformanceVal;
extern uint16_t maxFreqHall;
extern uint16_t maxFreqCAN;
extern uint16_t maxSpeed;
extern uint16_t maxRPM;
extern uint16_t clusterRPMLimit;

// ============================================================================
// Speed Offset and Calibration
// ============================================================================
extern uint8_t speedOffset;
extern bool speedOffsetPositive;
extern bool useGlobalSpeedOffset;
extern bool useSpeedOffsetCurve;
extern int16_t speedOffsetCurveOffsets[SPEED_OFFSET_CURVE_POINTS];
extern int16_t currentSpeedOffset;
extern float stepRPM;
extern float stepSpeed;
extern uint8_t averageFilterHall;
extern uint8_t averageFilterRPM;
extern uint16_t filteredRPM;

// ============================================================================
// Test Variables
// ============================================================================
extern bool testSpeedo;
extern bool testRPM;
extern bool testCal;
extern long tempSpeed;
extern long tempRPM;
extern long tempDutyCycle;
extern bool tempNeedleSweep;

// ============================================================================
// Configuration Variables
// ============================================================================
extern bool hasNeedleSweep;
extern bool coilType;
extern bool broadcastSpeed;
extern uint8_t sweepSpeed;
extern uint8_t speedType;

// ============================================================================
// Speed Input Selection
// ============================================================================
extern bool useHall;
extern bool useDSG;
extern bool useGPS;
extern bool useABS;
extern bool useECU;
extern bool useUDS;

// RPM Input Selection
extern bool useRPMHall;
extern bool useRPMCAN;

// ============================================================================
// DSG Variables
// ============================================================================
extern uint8_t gear;
extern uint8_t lever;
extern uint8_t gear_raw;
extern uint8_t lever_raw;

// ============================================================================
// Vehicle Status Variables
// ============================================================================
extern bool vehicleEML;
extern bool vehicleEPC;
extern bool vehicleReverse;
extern bool vehiclePark;

// ============================================================================
// Timing Variables
// ============================================================================
extern uint32_t lastMillis;
extern uint32_t lastMillis2;
extern uint32_t lastCAN;
extern unsigned long lastPulse;
extern unsigned long lastPulseRPM;

// ============================================================================
// System Status Variables
// ============================================================================
extern bool hasError;
extern bool hasCAN;
extern bool hasGPS;
extern bool gpsTaskSuspended;
extern bool ledOnboard;
extern int ledCounter;
extern int rawCount;
extern int rawCountRPM;

#endif // GLOBALS_H
