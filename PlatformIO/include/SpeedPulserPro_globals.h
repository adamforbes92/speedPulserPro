#ifndef GLOBALS_H
#define GLOBALS_H

#include "Arduino.h"
#include "driver/ledc.h"
#include "soc/soc_caps.h"
#include "SpeedPulserPro_board.h" // SPP_BOARD_* / SPP_HAS_* (self-contained, safe to include first)
#include <RunningMedian.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <TinyGPSPlus.h>
#include <ESPAsyncWebServer.h>
#include <driver/twai.h>

// Note: config.h is NOT included here to avoid circular includes
// config.h includes this file (globals.h), so we can't include it back
// config.h must be included BEFORE globals.h in translation units

// ============================================================================
// Global Object Declarations
// ============================================================================
// LEDC PWM Control
#define LEDC_TIMER LEDC_TIMER_0
// The classic ESP32 has a high-speed LEDC timer group; the S3 only has the
// low-speed group, so the mode follows the chip's capabilities. (The RPM output
// is a GPTimer-toggled pin on both boards, so it never touches LEDC.)
#if SOC_LEDC_SUPPORT_HS_MODE
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#else
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#endif
#define LEDC_OUTPUT_CHANNEL LEDC_CHANNEL_0
#define LEDC_FREQUENCY (10000)
#define SPEED_OFFSET_CURVE_POINTS 5
extern RunningMedian samplesRPM;
extern HardwareSerial ss;
extern TinyGPSPlus gps;
extern Preferences settingsStore;
extern AsyncWebServer server;
extern TaskHandle_t taskCANRxHandle;

// ============================================================================
// RPM and PWM Variables
// ============================================================================
extern long frequencyRPM;
extern uint16_t needleTargetSpeed; // what the needle should show now (dial units, after offset + mph)
extern float hallFrequencyHz; // hall speed input (Hz), averaged over inputSmoothingMs
extern float rpmInputHz; // engine RPM input frequency (Hz), windowed average
#if SPP_HAS_VR
extern float vrFrequencyHz;   // VR speed input (Hz), averaged
#endif
extern uint16_t motorDutyApplied; // last duty value written to the motor (for the live curve marker)

// Motor direction: false = normal (pinMotorDirection LOW), true = reverse (HIGH)
extern bool reverseDirection;
void applyDirection(); // drive pinMotorDirection from reverseDirection (io.cpp)

#if SPP_HAS_SPEED_BIAS
// Speed-input bias (S3 board): the old PullDown/Up header is now two GPIO-driven
// switches. 0 = none (input floats — fine for push-pull sources such as Can2Cluster),
// 1 = 4.7k pull-up to VCC/12 V (open-collector hall sensors), 2 = 4.7k pull-down to GND.
// Only ever ONE switch is driven on; both on would put ~1.3 mA straight through R19+R46.
#define SPEED_BIAS_NONE      0
#define SPEED_BIAS_PULL_UP   1
#define SPEED_BIAS_PULL_DOWN 2
extern uint8_t speedInputBias;
void applySpeedInputBias(); // drive pinSpeedPullUp / pinSpeedPullDown from speedInputBias (io.cpp)
#endif

// ============================================================================
// Closed-Loop Motor Feedback (PID)
// ============================================================================
extern bool feedbackEnable;             // S3: hold the calibrated tacho speed with the drive trim
extern float feedbackDeadband;          // S3: trim ignores errors smaller than this (Hz)
extern uint16_t feedbackMinSpeed;       // S3: no trim below this needle speed (dial units)
extern uint16_t measuredSpeed;          // S3: motor tacho speed as a dial reading (UI/status)
extern float measuredFreqHz;            // EMA-smoothed tacho frequency (Hz)
extern float measuredFreqRawHz;         // un-smoothed tacho frequency (Hz), diagnostic
extern bool feedbackAvailable;          // true once a real tacho signal has been seen this session
extern bool feedbackMissing;            // true when the motor runs but no feedback signal is present (legacy PCB)

// ============================================================================
// Speed Variables
// ============================================================================
extern uint16_t vehicleRPMCAN;
extern uint16_t vehicleRPM;
extern uint16_t vehicleRPMHall;
extern float inputSpeedKph;      // the selected source's speed (km/h), in every mode (V5.09)
extern uint16_t vehicleSpeed;
extern uint16_t vehicleSpeedHall;
extern uint16_t hallSpeed;
#if SPP_HAS_VR
extern uint16_t vehicleSpeedVR;
extern uint16_t vrSpeed;
#endif
extern uint16_t ecuSpeed;
extern uint16_t dsgSpeed;
extern uint16_t gpsSpeed;
extern uint16_t absSpeed;
extern uint16_t udsSpeed;
extern uint16_t tp20Speed;

// ============================================================================
// Motor Performance and Limits
// ============================================================================
extern uint16_t presetSpeedTable[385];
extern bool updateMotorPerformance;
extern uint8_t presetId;
// Speed sensors are described by pulses per km: VW / Audi gearbox senders give
// 1 Hz per km/h = 3600 per km; 4000 / 8000 / 16000 pulses-per-mile sensors are
// 2485.5 / 4971 / 9942. Speed (km/h) = Hz x 3600 / pulses per km.
#define SENSOR_PULSES_PER_KM_DEFAULT 3600.0f
#define SENSOR_PULSES_PER_KM_MIN 100.0f
#define SENSOR_PULSES_PER_KM_MAX 50000.0f
#define INPUT_SMOOTHING_MS_DEFAULT 200 // the speed is averaged over at least this long
#define INPUT_SMOOTHING_MS_MIN 50
#define INPUT_SMOOTHING_MS_MAX 1000
extern float hallPulsesPerKm;
#if SPP_HAS_VR
extern float vrPulsesPerKm;
#endif
extern uint16_t inputSmoothingMs;
extern uint16_t maxSpeed;
extern uint16_t maxRPM;
extern uint16_t clusterRPMLimit;

// ============================================================================
// Speed Offset and Calibration
// ============================================================================
extern int16_t globalSpeedOffsetKph;  // fixed offset added to every speed (km/h, + or -)
extern bool useSpeedOffsetCurve;      // use the 5-point curve instead of the global offset
extern int16_t speedOffsetCurveOffsets[SPEED_OFFSET_CURVE_POINTS]; // at 25/75/125/175/225 km/h
extern float speedOffsetApplied;      // offset applied to the current speed (km/h)
extern float stepRPM;
extern float stepSpeed;
extern uint8_t averageFilterRPM;
extern float filteredRPMHz; // median-filtered RPM input (Hz), passed straight to the RPM output

// ============================================================================
// Test Variables
// ============================================================================
extern bool speedTestOn;
extern bool testRPM;
extern bool calibrationModeOn;
extern long speedTestKph;
extern long tempRPM;
extern long calibrationProbeDuty;
extern bool needleSweepRequested;

// ============================================================================
// Configuration Variables
// ============================================================================
extern bool needleSweepOnBoot;
extern bool coilType;
extern bool clusterInMph;
extern bool broadcastSpeedEnabled;
extern uint32_t broadcastSpeedID;
extern uint8_t broadcastSpeedDLC;
extern uint8_t broadcastSpeedLowByte;
extern uint8_t broadcastSpeedHighByte;
extern bool broadcastSpeedLittleEndian;
extern float broadcastSpeedScale;
extern int16_t broadcastSpeedOffset;
extern uint8_t broadcastSpeedData[8];
extern uint16_t broadcastSpeedValue;
extern uint8_t needleSweepSpeed;

// ============================================================================
// Speed Input Selection
// ============================================================================
extern bool useHall;                      // true = Hall sensor
#if SPP_HAS_VR
extern bool useVR;                        // true = variable-reluctance (VR) sensor (S3 board only)
#endif
extern bool useDSG;                       // true = DSG speed
extern bool useGPS;                       // true = GPS speed
extern bool useABS;                       // true = ABS speed
extern bool useECU;                       // true = ECU speed
extern bool useUDS;                       // true = UDS speed
extern bool useTP20;                      // true = TP2.0 speed
extern bool useAftermarket;               // speed input: custom CAN (aftermarket)
extern uint32_t aftermarketSpeedID;       // CAN ID to listen to
extern uint8_t aftermarketSpeedLowByte;   // byte index holding speed LSB
extern uint8_t aftermarketSpeedHighByte;  // byte index holding speed MSB
extern bool aftermarketSpeedLittleEndian; // true = LSB at lowByte index
extern float aftermarketSpeedScale;       // scale applied to raw value
extern int16_t aftermarketSpeedOffset;    // offset applied after scale
extern uint16_t aftermarketSpeed;         // parsed speed from aftermarket CAN frame

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
extern float dsgGearRatio[7];  // adjustable gear ratios, index 1..6 (DQ250 defaults)
extern float dsgFinalDrive14;  // final drive for gears 1-4
extern float dsgFinalDrive56;  // final drive for gears 5-6
extern float dsgTireCirc;      // tire circumference in metres

// ============================================================================
// Vehicle Status Variables
// ============================================================================

// ============================================================================
// Timing Variables
// ============================================================================
extern uint32_t lastCAN;
extern unsigned long lastPulseRPM;

// ============================================================================
// SavvyCAN / Analyser Variables
// ============================================================================
extern bool analyzerMode;        // WiFi GVRET/SLCAN (TCP port 23)
extern bool analyzerSerial;      // Serial GVRET (SavvyCAN over USB, 1 Mbaud)
extern uint8_t analyzerProtocol; // ANALYZER_PROTOCOL_GVRET or ANALYZER_PROTOCOL_LAWICEL

// ============================================================================
// System Status Variables
// ============================================================================
extern bool hasCAN;
extern bool hasGPS;
extern bool gpsUnavailable;
extern bool ledOnboard;
extern int ledCounter;
extern int rawCountRPM;

#endif // GLOBALS_H
