#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_config.h"
#include "SpeedPulserPro_savvycan.h"  // for ANALYZER_PROTOCOL_GVRET

// ============================================================================
// SavvyCAN / Analyzer
// ============================================================================
bool    analyzerMode     = false;
bool    analyzerSerial   = false;
uint8_t analyzerProtocol = ANALYZER_PROTOCOL_GVRET;

// ============================================================================
// Global Object Definitions
// ============================================================================
// LEDC PWM is configured via driver/ledc.h - no object needed
RunningMedian samplesRPM = RunningMedian(10); // for median filtering of RPM input
HardwareSerial ss(2);
TinyGPSPlus gps;
Preferences settingsStore;
AsyncWebServer server(80);
TaskHandle_t taskCANRxHandle = NULL;

// ============================================================================
// RPM and PWM Variables
// ============================================================================
long frequencyRPM = 20;
uint16_t needleTargetSpeed = 0;  // what the needle should show now (dial units)
float hallFrequencyHz = 0.0f;   // hall speed input (Hz), averaged
float rpmInputHz = 0.0f;
#if SPP_HAS_VR
float vrFrequencyHz = 0.0f;     // VR speed input (Hz), averaged
#endif
uint16_t motorDutyApplied = 0;

// Motor direction: false = normal (pinMotorDirection LOW), true = reverse (HIGH)
bool reverseDirection = false;

#if SPP_HAS_SPEED_BIAS
uint8_t speedInputBias = SPEED_BIAS_NONE; // S3 board: speed-input pull-up/down switch (persisted)
#endif

// ============================================================================
// Closed-Loop Motor Feedback (PID)
// ============================================================================
bool feedbackEnable = true;       // S3 drive trim (SpeedPulser V5.04 bench-tuned defaults)
float feedbackDeadband = 0.3f;    // trim ignores errors smaller than this (Hz)
uint16_t feedbackMinSpeed = 8;    // no trim below this needle speed (dial units)
uint16_t measuredSpeed = 0;
float measuredFreqHz = 0.0f;
float measuredFreqRawHz = 0.0f;
bool feedbackAvailable = false;   // true once a real tacho signal (GPIO feedback) has been seen this session
bool feedbackMissing = false;     // true when the motor runs but no feedback signal is present (legacy PCB)

// ============================================================================
// Speed Variables
// ============================================================================
uint16_t vehicleRPMCAN = 0;
uint16_t vehicleRPM = 0;
uint16_t vehicleRPMHall = 0;
float inputSpeedKph = 0.0f;       // the selected source's speed (km/h), in every mode (V5.09)
uint16_t vehicleSpeed = 0;
uint16_t vehicleSpeedHall = 0;
uint16_t hallSpeed = 0;
#if SPP_HAS_VR
uint16_t vehicleSpeedVR = 0;
uint16_t vrSpeed = 0;
#endif
uint16_t ecuSpeed = 0;
uint16_t dsgSpeed = 0;
uint16_t gpsSpeed = 0;
uint16_t absSpeed = 0;
uint16_t udsSpeed = 0;
uint16_t tp20Speed = 0;

// ============================================================================
// Motor Performance and Limits
// ============================================================================
uint16_t presetSpeedTable[385];
bool updateMotorPerformance = false;
uint8_t presetId = 1;
float hallPulsesPerKm = SENSOR_PULSES_PER_KM_DEFAULT; // hall speed sensor (3600 = 1 Hz per km/h)
#if SPP_HAS_VR
float vrPulsesPerKm = SENSOR_PULSES_PER_KM_DEFAULT;   // VR speed sensor
#endif
uint16_t inputSmoothingMs = INPUT_SMOOTHING_MS_DEFAULT;
uint16_t maxSpeed = 200;
uint16_t maxRPM = 230;
uint16_t clusterRPMLimit = 7000;

// ============================================================================
// Speed Offset and Calibration
// ============================================================================
int16_t globalSpeedOffsetKph = 0;  // fixed offset added to every speed (km/h, + or -)
bool useSpeedOffsetCurve = false;
int16_t speedOffsetCurveOffsets[SPEED_OFFSET_CURVE_POINTS] = {0, 0, 0, 0, 0}; // at 25/75/125/175/225 km/h
float speedOffsetApplied = 0.0f;   // offset applied to the current speed (km/h)
float stepRPM = 14;
float stepSpeed = 17;
uint8_t averageFilterRPM = DEFAULT_AVERAGE_FILTER_RPM;
float filteredRPMHz = 0.0f;

// ============================================================================
// Test Variables
// ============================================================================
bool speedTestOn = false;
bool testRPM = false;
bool calibrationModeOn = false;
long speedTestKph = 0;
long tempRPM = 0;
long calibrationProbeDuty = 0;
bool needleSweepRequested = false;

// ============================================================================
// Configuration Variables
// ============================================================================
bool needleSweepOnBoot = false;
bool coilType = true;
bool clusterInMph = false;
bool broadcastSpeedEnabled = false;
uint32_t broadcastSpeedID = HALDEX_ID;
uint8_t broadcastSpeedDLC = 8;
uint8_t broadcastSpeedLowByte = 2;
uint8_t broadcastSpeedHighByte = 3;
bool broadcastSpeedLittleEndian = true;
float broadcastSpeedScale = 0.781f; // ~1/1.28 — default Haldex scaling
int16_t broadcastSpeedOffset = 0;
uint8_t broadcastSpeedData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t broadcastSpeedValue = 0;
uint8_t needleSweepSpeed = 18;

// ============================================================================
// Speed Input Selection
// ============================================================================
bool useHall = true; // default source — matches the UI default and the first-run EEPROM write
#if SPP_HAS_VR
bool useVR = false;
#endif
bool useDSG = false;
bool useGPS = false;
bool useABS = false;
bool useECU = false;
bool useUDS = false;
bool useTP20 = false;
bool useAftermarket = false;
uint32_t aftermarketSpeedID = 0x200;
uint8_t aftermarketSpeedLowByte = 0;
uint8_t aftermarketSpeedHighByte = 1;
bool aftermarketSpeedLittleEndian = true;
float aftermarketSpeedScale = 1.0f;
int16_t aftermarketSpeedOffset = 0;
uint16_t aftermarketSpeed = 0;
bool useRPMHall = true;
bool useRPMCAN = false;

// ============================================================================
// DSG Variables
// ============================================================================
uint8_t gear = 0;
uint8_t lever = 0;
uint8_t gear_raw = 0;
uint8_t lever_raw = 0;
float dsgGearRatio[7] = {1.0f, 3.462f, 2.050f, 1.300f, 0.902f, 0.914f, 0.756f}; // index 0 unused
float dsgFinalDrive14 = 4.118f;
float dsgFinalDrive56 = 3.043f;
float dsgTireCirc = 1.885f; // PI * 0.6m rolling diameter

// ============================================================================
// Vehicle Status Variables
// ============================================================================

// ============================================================================
// Timing Variables
// ============================================================================
uint32_t lastCAN = 0;
unsigned long lastPulseRPM = 0;

// ============================================================================
// System Status Variables
// ============================================================================
bool hasCAN = false;
bool hasGPS = false;
bool gpsUnavailable = false;
bool ledOnboard = false;
int ledCounter = 0;
int rawCountRPM = 0;
