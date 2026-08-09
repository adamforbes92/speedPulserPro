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
RunningMedian samples = RunningMedian(10); // for median filtering of speed input (Hall)
RunningMedian samplesRPM = RunningMedian(10); // for median filtering of RPM input
HardwareSerial ss(2);
TinyGPSPlus gps;
Preferences pref;
AsyncWebServer server(80);
hw_timer_t* timer0 = NULL;
TaskHandle_t taskCANRxHandle = NULL;

// ============================================================================
// RPM and PWM Variables
// ============================================================================
bool rpmTrigger = true;
long frequencyRPM = 20;
long pwmFrequency = 10000;
long dutyCycle = 0;
int pwmResolution = 12;
unsigned long dutyCycleIncoming = 0;
unsigned long dutyCycleMotor = 0;
uint16_t appliedDutyCycle = 0;

// Motor direction: false = normal (pinMotorDirection LOW), true = reverse (HIGH)
bool reverseDirection = false;

// ============================================================================
// Closed-Loop Motor Feedback (PID)
// ============================================================================
volatile uint32_t feedbackCount = 0;
bool feedbackEnable = false;
float pidKp = 0.15f;
float pidKi = 1.3f;
float pidKd = 0.0f;
float feedbackDeadband = 1.5f;    // PID deadband (Hz); within this error P/D are silenced (integral still trims). 0 = off
uint16_t feedbackMaxFreq = 254;   // bench-baked: motor tacho Hz at maxSpeed
uint16_t feedbackMinSpeed = 40;   // kph; below this target the loop runs open-loop
uint16_t measuredSpeed = 0;
int16_t pidCorrection = 0;
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
uint16_t calcSpeed = 0;
uint16_t vehicleSpeed = 0;
uint16_t vehicleSpeedHall = 0;
uint16_t vehicleSpeedCAN = 0;
uint16_t vehicleSpeedGPS = 0;
uint16_t hallSpeed = 0;
uint16_t ecuSpeed = 0;
uint16_t dsgSpeed = 0;
uint16_t gpsSpeed = 0;
uint16_t absSpeed = 0;
uint16_t udsSpeed = 0;
uint16_t tp20Speed = 0;

// ============================================================================
// Motor Performance and Limits
// ============================================================================
uint16_t motorPerformance[385];
bool updateMotorPerformance = false;
uint8_t motorPerformanceVal = 1;
uint16_t maxFreqHall = 200;
uint16_t maxFreqCAN = 200;
uint16_t maxSpeed = 200;
uint16_t maxRPM = 230;
uint16_t clusterRPMLimit = 7000;

// ============================================================================
// Speed Offset and Calibration
// ============================================================================
uint8_t speedOffset = 0;
bool speedOffsetPositive = false;
bool useGlobalSpeedOffset = true;
bool useSpeedOffsetCurve = false;
int16_t speedOffsetCurveOffsets[SPEED_OFFSET_CURVE_POINTS] = {0, 0, 0, 0, 0};
int16_t currentSpeedOffset = 0;
float stepRPM = 14;
float stepSpeed = 17;
uint8_t averageFilterHall = DEFAULT_AVERAGE_FILTER_HALL;
uint8_t averageFilterRPM = DEFAULT_AVERAGE_FILTER_RPM;
uint16_t filteredRPM = 0;

// ============================================================================
// Test Variables
// ============================================================================
bool testSpeedo = false;
bool testRPM = false;
bool testCal = false;
long tempSpeed = 0;
long tempRPM = 0;
long tempDutyCycle = 0;
bool tempNeedleSweep = false;

// ============================================================================
// Configuration Variables
// ============================================================================
bool hasNeedleSweep = false;
bool linearSpeedSweep = true;
bool coilType = true;
bool convertToMPH = false;
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
uint8_t sweepSpeed = 18;
uint8_t speedType = 0;

// ============================================================================
// Speed Input Selection
// ============================================================================
bool useHall = false;
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

// ============================================================================
// Vehicle Status Variables
// ============================================================================
bool vehicleEML = false;
bool vehicleEPC = false;
bool vehicleReverse = false;
bool vehiclePark = false;

// ============================================================================
// Timing Variables
// ============================================================================
uint32_t lastMillis = 0;
uint32_t lastMillis2 = 0;
uint32_t lastCAN = 0;
unsigned long lastPulse = 0;
unsigned long lastPulseRPM = 0;

// ============================================================================
// System Status Variables
// ============================================================================
bool hasError = false;
bool hasCAN = false;
bool hasGPS = false;
bool gpsUnavailable = false;
bool ledOnboard = false;
int ledCounter = 0;
int rawCount = 0;
int rawCountRPM = 0;
