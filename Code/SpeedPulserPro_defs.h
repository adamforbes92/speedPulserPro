#define _PWM_LOGLEVEL_ 0
#include "ESP32_FastPWM.h"
#include <RunningMedian.h>
#include <Arduino.h>
#include "TickTwo.h"      // for repeated tasks
#include <Preferences.h>  // for eeprom/remember settings
#include <ESPUI.h>
#include <WiFi.h>
#include <ESPmDNS.h>
// for OTA
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncHTTPUpdateServer.h>

#define baudSerial 115200              // baud rate for serial feedback
#define serialDebug 0                  // for Serial feedback - disable on release(!) ** CAN CHANGE THIS **
#define serialDebugIncoming 0          //
#define serialDebugWifi 0              // for wifi feedback
#define serialDebugEEP 0               //
#define serialDebugGPS 0               //
#define ChassisCANDebug 0              // if 1, will print CAN 2 (Chassis) messages ** CAN CHANGE THIS **
#define eepRefresh 5000                // EEPROM Refresh in ms
#define labelRefresh 200                // EEPROM Refresh in ms
#define wifiDisable 60000              // turn off WiFi in ms
#define wifiHostName "SpeedPulserPro"  // the WiFi name

extern bool testSpeedo = false;  // for testing only, vary final pwmFrequency for speed - disable on release(!) ** CAN CHANGE THIS **
extern bool testRPM = false;
extern bool testCal = false;         // for testing only, vary final pwmFrequency for speed ** CAN CHANGE THIS **

extern bool hasNeedleSweep = false;  // for needle sweep ** CAN CHANGE THIS **
extern uint8_t sweepSpeed = 18;      // for needle sweep rate of change (in ms) ** CAN CHANGE THIS **
extern uint8_t speedType = 0;        // 0 = ECU, 1 = DSG, 2 = GPS, 3 = ABS

#define incomingType 0      // 0 = CAN; 1 = hall sensor  ** CAN CHANGE THIS **
#define averageFilter 6     // number of samples to take to average/remove erraticness from freq. changes.  Higher number, more samples ** CAN CHANGE THIS **
#define durationReset 1500  // duration of 'last sample' before reset speed back to zero

extern uint16_t maxFreqHall = 200;  // max frequency for top speed using the 02J / 02M hall sensor ** CAN CHANGE THIS **
extern uint16_t maxFreqCAN = 200;   // max frequency for top speed using the 02J / 02M hall sensor ** CAN CHANGE THIS **
extern uint16_t maxSpeed = 200;  // minimum cluster speed in kmh on the cluster ** CAN CHANGE THIS **
extern uint16_t maxRPM = 230;    // minimum cluster speed in kmh on the cluster ** CAN CHANGE THIS **

extern uint16_t clusterRPMLimit = 7000;  // rpm ** CAN CHANGE THIS **

// setup - step changes (for needle sweep)
extern float stepRPM = 12;
extern float stepSpeed = 10;
extern uint8_t speedOffset = 0;           // for adjusting a GLOBAL FIXED speed offset - so the entire range is offset by X value.  Might be easier to use this than the input max freq.
extern bool speedOffsetPositive = false;  // set to 1 for the above value to be ADDED, set to zero for the above value to be SUBTRACTED
#define speedMultiplier 1
#define convertToMPH 0  // for speedos that ONLY read in mph (and therefore the calibration is in mph), change KMH to find MPH
#define mphFactor 0.621371

#define pinMotorOutput 21  // pin for motor PWM output - needs stepped up to 5v for the motor (NPN transistor on the board).  Needs to support LED PWM(!)
#define pinMotorInput 18   // pin for motor speed input.  Assumed 5v, might be bad, should have checked(!)...
#define pinSpeedInput 26   // interrupt supporting pin for hall speed input
#define pinDirection 19    // motor direction pin (currently unused) but here for future revisions
#define pinOnboardLED 2    // for feedback for input checking / flash LED on input.  ESP32 C3 is Pin 8.  Devkit is 2

#define pinRX_CAN 17  // pin output for SN65HVD230 (CAN_RX)
#define pinTX_CAN 16  // pin output for SN65HVD230 (CAN_TX)
#define pinRX_GPS 14  // pin output for GPS NEO6M (GPS_RX)
#define pinTX_GPS 13  // pin output for GPS NEO6M (GPS_TX)
#define pinCoil 22    // pin output for RPM (MK2/High Output Coil Trigger)

#ifdef serialDebug
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(x...) Serial.printf(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(x...)
#endif

// Baud Rates
#define baudSerial 9600  // baud rate for debug
#define baudGPS 9600     // baud rate for the GPS device
extern uint16_t vehicleRPMCAN = 0;
extern uint16_t vehicleRPM = 0;  // current RPM.  If no CAN, this will catch dividing by zero by the map function
extern uint16_t vehicleRPMHall = 0;

extern uint16_t calcSpeed = 0;     // temp var for calculating speed
extern uint16_t vehicleSpeed = 0;  // current Speed.  If no CAN, this will catch dividing by zero by the map function
extern uint16_t vehicleSpeedHall = 0;
extern uint16_t vehicleSpeedCAN = 0;  // current Speed.  If no CAN, this will catch dividing by zero by the map function
extern uint16_t vehicleSpeedGPS = 0;  // current Speed.  If no CAN, this will catch dividing by zero by the map function
extern bool tempNeedleSweep = false;
extern long tempDutyCycle = 0;

uint16_t motorPerformance[385];  // for copying the motorPerformance data on selection of calibration value
extern bool updateMotorPerformance = false;
extern uint8_t motorPerformanceVal = 0;

// DSG variables
#define PI 3.141592653589793
#define LEVER_P 0x8               // park position
#define LEVER_R 0x7               // reverse position
#define LEVER_N 0x6               // neutral position
#define LEVER_D 0x5               // drive position
#define LEVER_S 0xC               // spot position
#define LEVER_TIPTRONIC_ON 0xE    // tiptronic
#define LEVER_TIPTRONIC_UP 0xA    // tiptronic up
#define LEVER_TIPTRONIC_DOWN 0xB  // tiptronic down
#define gearPause 20              // Send packets every x ms ** CAN CHANGE THIS **
#define rpmPause 50

extern double hallSpeed = 0;  // ECU speed (from analog speed sensor)
extern double ecuSpeed = 0;  // ECU speed (from analog speed sensor)
extern double dsgSpeed = 0;  // DSG speed (from RPM & Gear), ratios in '_dsg.ino'
extern double gpsSpeed = 0;  // GPS speed (from '_gps.ino')
extern double absSpeed = 0;  // ABS speed (from '_gps.ino')

extern bool useHall = false;
extern bool useDSG = false;
extern bool useGPS = false;
extern bool useABS = false;
extern bool useECU = false;

// DSG variables
extern uint8_t gear = 0;   // current gear from DSG
extern uint8_t lever = 0;  // shifter position
extern uint8_t gear_raw = 0;
extern uint8_t lever_raw = 0;
uint32_t lastMillis = 0;  // Counter for sending frames x ms
uint32_t lastMillis2 = 0;
uint32_t lastCAN = 0;

// ECU variables
extern bool vehicleEML = false;  // current EML light status
extern bool vehicleEPC = false;  // current EPC light status
extern bool vehicleReverse = false;
extern bool vehiclePark = false;

extern unsigned long dutyCycleIncoming = 0;  // Duty Cycle % coming in from Can2Cluster or Hall
extern unsigned long dutyCycleMotor = 0;     // Duty Cycle % coming in from Can2Cluster or Hall
extern long tempSpeed = 0;                   // for testing only, set fixed speed in kmh.  Can set to 0 to speed up / slow down on repeat with testSpeed enabled
extern long tempRPM = 0;                     // for testing only, set fixed speed in kmh.  Can set to 0 to speed up / slow down on repeat with testSpeed enabled
extern long pwmFrequency = 10000;            // PWM Hz (motor supplied is 10kHz)
extern long dutyCycle = 0;                   // starting / default Hz: 0% is motor 'off'
extern int pwmResolution = 10;               // number of bits for motor resolution.  Can use 8 or 10, although 8 makes it a bit 'jumpy'

extern int rawCount = 0;  // counter for pulses incoming
extern unsigned long lastPulse = 0;
extern unsigned long lastPulseRPM = 0;

// onboard LED for error
extern bool hasError = false;
extern bool hasCAN = false;
extern bool hasGPS = false;

extern uint8_t incomingFreqMedian[] = { 0, 0, 0, 0, 0 };

extern bool ledOnboard = false;
extern int ledCounter = 0;

// define CAN Addresses.  All not req. but here for keepsakes
#define MOTOR1_ID 0x280
#define MOTOR2_ID 0x288
#define MOTOR3_ID 0x380
#define MOTOR5_ID 0x480
#define MOTOR6_ID 0x488
#define MOTOR7_ID 0x588

#define MOTOR_FLEX_ID 0x580
#define GRA_ID 0x38A   // byte 1 & 3 are shaft speeds?
#define gear_ID 0x440  // lower 4 bits of byte 2 are gear?

#define BRAKES1_ID 0x1A0
#define BRAKES2_ID 0x2A0
#define BRAKES3_ID 0x4A0
#define BRAKES5_ID 0x5A0

#define gearLever_ID 0x448
#define mWaehlhebel_1_ID 0x540  // DQ250 DSG ID

#define HALDEX_ID 0x2C0

#define emeraldECU1_ID 0x1000
#define emeraldECU2_ID 0x1001

extern void canInit(void);
extern void onBodyRX(void);

//Function Prototypes
extern void connectWifi();
extern void disconnectWifi();
extern void setupUI();
extern void setupOTA();
extern void textCallback(Control *sender, int type);
extern void generalCallback(Control *sender, int type);
extern void updateCallback(Control *sender, int type);
extern void getTimeCallback(Control *sender, int type);
extern void graphAddCallback(Control *sender, int type);
extern void graphClearCallback(Control *sender, int type);
extern void randomString(char *buf, int len);
extern void extendedCallback(Control *sender, int type, void *param);

extern void readEEP();       // in _eep.ino
extern void writeEEP();      // in _eep.ino
extern void updateLabels();  // in main.ino

// UI handles for WiFi
uint16_t bool_NeedleSweep, int16_sweepSpeed, int16_stepSpeed, int16_stepRPM;
uint16_t bool_testSpeedo, int16_tempSpeed, bool_testRPM;
uint16_t bool_useDSG, bool_useECU, bool_useGPS, bool_useHall, bool_useABS, bool_testCal; 

uint16_t bool_positiveOffset, int16_speedOffset;
uint16_t int16_maxSpeed, int16_maxHall, int16_maxCAN, int16_calNumber;
uint16_t int16_maxRPM, int16_tempRPM, int16_clusterRPM, int16_RPMScaling, int16_speedType;

int label_RPMHall, label_RPMCAN, label_hasCAN, label_hasGPS;
int label_speedHall, label_speedGPS, label_speedDSG, label_speedABS, label_speedECU, label_currentPWM;

uint16_t graph;
uint16_t mainTime;
volatile bool updates = false;
