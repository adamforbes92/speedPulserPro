#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>

// Serial Debug Configuration
#define baudSerial 115200
#define serialDebug 0
#define serialDebugIncoming 0
#define serialDebugWifi 0
#define serialDebugEEP 0
#define serialDebugGPS 0
#define ChassisCANDebug 0

// System Configuration
#define eepRefresh 5000
#define labelRefresh 200
#define wifiDisable 60000
#define wifiHostName "SpeedPulserPro"
#define FW_VERSION "2.10"

// Speed Input Configuration
#define incomingType 0
#define DEFAULT_AVERAGE_FILTER_HALL 6
#define DEFAULT_AVERAGE_FILTER_RPM 6
#define durationReset 1500

// Pin Definitions
#define pinMotorOutput 21
#define pinMotorInput 18
#define pinSpeedInput 26
#define pinDirection 19
#define pinOnboardLED 2
#define pinRX_CAN 17
#define pinTX_CAN 16
#define pinRX_GPS 14
#define pinTX_GPS 13
#define pinCoil 22

// Baud Rates
#define baudGPS 9600

// Motor Configuration
#define speedMultiplier 1
#define convertToMPH 0
#define mphFactor 0.621371

// DSG Configuration
#define PI 
#define LEVER_P 0x8
#define LEVER_R 0x7
#define LEVER_N 0x6
#define LEVER_D 0x5
#define LEVER_S 0xC
#define LEVER_TIPTRONIC_ON 0xE
#define LEVER_TIPTRONIC_UP 0xA
#define LEVER_TIPTRONIC_DOWN 0xB
#define gearPause 20
#define rpmPause 50

// CAN IDs
#define MOTOR1_ID 0x280
#define MOTOR2_ID 0x288
#define MOTOR3_ID 0x380
#define MOTOR5_ID 0x480
#define MOTOR6_ID 0x488
#define MOTOR7_ID 0x588
#define MOTOR_FLEX_ID 0x580
#define GRA_ID 0x38A
#define gear_ID 0x440
#define BRAKES1_ID 0x1A0
#define BRAKES2_ID 0x2A0
#define BRAKES3_ID 0x4A0
#define BRAKES5_ID 0x5A0
#define gearLever_ID 0x448
#define mWaehlhebel_1_ID 0x540
#define HALDEX_ID 0x2C0
#define emeraldECU1_ID 0x1000
#define emeraldECU2_ID 0x1001

// Debug Macros
#ifdef serialDebug
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(x...) Serial.printf(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(x...)
#endif

// Include global variables from globals.h
#include "SpeedPulserPro_globals.h"

#endif // CONFIG_H
