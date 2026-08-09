#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>

// ---------------------------------------------------------------------------
// Serial debug — tagged, per-subsystem logging (AirLift-style)
//
//   * enableDebug is the SINGLE master switch: 0 silences EVERY serial debug
//     statement (release-safe). For a dev build, override without editing this
//     file by adding   -D enableDebug=1   to build_flags in platformio.ini.
//   * With enableDebug = 1, toggle the per-subsystem flags below to pick exactly
//     which [TAG] streams print.
//   * Every subsystem has a matched macro pair (defined further down):
//       DEBUG_XXX(x, ...)   -> "[TAG] " + printf + trailing '\n'
//       DEBUG_XXX_(x, ...)  -> same, but WITHOUT the trailing '\n' (line in parts)
//   * [SYS] also carries the 1 Hz telemetry block (see taskDiagnostics).
// ---------------------------------------------------------------------------
#define baudSerial 115200

#ifndef enableDebug
#define enableDebug 0        // ** MASTER ** 0 = silence ALL serial debug
#endif

#define debugSys     1       // [SYS]   boot / tasks / general / 1 Hz telemetry
#define debugPower   1       // [PWR]   power_manager (WiFi-off / CPU scaling)
#define debugWifi    1       // [WiFi]  soft-AP, web server, REST, OTA, LittleFS
#define debugIO      1       // [IO]    hardware bring-up (LED / PWM / interrupts)
#define debugEEP     1       // [EEP]   Preferences read / write
#define debugCAN     1       // [CAN]   TWAI driver + chassis CAN RX
#define debugGPS     1       // [GPS]   GPS init / baud / rate / RX
#define debugSpeed   1       // [SPD]   speed processing + incoming pulses
#define debugRPM     1       // [RPM]   RPM processing + output
#define debugDSG     1       // [DSG]   DSG gear + speed
#define debugUDS     1       // [UDS]   UDS / TP2.0 diagnostic speed
#define debugSavvy   1       // [SAVVY] SavvyCAN GVRET bridge
#define debugCtrl    1       // [CTRL]  speed offset / motor calibration
#define debugFB      1       // [FB]    closed-loop feedback (PID)

// Backward-compat flag aliases — existing "#if serialDebugX" blocks now route
// through the master switch AND the matching per-subsystem flag.
#define serialDebug         (enableDebug && debugSys)
#define serialDebugIncoming (enableDebug && debugSpeed)
#define serialDebugWifi     (enableDebug && debugWifi)
#define serialDebugEEP      (enableDebug && debugEEP)
#define serialDebugGPS      (enableDebug && debugGPS)
#define ChassisCANDebug     (enableDebug && debugCAN)

// System Configuration
#define eepRefresh 5000
#define labelRefresh 200
#define wifiDisable 60000
#define wifiHostName "SpeedPulserPro"
#define FW_VERSION "3.04"

// Speed Input Configuration
#define incomingType 0
#define DEFAULT_AVERAGE_FILTER_HALL 6
#define DEFAULT_AVERAGE_FILTER_RPM 6
#define durationReset 1500

// Pin Definitions
#define pinMotorOutput 21       // Motor Output (PWM)
#define pinRPMOutput 22         // RPM Output

#define pinEngineRPMInput 18    // Engine RPM Input (hall)
#define pinGearboxHall 26       // Gearbox Hall Input

#define pinRX_CAN 17 // Chassis CAN RX
#define pinTX_CAN 16 // Chassis CAN TX

#define pinRX_GPS 14 // GPS RX
#define pinTX_GPS 13 // GPS TX

#define pinMotorFeedback 23     // Motor Feedback Input
#define pinMotorDirection 19    // Motor Direction (HIGH = reverse)

// Coolant temperature emulation — X9C103 digital potentiometer (as used in the MFSW)
#define pinCoolantUD 25         // Coolant Resistance (U/D — up/down direction)
#define pinCoolantINC 27        // Coolant Resistance (INC — increment clock)
#define pinCoolantCS 12         // Coolant Resistance (CS — chip select)

#define pinOnboardLED 2 // onboard LED

// Baud Rates
#define baudGPS 9600 // default low baud rate for Neo-6M GPS module (can be raised to 115200 after init)

// Motor Configuration
#define speedMultiplier 1
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
// --- [SYS] master / general (uncategorised) + 1 Hz telemetry ---
#if enableDebug && debugSys
#define DEBUG(x, ...)  Serial.printf("[SYS] " x "\n", ##__VA_ARGS__)
#define DEBUG_(x, ...) Serial.printf("[SYS] " x, ##__VA_ARGS__)
#else
#define DEBUG(x, ...)
#define DEBUG_(x, ...)
#endif

// --- [PWR] power management ---
#if enableDebug && debugPower
#define DEBUG_PWR(x, ...)  Serial.printf("[PWR] " x "\n", ##__VA_ARGS__)
#define DEBUG_PWR_(x, ...) Serial.printf("[PWR] " x, ##__VA_ARGS__)
#else
#define DEBUG_PWR(x, ...)
#define DEBUG_PWR_(x, ...)
#endif

// --- [WiFi] soft-AP, web server, REST, OTA, LittleFS ---
#if enableDebug && debugWifi
#define DEBUG_WIFI(x, ...)  Serial.printf("[WiFi] " x "\n", ##__VA_ARGS__)
#define DEBUG_WIFI_(x, ...) Serial.printf("[WiFi] " x, ##__VA_ARGS__)
#else
#define DEBUG_WIFI(x, ...)
#define DEBUG_WIFI_(x, ...)
#endif

// --- [IO] hardware bring-up ---
#if enableDebug && debugIO
#define DEBUG_IO(x, ...)  Serial.printf("[IO] " x "\n", ##__VA_ARGS__)
#define DEBUG_IO_(x, ...) Serial.printf("[IO] " x, ##__VA_ARGS__)
#else
#define DEBUG_IO(x, ...)
#define DEBUG_IO_(x, ...)
#endif

// --- [EEP] Preferences read / write ---
#if enableDebug && debugEEP
#define DEBUG_EEP(x, ...)  Serial.printf("[EEP] " x "\n", ##__VA_ARGS__)
#define DEBUG_EEP_(x, ...) Serial.printf("[EEP] " x, ##__VA_ARGS__)
#else
#define DEBUG_EEP(x, ...)
#define DEBUG_EEP_(x, ...)
#endif

// --- [CAN] TWAI driver + chassis CAN RX ---
#if enableDebug && debugCAN
#define DEBUG_CAN(x, ...)  Serial.printf("[CAN] " x "\n", ##__VA_ARGS__)
#define DEBUG_CAN_(x, ...) Serial.printf("[CAN] " x, ##__VA_ARGS__)
#else
#define DEBUG_CAN(x, ...)
#define DEBUG_CAN_(x, ...)
#endif

// --- [GPS] GPS init / baud / rate / RX ---
#if enableDebug && debugGPS
#define DEBUG_GPS(x, ...)  Serial.printf("[GPS] " x "\n", ##__VA_ARGS__)
#define DEBUG_GPS_(x, ...) Serial.printf("[GPS] " x, ##__VA_ARGS__)
#else
#define DEBUG_GPS(x, ...)
#define DEBUG_GPS_(x, ...)
#endif

// --- [SPD] speed processing + incoming pulses ---
#if enableDebug && debugSpeed
#define DEBUG_SPD(x, ...)  Serial.printf("[SPD] " x "\n", ##__VA_ARGS__)
#define DEBUG_SPD_(x, ...) Serial.printf("[SPD] " x, ##__VA_ARGS__)
#else
#define DEBUG_SPD(x, ...)
#define DEBUG_SPD_(x, ...)
#endif

// --- [RPM] RPM processing + output ---
#if enableDebug && debugRPM
#define DEBUG_RPM(x, ...)  Serial.printf("[RPM] " x "\n", ##__VA_ARGS__)
#define DEBUG_RPM_(x, ...) Serial.printf("[RPM] " x, ##__VA_ARGS__)
#else
#define DEBUG_RPM(x, ...)
#define DEBUG_RPM_(x, ...)
#endif

// --- [DSG] DSG gear + speed ---
#if enableDebug && debugDSG
#define DEBUG_DSG(x, ...)  Serial.printf("[DSG] " x "\n", ##__VA_ARGS__)
#define DEBUG_DSG_(x, ...) Serial.printf("[DSG] " x, ##__VA_ARGS__)
#else
#define DEBUG_DSG(x, ...)
#define DEBUG_DSG_(x, ...)
#endif

// --- [UDS] UDS / TP2.0 diagnostic speed ---
#if enableDebug && debugUDS
#define DEBUG_UDS(x, ...)  Serial.printf("[UDS] " x "\n", ##__VA_ARGS__)
#define DEBUG_UDS_(x, ...) Serial.printf("[UDS] " x, ##__VA_ARGS__)
#else
#define DEBUG_UDS(x, ...)
#define DEBUG_UDS_(x, ...)
#endif

// --- [SAVVY] SavvyCAN GVRET bridge ---
#if enableDebug && debugSavvy
#define DEBUG_SAVVY(x, ...)  Serial.printf("[SAVVY] " x "\n", ##__VA_ARGS__)
#define DEBUG_SAVVY_(x, ...) Serial.printf("[SAVVY] " x, ##__VA_ARGS__)
#else
#define DEBUG_SAVVY(x, ...)
#define DEBUG_SAVVY_(x, ...)
#endif

// --- [CTRL] speed offset / motor calibration ---
#if enableDebug && debugCtrl
#define DEBUG_CTRL(x, ...)  Serial.printf("[CTRL] " x "\n", ##__VA_ARGS__)
#define DEBUG_CTRL_(x, ...) Serial.printf("[CTRL] " x, ##__VA_ARGS__)
#else
#define DEBUG_CTRL(x, ...)
#define DEBUG_CTRL_(x, ...)
#endif

// --- [FB] closed-loop feedback (PID) ---
#if enableDebug && debugFB
#define DEBUG_FB(x, ...)  Serial.printf("[FB] " x "\n", ##__VA_ARGS__)
#define DEBUG_FB_(x, ...) Serial.printf("[FB] " x, ##__VA_ARGS__)
#else
#define DEBUG_FB(x, ...)
#define DEBUG_FB_(x, ...)
#endif

// --- Legacy print macros (compile safety-net) — route to the [SYS] stream.
//     Prefer the tagged DEBUG_XXX macros above for new code. ---
#if enableDebug && debugSys
#define DEBUG_PRINT(x)    Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif

// Include global variables from globals.h
#include "SpeedPulserPro_globals.h"

#endif // CONFIG_H
