#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include "SpeedPulserPro_board.h" // SPP_BOARD_DEVKIT / SPP_BOARD_S3 + SPP_HAS_* feature flags

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
#define enableDebug 0        // ** MASTER ** 0 = silence ALL serial debug. V5.06: releases ship
                             // with 0 (with 1, the per-frame CAN print at 115200 baud
                             // blocked the CAN task on a busy bus and dropped frames)
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
#define wifiHostName "SpeedPulserPro"
// V5.00 - S3 board (rev6) support on the V4.02 base + voltage-injection Test Board and one-point calibration
// V5.01 - drive calibration (multi-mark, ported from SpeedPulser V5.01): open-loop drive + gentle feedback trim
// V5.02 - RPM hall input passed through 1:1 in Hz; per-input glitch threshold from each input's full scale
// V5.03 - speed source picks exactly one input; over-range speed holds at Max Speed; Global Speed Offset sign now saves
// V5.04 - MQB DSG theoretical speed over UDS corrected (DID 0x100A on 0x7E1/0x7E9, raw/10)
// V5.05 - motor side mirrors SpeedPulser V5.05: timed tacho, Minimum Rail, ref-model trim, Auto Fill
// V5.06 - audit pass mirroring SpeedPulser V5.06: pulses-per-km inputs, safety fixes, DevKit PID removed, UI in Setup steps
// V5.07 - even needle sweep through the calibration (mirrors SpeedPulser V5.07)
// V5.08 - second audit pass mirroring SpeedPulser V5.08: calibration safety, Auto Fill rebuilds visibly, GPS / CAN fixes, UI tidy
// V5.09 - web UI shared with SpeedPulser (same API names), small audit fixes, start-up log, tuning constants here
#define FW_VERSION "5.09"

// Needle sweep (SpeedPulserPro_io.cpp), the same as SpeedPulser
#define SWEEP_TICK_MS 10           // output update rate while sweeping
#define SWEEP_EASE 0.2f            // last fraction of the climb spent slowing to a stop at the top (0..1)
#define SWEEP_TOP_HOLD_MS 400      // pause at the top before coming back (no tacho)
#define SWEEP_TOP_WAIT_MAX_MS 1500 // with a tacho: wait at the top until the motor is there, but no longer than this
#define SWEEP_TOP_ARRIVED 0.97f    // "there" = the tacho is within 3 % of the top mark's Hz
#define SWEEP_TOP_DWELL_MS 150     // then rest on the mark this long
#define SWEEP_RATE_REF 17.0f       // Speed / Tach needle rate that runs at exactly Sweep pace

// ===== Tuning (V5.09) =====
// Every value here was set on the bench for the TY3816B motor and a VW / Audi speedo;
// change them for different hardware. Each one is explained where it is used.
// Elsewhere: rail and drive limits in SpeedPulserPro_voltage.h, the drive table size in
// SpeedPulserPro_driveCal.h.

// Speed input glitch filter: half a period at Maximum speed, kept within these
#define INPUT_GLITCH_MIN_US 200        // 2.5 kHz: nothing real is faster
#define INPUT_GLITCH_MAX_US 3000       // the V5.05 fixed limit: no slower sensor needs more

// Motor tacho + feedback trim (SpeedPulserPro_control.cpp)
#define FB_PULSES_PER_REV 5       // TY3816B tacho: the interval pattern repeats every 5 pulses
constexpr uint32_t FB_GLITCH_US = 1000;     // tacho edges closer than this are line noise (top speed ~254 Hz = 3.9 ms)
constexpr uint32_t FB_STALL_US = 600000UL; // no edge for 0.6 s: stopped (the motor's floor is ~6 Hz = 0.17 s)
constexpr uint32_t FB_SPAN_US = 200000UL;  // average over at least this much time (whole turns)
constexpr float FB_MEAS_ALPHA = 0.5f;      // EMA weight for the measured-Hz low-pass
constexpr float FB_REF_TAU_S = 0.35f;     // the motor + measurement follow a step with this time constant (bench)
constexpr float FB_REF_SETTLED = 0.05f; // integrate once the model is within 5 % of the target
constexpr uint8_t FB_LOST_TICKS = 15;      // 1.5 s of control ticks (a start from rest reads 0 for up to ~1 s)
constexpr float FB_LOST_MIN_HZ = 9.0f;     // below this a real motor can read 0 (its floor)
constexpr float FB_PLAUSIBLE = 0.5f;       // integrate only within ±50 % of the model
constexpr float CAL_DRIVE_MAX_STEP_V = 0.5f; // one Calibration Mode drive request may raise the drive by this much

// Auto Fill (SpeedPulserPro_autoFill.cpp)
constexpr uint16_t AF_LOW_GRID[] = {10, 20, 30, 40}; // the motor bends most down here
constexpr uint16_t AF_STEPS[] = {10, 15, 20, 25, 30, 40, 50}; // spacing above 40, first that fits
constexpr uint16_t AF_MIN_GAP = 5;     // no auto mark this close to an eye mark
constexpr float AF_MIN_HZ = 9.0f;      // motor floor is ~6-8 Hz at a 4.8 V rail: keep auto marks clear of it
constexpr float AF_SKIP_HZ = 25.0f;    // an auto mark below this that can't be held is left out
constexpr float AF_TOL_HZ = 0.15f;     // done when within max(this, AF_TOL_FRAC x target)
constexpr float AF_TOL_FRAC = 0.005f;
constexpr float AF_ACCEPT_FRAC = 0.02f; // after AF_MAX_ITERS the best try must be this close
constexpr uint8_t AF_MAX_ITERS = 8;
constexpr uint8_t AF_MAX_STALLS = 6;
constexpr float AF_SLOPE0 = 36.0f;     // Hz per V of drive (TY3816B at a 4.8 V rail), refined as it goes
constexpr float AF_SLOPE_MIN = 8.0f, AF_SLOPE_MAX = 150.0f;
constexpr float AF_STEP_FRAC = 0.25f;  // largest drive change per try (fraction of the drive)
constexpr float AF_STEP_MIN_V = 0.05f;
constexpr uint16_t AF_SETTLE_START = 25; // ticks (100 ms): first mark, from rest
constexpr uint16_t AF_SETTLE_LOW = 16;   // after a change, below 25 Hz (slower measurement)
constexpr uint16_t AF_SETTLE = 12;
constexpr uint16_t AF_HOLD_LOW = 20;     // averaging, below 25 Hz
constexpr uint16_t AF_HOLD = 10;

// Speed Input Configuration
#define DEFAULT_AVERAGE_FILTER_RPM 6
#define durationReset 1500
#define LOOP_REST_MS 10            // loop() rests this long each pass (V5.09)
#define GPS_MIN_SPEED_KPH 3       // GPS speed below this reads 0 (standstill noise)
#define CAN_SCALE_MAX 1000.0f     // CAN speed scale limit (+/-): a typo can't overflow the speed (V5.09)
#define DSG_RATIO_MIN 0.1f        // DSG gear ratios and final drives, accepted range (V5.09)
#define DSG_RATIO_MAX 20.0f
#define DSG_TIRE_CIRC_MIN 0.5f    // tyre circumference (m), accepted range
#define DSG_TIRE_CIRC_MAX 4.0f
#define SPEED_OVER_RANGE_LOOPS 10 // speed task loops (10 x 50 ms = 0.5 s) a source must stay above Max Speed before the needle parks at Max Speed

// ===========================================================================
// Pin Definitions — one block per board (selected in SpeedPulserPro_board.h)
// ===========================================================================
#if SPP_BOARD_S3
// ---------------------------------------------------------------------------
// S3 board: ESP32-S3-WROOM-1-N16R8 (U13). Confirmed pad-by-pad against the netlist
// of PCBs/rev6-allConfirmed/ProPrj_speedPulserProV3_claude-s3_2026-09-29_basic.epro2
// (module pad -> IOn per the WROOM-1 pinout). Every output that must be safe at
// reset has its own pull-down on the board (EN_ADJ R7, ESP_MOTOR R28,
// ESP_MOTOR_DIR R27, CAN0_STANDBY R18, ESP_SPD_PU/PD R44/R45, ESP_MK2RPM R25).
// Unused: IO3 / IO45 / IO46 (strapping), IO35..IO42, UART0 (IO43/IO44 — the
// console is native USB on IO19/IO20). IO0 is the BOOT button only.
// ---------------------------------------------------------------------------
#define pinMotorOutput 10       // pad 18 ESP_MOTOR      -> U11 SN74LVC1G14 -> MOTOR_OUT (PWM, LEDC)
#define pinRPMOutput 9          // pad 17 ESP_MK2RPM     -> R24 330R -> Q1 MMBT5551 (coil RPM, GPTimer toggled)

#define pinEngineRPMInput 12    // pad 20 ESP_RPM        <- U12 (ECU_RPM after R30/D8 zener/Schmitt)
#define pinGearboxHall 16       // pad 9  ESP_SPEEDPULSE <- U12 (SPEEDPULSE after R21/D6 zener/Schmitt)

#define pinRX_CAN 13            // pad 21 CAN0_RX  <- U7 TCAN1044V RXD
#define pinTX_CAN 14            // pad 22 CAN0_TX  -> U7 TCAN1044V TXD
#define pinCANStandby 21        // pad 23 CAN0_STANDBY -> U7 STB; R18 pull-down, firmware holds LOW = normal mode

// GPS header: pin 2 = net GPS_RX (ESP TX -> module RX), pin 3 = net GPS_TX (ESP RX <- module TX).
// Same sense as the DevKit (GPIO14 was net GPS_TX = ESP RX). If a module stays silent, swap these two.
#define pinRX_GPS 17            // pad 10 ESP UART RX  <- GPS_TX
#define pinTX_GPS 18            // pad 11 ESP UART TX  -> GPS_RX

#define pinMotorFeedback 8      // pad 12 ESP_MOTFEEDBACK <- U9 (MOTOR_FEEDBACK after R20/D4 zener/Schmitt)
#define pinMotorDirection 11    // pad 19 ESP_MOTOR_DIR   -> R26 -> Q2 2N7002 -> MOTOR_DIR (HIGH = reverse)

// --- Buck voltage control (always fitted on this board) ---
#define pinBoardVersion 7       // pad 7 BOARD_ID: R12 1k to GND -> reads LOW with INPUT_PULLUP (sanity check only)
#define pinVoltageAdjust 6      // pad 6 V_ADJ PWM "DAC" -> R32 1k/C38 1uF -> R9 1k -> R8 10k -> U5 FB (inverse)
#define pinBuckEnable 5         // pad 5 EN_ADJ -> U5 TPS62933 EN (HIGH = motor supply on; R7 holds it LOW at reset)
#define pinMotorRailSense 4     // pad 4 V_MOT_SENSE: 9V rail via R33 39k / R34 10k + C39 100nF -> ADC1_CH3

// --- Variable-reluctance (VR) speed input — MAX9924 (U8) C Out, R15 10k pull-up to 3V3 ---
#define pinVRInput 15           // pad 8 VR_COUT, one clean edge per zero crossing, counted like the hall input

// --- Speed-input bias: replaces the old PullDown/Up jumper (see applySpeedInputBias) ---
#define pinSpeedPullUp 47       // pad 24 ESP_SPD_PU -> Q4 BSS138 -> Q3 MMBT5401: R19 4.7k from VCC (12 V) onto SPEEDPULSE
#define pinSpeedPullDown 48     // pad 25 ESP_SPD_PD -> Q5 BSS138:                R46 4.7k from SPEEDPULSE to GND

#define pinOnboardLED 38        // pad 31 ESP_LED -> R41 330R -> LED -> GND (active HIGH); the module has no LED

#else
// ---------------------------------------------------------------------------
// ESP32 DevKit V1 (WROOM-32) boards — PWM motor drive only. The DevKit never has
// voltage injection, rail sense, VR or switched bias, so none of those pins exist.
// ---------------------------------------------------------------------------
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

// Coolant temperature emulation — X9C103 digital potentiometer (as used in the MFSW).
// These pins are currently DEFINE-ONLY (the coolant feature isn't wired up yet) —
// re-check against the PCB before implementing it.
#define pinCoolantUD 4          // Coolant Resistance (U/D — up/down direction)
#define pinCoolantINC 5         // Coolant Resistance (INC — increment clock)
#define pinCoolantCS 12         // Coolant Resistance (CS — chip select)

#define pinOnboardLED 2 // onboard LED
#endif // SPP_BOARD_S3

// Baud Rates
#define baudGPS 9600 // default low baud rate for Neo-6M GPS module (can be raised to 115200 after init)

// Motor Configuration

// DSG Configuration
#define LEVER_P 0x8
#define LEVER_R 0x7
#define LEVER_N 0x6
#define LEVER_D 0x5
#define LEVER_S 0xC
#define LEVER_TIPTRONIC_ON 0xE
#define LEVER_TIPTRONIC_UP 0xA
#define LEVER_TIPTRONIC_DOWN 0xB

// DSG speed smoothing: gear and RPM arrive on separate CAN frames, so during a
// shift they are momentarily inconsistent and the calculated speed spikes/bucks.
#define dsgGearSettleMs 300       // hold last DSG speed for this long after a gear change (real road speed barely moves during a shift)
#define dsgSpeedSmoothing 0.35    // low-pass factor 0..1 (1 = no smoothing) applied to DSG speed to remove residual RPM jitter

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
