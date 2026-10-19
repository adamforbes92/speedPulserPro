#ifndef SPEEDPULSERPRO_BOARD_H
#define SPEEDPULSERPRO_BOARD_H

// ---------------------------------------------------------------------------
// Board selection — ONE codebase, two PCB families. Pick the board in
// platformio.ini (each [env:...] sets exactly one of these build flags):
//
//   -DSPP_BOARD_DEVKIT   ESP32 DevKit V1 (WROOM-32) boards. Plain PWM motor drive:
//                        NO voltage injection, NO rail sense, NO VR, NO switched
//                        speed bias — and never will have. A DevKit build carries
//                        no code, settings or UI for any of them.
//   -DSPP_BOARD_S3       ESP32-S3-WROOM-1-N16R8 board (PCBs/rev6-allConfirmed,
//                        speedPulserProV3_claude-s3): native USB, adjustable motor
//                        buck with V_ADJ injection + rail read-back, MAX9924 VR
//                        front end, firmware-switched speed-input bias, CAN standby.
//
// Everything board-specific hangs off the SPP_BOARD_* / SPP_HAS_* macros below:
// pin numbers live in SpeedPulserPro_config.h, and each optional hardware block
// is compiled in with `#if SPP_HAS_xxx` so a board without it carries no code,
// no settings and no UI for it. The PCB defines what a board has — nothing is
// detected at run time.
//
// This header only depends on sdkconfig so it can be included from anywhere
// (globals.h is included before config.h in some translation units).
// ---------------------------------------------------------------------------
#include "sdkconfig.h"

#if defined(SPP_BOARD_DEVKIT) && defined(SPP_BOARD_S3)
#error "SPP_BOARD_DEVKIT and SPP_BOARD_S3 are mutually exclusive - check platformio.ini build_flags"
#endif

// No flag given (e.g. a bare `pio run` on a hand-made env): follow the chip.
#if !defined(SPP_BOARD_DEVKIT) && !defined(SPP_BOARD_S3)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define SPP_BOARD_S3 1
#else
#define SPP_BOARD_DEVKIT 1
#endif
#endif

#ifndef SPP_BOARD_DEVKIT
#define SPP_BOARD_DEVKIT 0
#endif
#ifndef SPP_BOARD_S3
#define SPP_BOARD_S3 0
#endif

// The pin maps are chip-specific, so refuse a board/chip mismatch at compile time.
#if SPP_BOARD_S3 && !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "SPP_BOARD_S3 must be built for an ESP32-S3 target (board = esp32-s3-devkitc-1)"
#endif
#if SPP_BOARD_DEVKIT && !defined(CONFIG_IDF_TARGET_ESP32)
#error "SPP_BOARD_DEVKIT must be built for the classic ESP32 (board = esp32doit-devkit-v1)"
#endif

// ---- Optional hardware blocks (all on the S3 board, none on the DevKit) --------
#define SPP_HAS_VOLTAGE_CTRL SPP_BOARD_S3  // adjustable motor buck: V_ADJ injection + EN_ADJ (U5 TPS62933)
#define SPP_HAS_RAIL_SENSE   SPP_BOARD_S3  // motor-rail read-back divider on an ADC1 pin (V_MOT_SENSE)
#define SPP_HAS_VR           SPP_BOARD_S3  // MAX9924 variable-reluctance speed input (VR_COUT)
#define SPP_HAS_SPEED_BIAS   SPP_BOARD_S3  // speed-input pull-up / pull-down switched by GPIO (Q3/Q4/Q5)
#define SPP_HAS_CAN_STANDBY  SPP_BOARD_S3  // TCAN1044 STB reaches a GPIO (held LOW = normal mode)
#define SPP_HAS_NATIVE_USB   SPP_BOARD_S3  // console/esptool over USB-Serial-JTAG, no CH340

#if SPP_HAS_RAIL_SENSE && !SPP_HAS_VOLTAGE_CTRL
#error "rail sense without voltage control makes no sense - check SPP_HAS_* flags"
#endif

// ---- Identity strings (OTA release channel + boot banner) --------------------
// SPP_BOARD_ID selects the per-board images in Releases/releases.json ("boards")
// and the Releases/V<ver>[-S3]/ layout written by tools/make_release.py.
#if SPP_BOARD_S3
#define SPP_BOARD_ID   "esp32s3"
#define SPP_BOARD_NAME "ESP32-S3 board"
#else
#define SPP_BOARD_ID   "esp32"
#define SPP_BOARD_NAME "ESP32 DevKit V1 board"
#endif

#endif // SPEEDPULSERPRO_BOARD_H
