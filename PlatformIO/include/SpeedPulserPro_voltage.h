#ifndef SPEEDPULSERPRO_VOLTAGE_H
#define SPEEDPULSERPRO_VOLTAGE_H

#include <Arduino.h>
#include "SpeedPulserPro_board.h"

#if SPP_HAS_VOLTAGE_CTRL
// ---------------------------------------------------------------------------
// SpeedPulserPro — buck voltage control (S3 board only; mirrors SpeedPulser V5)
//
// The S3 board has an adjustable motor supply: an ESP PWM "DAC" (V_ADJ, IO6) is
// injected into the U5 TPS62933 feedback node so firmware sets the motor rail,
// EN_ADJ (IO5) gates the supply on/off, and V_MOT_SENSE (IO4, R33 39k / R34 10k)
// reads the rail back. The "Test Board" routine steps the rail and reads it back
// to prove the injection path AND the sense divider, and stores a measured
// duty->volts fit for this board. The DevKit has none of this hardware, so a
// DevKit build compiles none of it.
//
// The injection is INVERSE: a higher V_ADJ pulls FB up and LOWERS the motor
// rail. Rail commands are in volts and mapped to the inverse V_ADJ duty
// internally, so the rest of the firmware never has to think about the inversion.
//
// V5.01 — DRIVE (ported from SpeedPulser V5.01): the motor is commanded with one
// number, the drive in volts (rail volts x PWM fraction, 0..Maximum Rail). Down to
// Minimum Rail x Nominal PWM the PWM is held at Nominal and only the rail moves;
// below that the rail sits at Minimum Rail and the PWM falls to zero, so the drive
// reaches all the way to a stopped needle. V5.05: Minimum Rail is a setting
// (default 4.8 V, see SpeedPulserPro_voltage.cpp). Calibration captures needle
// marks against drive (see SpeedPulserPro_driveCal.h).
// ---------------------------------------------------------------------------

// ===== Rail model (rev6 values — identical network to SpeedPulser V5) =====
// Vrail = Vref*(1 + R6/R11) + (R6/Rinj)*(Vref - D*3V3)
//       = 0.8*(1 + 39k/3k) + (39k/12k)*(0.8 - D*3.3)    (Rinj = R8+R9+R32 = 12k)
//       = 13.8 - 10.725*D                                (D = V_ADJ duty 0..1)
// Used until the Test Board fit replaces it with this board's measured line.
#define RAIL_MODEL_V0      13.8f    // rail (V) at 0 % V_ADJ duty
#define RAIL_MODEL_SLOPE  -10.725f  // rail volts per unit V_ADJ duty
#define RAIL_V_FLOOR       3.0f     // ~100 % V_ADJ duty: lowest rail the injection can reach
#define RAIL_V_CEIL       12.0f     // firmware ceiling: motor rating, C21's voltage rating, and the
                                    // ADC read-back saturates at ~12.2 V (2.5 V at the pin)
#define RAIL_V_MAX_DEFAULT 8.0f    // V5.08 Maximum Rail default (new units): room for 240-300 km/h dials, raise to 12 V if needed
#define RAIL_OV_MARGIN_V   2.0f     // over-voltage: the rail this far above its command...
#define RAIL_OV_TICKS      5        // ...for this many 100 ms reads (0.5 s)...
#define RAIL_OV_SETTLE_MS  3000     // ...with the command steady this long (a lowered rail falls slowly)
#define RAIL_V_MIN_DEFAULT 4.8f    // V5.05 Minimum Rail default (SpeedPulser V5.04): the TY3816B's driver
                                    // needs ~4.75 V+ to run smoothly at low speed (bench, 2026-11-15)

// ===== Rail read-back divider (V_MOT_SENSE) =====
#define RAIL_SENSE_R_TOP_K 39.0f    // R33 (kΩ) rail -> IO4
#define RAIL_SENSE_R_BOT_K 10.0f    // R34 (kΩ) IO4 -> GND

// Board detection + live state
extern bool  boardHasVoltageControl;  // always true on an S3 build (the PCB defines it, not a strap)
extern bool  boardIdStrapOk;          // BOARD_ID (IO7, R12 1k) read LOW at boot — sanity check only
extern bool  buckEnabled;             // true once the buck supply has been enabled
extern volatile bool railTripped;    // rail read too high: motor + buck off until a reboot (latched)
extern float railTripV;              // the rail reading that tripped it
extern float lastRailCmdV;            // last commanded motor rail (V)
extern float lastPwmFrac;             // last motor PWM fraction applied by the drive (0..1)
extern float lastDriveV;              // last drive applied (V = rail x PWM)
extern float motorRailV;              // measured motor rail (V), smoothed (V_MOT_SENSE)

// User configuration (persisted). The S3 board always runs on the drive
// calibration (V5.06: the "Voltage Control off" fixed-rail mode is gone).
extern float pwmNominal;           // PWM fraction held while the rail does the work
extern float railMaxV;             // highest rail used (V); the drive tops out at this x 100 % PWM
extern float railMinV;             // rail held at the low end (V) while the PWM falls to zero
extern float trimGain;             // feedback trim: relative drive change per second per unit relative Hz error
extern float trimLimit;            // feedback trim authority (fraction of the calibrated drive, e.g. 0.2 = ±20 %)

// Calibration: drive applied in calibration mode (V)
extern float calibrationDriveV;

// ===== Board test (Test Board button) =====
enum BoardTestCode : uint8_t {
    BT_NOT_RUN      = 0,  // never run on this unit
    BT_PASS         = 1,  // V_ADJ injection + rail read-back both verified
    BT_LEGACY       = 2,  // (unused on the Pro: a DevKit build has no Test Board)
    BT_NO_SENSE     = 3,  // rail reads ~0 V at every step: sense divider open, or buck not running
    BT_NO_RESPONSE  = 4,  // rail is there but does not follow V_ADJ
    BT_OUT_OF_RANGE = 5,  // follows V_ADJ but far from the expected voltages (check R6/R11/R8/R9/R32)
};
#define BOARD_TEST_BOTTOM_V 5.0f   // Test Board steps: from here...
#define BOARD_TEST_TOP_V 10.0f     // ...up to this, or Maximum rail if that is lower...
#define BOARD_TEST_MIN_TOP_V 6.0f  // ...which must be at least this
#define BOARD_TEST_HANDOVER_MS 150 // the speed task stands back within one loop (50 ms)
#define BOARD_TEST_POINTS 3
extern volatile bool boardTestActive;   // true while the test owns the motor/rail (speed task yields)
extern uint8_t boardTestCode;           // BoardTestCode of the last run (persisted)
extern float   boardTestTargetV[BOARD_TEST_POINTS]; // model target for each step (V)
extern float   boardTestMeasV[BOARD_TEST_POINTS];   // measured rail at each step (V, persisted)
extern bool    railFitValid;            // true when railFitV0/railFitSlope came from a passed test
extern float   railFitV0;               // measured rail at 0 % V_ADJ duty (extrapolated)
extern float   railFitSlope;            // measured rail volts per unit V_ADJ duty (negative)

void detectBoardVersion();            // read the BOARD_ID strap once at boot (sanity check)
void initVoltageControl();            // configure V_ADJ PWM + buck-enable pin + rail ADC
void setMotorRailVolts(float volts);  // command the rail in volts (clamped to RAIL_V_FLOOR..RAIL_V_CEIL)
float driveMaxV();                    // top of the drive range (V): Maximum Rail x 100 % PWM
uint16_t applyMotorDrive(float driveV); // split drive into rail + PWM, apply both; returns the raw PWM duty
void enableBuck();                   // set a safe (low) rail, then drive EN high
void disableBuck();                   // drive EN low (supply off)

float readMotorRailVolts();           // one averaged ADC read of the rail (V)
void  updateMotorRail();              // smoothed read into motorRailV — call on a ~100 ms cadence
bool  railLimited();                  // rail sitting well below its command (supply too low / overload)

bool  startBoardTest();               // kick off the Test Board task; false if already running
const char *boardTestText(uint8_t code);

#endif // SPP_HAS_VOLTAGE_CTRL

#endif // SPEEDPULSERPRO_VOLTAGE_H
