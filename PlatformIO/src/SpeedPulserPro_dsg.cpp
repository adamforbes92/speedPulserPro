#include "SpeedPulserPro_dsg.h"

// Ratios, final drives and tire circumference are adjustable via the web UI
// (defaults in SpeedPulserPro_globals.cpp are DQ250 02E ratios).
double dq250_gear_ratio(uint8_t gear) {
  return (gear >= 1 && gear <= 6) ? dsgGearRatio[gear] : 1.0;
}

double dq250_final(uint8_t gear) {
  return (gear == 5 || gear == 6) ? dsgFinalDrive56 : dsgFinalDrive14;
}

double dq250_speed(uint16_t rpm_in, uint8_t gear) {
  double tireCircumference = dsgTireCirc;
  double rpm = (double)rpm_in * 1.0;
  double speed_mps = (rpm * tireCircumference) / (dq250_gear_ratio(gear) * dq250_final(gear) * 60);
  double vehicleSpeedTemp = speed_mps * 3.6;
  return vehicleSpeedTemp > 10 ? vehicleSpeedTemp : 0; // V5.06: was 1, so stopped in D the needle sat at 1+offset
}

void parseDSG() {
  static uint8_t lastGear = 0;
  static uint32_t gearChangedMs = 0; // elapsed time (V5.09): "millis() >= until" broke at the 49.7-day wrap
  static double dsgSpeedFiltered = 0;

  // Engine RPM from the input itself (CAN or hall), never the tach output value:
  // with Test RPM on that is the test value (V5.06).
  const uint16_t engineRpm = useRPMCAN ? vehicleRPMCAN : vehicleRPMHall;
  if (engineRpm != 0 && gear != 0) {
    switch (lever) {
      case LEVER_D:
      case LEVER_S:
      case LEVER_TIPTRONIC_ON:
      case LEVER_TIPTRONIC_UP:
      case LEVER_TIPTRONIC_DOWN:
        // gear and vehicleRPM come from separate CAN frames; during a shift
        // they are transiently inconsistent (esp. MQB, where 'gear' is the
        // target gear and updates before RPM settles), which spikes the
        // calculated speed. Hold the last value briefly, then low-pass it.
        if (gear != lastGear) {
          lastGear = gear;
          gearChangedMs = millis();
        }
        if (millis() - gearChangedMs >= dsgGearSettleMs) {
          double raw = dq250_speed(engineRpm, gear);
          dsgSpeedFiltered += (raw - dsgSpeedFiltered) * dsgSpeedSmoothing;
          dsgSpeed = (uint16_t)(dsgSpeedFiltered + 0.5);
        }
        break;
      case LEVER_P:
        dsgSpeed = 0;
        dsgSpeedFiltered = 0;
      default:
        dsgSpeed = 0;
        dsgSpeedFiltered = 0;
        break;
    }
  } else {
    dsgSpeed = 0;  // engine stopped / no gear engaged - don't hold a stale value
    dsgSpeedFiltered = 0;
  }
}
