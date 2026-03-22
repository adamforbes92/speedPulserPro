#include "SpeedPulserPro_dsg.h"

double dq250_gear_ratio(uint8_t gear) {
  switch (gear) {
    case 1:
      return 3.462;
    case 2:
      return 2.050;
    case 3:
      return 1.300;
    case 4:
      return 0.902;
    case 5:
      return 0.914;
    case 6:
      return 0.756;
    default:
      return 1.0;
  }
}

double dq250_final(uint8_t gear) {
  return (gear == 5 || gear == 6) ? 3.043 : 4.118;
}

double dq250_speed(uint16_t rpm_in, uint8_t gear) {
  double tireCircumference = M_PI * 0.6;
  double rpm = (double)rpm_in * 1.0;
  double speed_mps = (rpm * tireCircumference) / (dq250_gear_ratio(gear) * dq250_final(gear) * 60);
  double vehicleSpeedTemp = speed_mps * 3.6;
  return vehicleSpeedTemp > 10 ? vehicleSpeedTemp : 1;
}

void parseDSG() {
  if (vehicleRPM != 0 && gear != 0) {
    switch (lever) {
      case LEVER_D:
      case LEVER_S:
      case LEVER_TIPTRONIC_ON:
      case LEVER_TIPTRONIC_UP:
      case LEVER_TIPTRONIC_DOWN:
        dsgSpeed = (uint16_t)(dq250_speed(vehicleRPM, gear) + 0.5);
        break;
      case LEVER_P:
        dsgSpeed = 0;
      default:
        dsgSpeed = 0;
        break;
    }
  }
}
