#ifndef MOTOR_CAL_H
#define MOTOR_CAL_H

#include "SpeedPulserPro_config.h"

void updateMotorArray();
const char *getCalibrationText(uint8_t calibrationVal);
uint8_t getCalibrationCount();

#endif // MOTOR_CAL_H
