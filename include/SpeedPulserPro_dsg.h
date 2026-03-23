#ifndef DSG_H
#define DSG_H

#include "SpeedPulserPro_config.h"

double dq250_gear_ratio(uint8_t gear);
double dq250_final(uint8_t gear);
double dq250_speed(uint16_t rpm_in, uint8_t gear);
void parseDSG();

#endif // DSG_H
