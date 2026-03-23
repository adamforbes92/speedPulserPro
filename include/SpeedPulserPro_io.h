#ifndef IO_H
#define IO_H

#include "SpeedPulserPro_config.h"
#include "SpeedPulserPro_main.h"
#include "driver/ledc.h"

// Forward declarations for functions from other modules
void readEEP();    // from eep.cpp
void canInit();    // from can.cpp

void basicInit();
void testSpeed();
void needleSweep();

#endif // IO_H
