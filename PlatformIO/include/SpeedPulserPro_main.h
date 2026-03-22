#ifndef MAIN_H
#define MAIN_H

// Forward declarations for functions defined in main.cpp
void incomingHz();
void incomingMotorSpeed();
void setFrequencyRPM(long frequencyHz);
uint16_t findClosestMatch(uint16_t val);

#endif // MAIN_H
