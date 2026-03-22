#include "SpeedPulserPro_eep.h"

void readEEP() {
#if serialDebugEEP
  DEBUG_PRINTLN("EEPROM initialising!");
#endif

  pref.begin("useHall", false);
  pref.begin("useDSG", false);
  pref.begin("useGPS", false);
  pref.begin("useABS", false);
  pref.begin("useECU", false);
  pref.begin("useRPMHall", false);
  pref.begin("useRPMCAN", false);
  pref.begin("broadcastSpeed", false);
  pref.begin("coilType", false);

  pref.begin("hasNeedleSweep", false);
  pref.begin("sweepSpeed", false);

  pref.begin("maxFreqHall", false);
  pref.begin("maxFreqCAN", false);
  pref.begin("maxSpeed", false);
  pref.begin("maxRPM", false);
  pref.begin("clusterRPMLimit", false);

  pref.begin("speedOffset", false);
  pref.begin("speedOffsetPositive", false);
  pref.begin("motorPerfVal", false);

  pref.begin("stepRPM", false);
  pref.begin("stepSpeed", false);

  if (pref.getUChar("motorPerfVal", 255) == 255) {
#if serialDebugEEP
    DEBUG_PRINTLN("First run, set Bluetooth module, write Software Version etc");
    DEBUG_PRINTLN(pref.getUChar("motorPerfVal", 255));
#endif
    pref.putBool("useHall", useHall);
    pref.putBool("useDSG", useDSG);
    pref.putBool("useGPS", useGPS);
    pref.putBool("useABS", useABS);
    pref.putBool("useECU", useABS);
    pref.putBool("useRPMHall", useRPMHall);
    pref.putBool("useRPMCAN", useRPMCAN);
    pref.putBool("broadcastSpeed", broadcastSpeed);
    pref.putBool("coilType", coilType);

    pref.putBool("hasNeedleSweep", hasNeedleSweep);
    pref.putUChar("sweepSpeed", sweepSpeed);

    pref.putUShort("maxFreqHall", maxFreqHall);
    pref.putUShort("maxFreqCAN", maxFreqCAN);
    pref.putUShort("maxSpeed", maxSpeed);
    pref.putUShort("maxRPM", maxRPM);
    pref.putUShort("clusterRPMLimit", clusterRPMLimit);

    pref.putUChar("speedOffset", speedOffset);
    pref.putBool("speedOffsetPositive", speedOffsetPositive);
    pref.putUChar("motorPerfVal", motorPerformanceVal);

    pref.putUShort("stepRPM", stepRPM);
    pref.putUShort("stepSpeed", stepSpeed);
  } else {
    useHall = pref.getBool("useHall", true);
    useDSG = pref.getBool("useDSG", false);
    useGPS = pref.getBool("useGPS", false);
    useABS = pref.getBool("useABS", false);
    useECU = pref.getBool("useECU", false);
    useRPMHall = pref.getBool("useRPMHall", true);
    useRPMCAN = pref.getBool("useRPMCAN", false);
    broadcastSpeed = pref.getBool("broadcastSpeed", false);
    coilType = pref.getBool("coilType", false);

    hasNeedleSweep = pref.getBool("hasNeedleSweep", false);
    sweepSpeed = pref.getUChar("sweepSpeed", 18);

    maxFreqHall = pref.getUShort("maxFreqHall", 200);
    maxFreqCAN = pref.getUShort("maxFreqCAN", 200);
    maxSpeed = pref.getUShort("maxSpeed", 200);
    maxRPM = pref.getUShort("maxRPM", 230);
    clusterRPMLimit = pref.getUShort("clusterRPMLimit", 7000);

    speedOffset = pref.getUChar("speedOffset", 0);
    speedOffsetPositive = pref.getBool("speedOffsetPositive", 0);
    motorPerformanceVal = pref.getUChar("motorPerfVal", 0);

    stepRPM = pref.getUShort("stepRPM", 12);
    stepSpeed = pref.getUShort("stepSpeed", 10);
  }
#if serialDebugEEP
  DEBUG_PRINTLN("EEPROM initialised with...");
  DEBUG_PRINTLN("Written EEPROM with data:...");
  DEBUG_PRINTLN(testSpeedo);
  DEBUG_PRINTLN(tempSpeed);
  DEBUG_PRINTLN(hasNeedleSweep);
  DEBUG_PRINTLN(sweepSpeed);
  DEBUG_PRINTLN(maxFreqHall);
  DEBUG_PRINTLN(maxFreqCAN);
  DEBUG_PRINTLN(maxSpeed);
  DEBUG_PRINTLN(speedOffset);
  DEBUG_PRINTLN(speedOffsetPositive);
  DEBUG_PRINTLN(motorPerformanceVal);
#endif
}

void writeEEP() {
#if serialDebugEEP
  DEBUG_PRINTLN("Writing EEPROM...");
#endif

  pref.putBool("useHall", useHall);
  pref.putBool("useDSG", useDSG);
  pref.putBool("useGPS", useGPS);
  pref.putBool("useABS", useABS);
  pref.putBool("useRPMHall", useRPMHall);
  pref.putBool("useRPMCAN", useRPMCAN);
  pref.putBool("broadcastSpeed", broadcastSpeed);
  pref.putBool("coilType", coilType);

  pref.putBool("hasNeedleSweep", hasNeedleSweep);
  pref.putUChar("sweepSpeed", sweepSpeed);

  pref.putUShort("maxFreqHall", maxFreqHall);
  pref.putUShort("maxFreqCAN", maxFreqCAN);
  pref.putUShort("maxSpeed", maxSpeed);
  pref.putUShort("maxRPM", maxRPM);
  pref.putUShort("clusterRPMLimit", clusterRPMLimit);

  pref.putUChar("speedOffset", speedOffset);
  pref.putBool("speedOffsetPositive", speedOffsetPositive);
  pref.putUChar("motorPerfVal", motorPerformanceVal);

  pref.putUShort("stepRPM", stepRPM);
  pref.putUShort("stepSpeed", stepSpeed);

#if serialDebugEEP
  DEBUG_PRINTLN("Written EEPROM with data:...");
  DEBUG_PRINTLN(testSpeedo);
  DEBUG_PRINTLN(tempSpeed);
  DEBUG_PRINTLN(hasNeedleSweep);
  DEBUG_PRINTLN(sweepSpeed);
  DEBUG_PRINTLN(maxFreqHall);
  DEBUG_PRINTLN(maxFreqCAN);
  DEBUG_PRINTLN(maxSpeed);
  DEBUG_PRINTLN(speedOffset);
  DEBUG_PRINTLN(speedOffsetPositive);
  DEBUG_PRINTLN(motorPerformanceVal);
#endif
}
