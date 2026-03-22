#include "SpeedPulserPro_eep.h"
#include "SpeedPulserPro_control.h"

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
  pref.begin("useGlobalOffset", false);
  pref.begin("useCurve", false);
  pref.begin("curveO0", false);
  pref.begin("curveO1", false);
  pref.begin("curveO2", false);
  pref.begin("curveO3", false);
  pref.begin("curveO4", false);
  pref.begin("motorPerfVal", false);

  pref.begin("stepRPM", false);
  pref.begin("stepSpeed", false);
  pref.begin("averageFilter", false);
  pref.begin("avgFilterHall", false);
  pref.begin("avgFilterRPM", false);

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
    pref.putBool("useGlobalOffset", useGlobalSpeedOffset);
    pref.putBool("useCurve", useSpeedOffsetCurve);
    pref.putShort("curveO0", speedOffsetCurveOffsets[0]);
    pref.putShort("curveO1", speedOffsetCurveOffsets[1]);
    pref.putShort("curveO2", speedOffsetCurveOffsets[2]);
    pref.putShort("curveO3", speedOffsetCurveOffsets[3]);
    pref.putShort("curveO4", speedOffsetCurveOffsets[4]);
    pref.putUChar("motorPerfVal", motorPerformanceVal);

    pref.putUShort("stepRPM", stepRPM);
    pref.putUShort("stepSpeed", stepSpeed);
    pref.putUChar("averageFilter", averageFilterHall);
    pref.putUChar("avgFilterHall", averageFilterHall);
    pref.putUChar("avgFilterRPM", averageFilterRPM);
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
    useGlobalSpeedOffset = pref.getBool("useGlobalOffset", true);
    useSpeedOffsetCurve = pref.getBool("useCurve", false);
    speedOffsetCurveOffsets[0] = pref.getShort("curveO0", 0);
    speedOffsetCurveOffsets[1] = pref.getShort("curveO1", 0);
    speedOffsetCurveOffsets[2] = pref.getShort("curveO2", 0);
    speedOffsetCurveOffsets[3] = pref.getShort("curveO3", 0);
    speedOffsetCurveOffsets[4] = pref.getShort("curveO4", 0);
    motorPerformanceVal = pref.getUChar("motorPerfVal", 0);

    stepRPM = pref.getUShort("stepRPM", 12);
    stepSpeed = pref.getUShort("stepSpeed", 10);

    uint8_t legacyAverage = pref.getUChar("averageFilter", 6);
    averageFilterHall = pref.getUChar("avgFilterHall", legacyAverage);
    averageFilterRPM = pref.getUChar("avgFilterRPM", legacyAverage);
  }

  averageFilterHall = constrain(averageFilterHall, 1, 10);
  averageFilterRPM = constrain(averageFilterRPM, 1, 10);
  normaliseSpeedOffsetCurve();
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
  pref.putBool("useGlobalOffset", useGlobalSpeedOffset);
  pref.putBool("useCurve", useSpeedOffsetCurve);
  pref.putShort("curveO0", speedOffsetCurveOffsets[0]);
  pref.putShort("curveO1", speedOffsetCurveOffsets[1]);
  pref.putShort("curveO2", speedOffsetCurveOffsets[2]);
  pref.putShort("curveO3", speedOffsetCurveOffsets[3]);
  pref.putShort("curveO4", speedOffsetCurveOffsets[4]);
  pref.putUChar("motorPerfVal", motorPerformanceVal);

  pref.putUShort("stepRPM", stepRPM);
  pref.putUShort("stepSpeed", stepSpeed);
  pref.putUChar("averageFilter", averageFilterHall);
  pref.putUChar("avgFilterHall", averageFilterHall);
  pref.putUChar("avgFilterRPM", averageFilterRPM);

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
