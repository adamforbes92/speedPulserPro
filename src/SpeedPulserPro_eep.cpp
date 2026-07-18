#include "SpeedPulserPro_eep.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_gps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t eepWriteMutex = nullptr;

static SemaphoreHandle_t getEEPWriteMutex()
{
  if (eepWriteMutex == nullptr)
  {
    eepWriteMutex = xSemaphoreCreateMutex();
  }

  return eepWriteMutex;
}

void readEEP()
{
  // Read GPS update rate (Hz), default 1
#if serialDebugEEP
  Serial.println("[EEP] EEPROM initialising!");
#endif

  pref.begin("useHall", false);
  pref.begin("useDSG", false);
  pref.begin("useGPS", false);
  pref.begin("useABS", false);
  pref.begin("useECU", false);
  pref.begin("useUDS", false);
  pref.begin("useTP20", false);
  pref.begin("useRPMHall", false);
  pref.begin("useRPMCAN", false);
  pref.begin("brdSpeedEn", false);
  pref.begin("brdSpeedID", false);
  pref.begin("brdSpeedDLC", false);
  pref.begin("brdSpeedLoByte", false);
  pref.begin("brdSpeedHiByte", false);
  pref.begin("brdSpeedLE", false);
  pref.begin("brdSpeedScale", false);
  pref.begin("brdSpeedOffset", false);
  pref.begin("brdSD0", false);
  pref.begin("brdSD1", false);
  pref.begin("brdSD2", false);
  pref.begin("brdSD3", false);
  pref.begin("brdSD4", false);
  pref.begin("brdSD5", false);
  pref.begin("brdSD6", false);
  pref.begin("brdSD7", false);
  pref.begin("coilType", false);
  pref.begin("convertToMPH", false);

  pref.begin("hasNeedleSweep", false);
  pref.begin("linSpeedSweep", false);
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

  pref.begin("gpsUpdateRateHz", false);
  pref.begin("useAftermarket", false);
  pref.begin("amSpeedID", false);
  pref.begin("amSpeedLow", false);
  pref.begin("amSpeedHigh", false);
  pref.begin("amSpeedLE", false);
  pref.begin("amSpeedScale", false);
  pref.begin("amSpeedOffset", false);

  if (pref.getUChar("motorPerfVal", 255) == 255)
  {
#if serialDebugEEP
    Serial.println("[EEP] First run, set Bluetooth module, write Software Version etc");
    Serial.print("[EEP] motorPerfVal: ");
    Serial.println(pref.getUChar("motorPerfVal", 255));
#endif
    pref.putBool("useHall", useHall);
    pref.putBool("useDSG", useDSG);
    pref.putBool("useGPS", useGPS);
    pref.putBool("useABS", useABS);
    pref.putBool("useECU", useECU);
    pref.putBool("useUDS", useUDS);
    pref.putBool("useTP20", useTP20);
    pref.putBool("useRPMHall", useRPMHall);
    pref.putBool("useRPMCAN", useRPMCAN);
    pref.putBool("brdSpeedEn", broadcastSpeedEnabled);
    pref.putUInt("brdSpeedID", broadcastSpeedID);
    pref.putUChar("brdSpeedDLC", broadcastSpeedDLC);
    pref.putUChar("brdSpeedLoByte", broadcastSpeedLowByte);
    pref.putUChar("brdSpeedHiByte", broadcastSpeedHighByte);
    pref.putBool("brdSpeedLE", broadcastSpeedLittleEndian);
    pref.putFloat("brdSpeedScale", broadcastSpeedScale);
    pref.putShort("brdSpeedOffset", broadcastSpeedOffset);
    for (uint8_t i = 0; i < 8; i++)
    {
      String dk = "brdSD" + String(i);
      pref.putUChar(dk.c_str(), broadcastSpeedData[i]);
    }
    pref.putBool("coilType", coilType);
    pref.putBool("convertToMPH", convertToMPH);

    pref.putBool("hasNeedleSweep", hasNeedleSweep);
    pref.putBool("linSpeedSweep", linearSpeedSweep);
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
    pref.putUChar("gpsUpdateRateHz", gpsUpdateRateHz);
    pref.putBool("useAftermarket", useAftermarket);
    pref.putUInt("amSpeedID", aftermarketSpeedID);
    pref.putUChar("amSpeedLow", aftermarketSpeedLowByte);
    pref.putUChar("amSpeedHigh", aftermarketSpeedHighByte);
    pref.putBool("amSpeedLE", aftermarketSpeedLittleEndian);
    pref.putFloat("amSpeedScale", aftermarketSpeedScale);
    pref.putShort("amSpeedOffset", aftermarketSpeedOffset);
  }
  else
  {
    useHall = pref.getBool("useHall", true);
    useDSG = pref.getBool("useDSG", false);
    useGPS = pref.getBool("useGPS", false);
    useABS = pref.getBool("useABS", false);
    useECU = pref.getBool("useECU", false);
    useUDS = pref.getBool("useUDS", false);
    useTP20 = pref.getBool("useTP20", false);
    useRPMHall = pref.getBool("useRPMHall", true);
    useRPMCAN = pref.getBool("useRPMCAN", false);
    broadcastSpeedEnabled = pref.getBool("brdSpeedEn", false);
    broadcastSpeedID = pref.getUInt("brdSpeedID", HALDEX_ID) & 0x7FF;
    broadcastSpeedDLC = pref.getUChar("brdSpeedDLC", 8);
    broadcastSpeedLowByte = pref.getUChar("brdSpeedLoByte", 2);
    broadcastSpeedHighByte = pref.getUChar("brdSpeedHiByte", 3);
    broadcastSpeedLittleEndian = pref.getBool("brdSpeedLE", true);
    broadcastSpeedScale = pref.getFloat("brdSpeedScale", 0.781f);
    broadcastSpeedOffset = pref.getShort("brdSpeedOffset", 0);
    for (uint8_t i = 0; i < 8; i++)
    {
      String dk = "brdSD" + String(i);
      broadcastSpeedData[i] = pref.getUChar(dk.c_str(), 0);
    }
    coilType = pref.getBool("coilType", false);
    convertToMPH = pref.getBool("convertToMPH", false);

    hasNeedleSweep = pref.getBool("hasNeedleSweep", false);
    linearSpeedSweep = pref.getBool("linSpeedSweep", true);
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

    stepRPM = pref.getUShort("stepRPM", 14);
    stepSpeed = pref.getUShort("stepSpeed", 17);

    uint8_t legacyAverage = pref.getUChar("averageFilter", 6);
    averageFilterHall = pref.getUChar("avgFilterHall", legacyAverage);
    averageFilterRPM = pref.getUChar("avgFilterRPM", legacyAverage);
    gpsUpdateRateHz = pref.getUChar("gpsUpdateRateHz", 1);
    useAftermarket = pref.getBool("useAftermarket", false);
    aftermarketSpeedID = pref.getUInt("amSpeedID", 0x200) & 0x7FF;
    aftermarketSpeedLowByte = pref.getUChar("amSpeedLow", 0);
    aftermarketSpeedHighByte = pref.getUChar("amSpeedHigh", 1);
    aftermarketSpeedLittleEndian = pref.getBool("amSpeedLE", true);
    aftermarketSpeedScale = pref.getFloat("amSpeedScale", 1.0f);
    aftermarketSpeedOffset = pref.getShort("amSpeedOffset", 0);
  }

  averageFilterHall = constrain(averageFilterHall, 1, 10);
  averageFilterRPM = constrain(averageFilterRPM, 1, 10);
  broadcastSpeedLowByte = constrain(broadcastSpeedLowByte, 0, 7);
  broadcastSpeedHighByte = constrain(broadcastSpeedHighByte, 0, 7);
  broadcastSpeedDLC = constrain(broadcastSpeedDLC, 0, 8);
  normaliseSpeedOffsetCurve();
#if serialDebugEEP
  Serial.println("[EEP] EEPROM initialised with...");
  Serial.print("[EEP] testSpeedo: ");
  Serial.println(testSpeedo);
  Serial.print("[EEP] tempSpeed: ");
  Serial.println(tempSpeed);
  Serial.print("[EEP] hasNeedleSweep: ");
  Serial.println(hasNeedleSweep);
  Serial.print("[EEP] sweepSpeed: ");
  Serial.println(sweepSpeed);
  Serial.print("[EEP] maxFreqHall: ");
  Serial.println(maxFreqHall);
  Serial.print("[EEP] maxFreqCAN: ");
  Serial.println(maxFreqCAN);
  Serial.print("[EEP] maxSpeed: ");
  Serial.println(maxSpeed);
  Serial.print("[EEP] speedOffset: ");
  Serial.println(speedOffset);
  Serial.print("[EEP] speedOffsetPositive: ");
  Serial.println(speedOffsetPositive);
  Serial.print("[EEP] motorPerformanceVal: ");
  Serial.println(motorPerformanceVal);
#endif
}

void writeEEP()
{
  SemaphoreHandle_t writeMutex = getEEPWriteMutex();
  if ((writeMutex == nullptr) || (xSemaphoreTake(writeMutex, pdMS_TO_TICKS(1000)) != pdTRUE))
  {
    return;
  }

  // Write GPS update rate
  pref.putUChar("gpsUpdateRateHz", gpsUpdateRateHz);
#if serialDebugEEP
  Serial.println("[EEP] Writing EEPROM...");
#endif

  pref.putBool("useHall", useHall);
  pref.putBool("useDSG", useDSG);
  pref.putBool("useGPS", useGPS);
  pref.putBool("useABS", useABS);
  pref.putBool("useECU", useECU);
  pref.putBool("useUDS", useUDS);
  pref.putBool("useTP20", useTP20);
  pref.putBool("useRPMHall", useRPMHall);
  pref.putBool("useRPMCAN", useRPMCAN);
  pref.putBool("brdSpeedEn", broadcastSpeedEnabled);
  pref.putUInt("brdSpeedID", broadcastSpeedID);
  pref.putUChar("brdSpeedDLC", broadcastSpeedDLC);
  pref.putUChar("brdSpeedLoByte", broadcastSpeedLowByte);
  pref.putUChar("brdSpeedHiByte", broadcastSpeedHighByte);
  pref.putBool("brdSpeedLE", broadcastSpeedLittleEndian);
  pref.putFloat("brdSpeedScale", broadcastSpeedScale);
  pref.putShort("brdSpeedOffset", broadcastSpeedOffset);
  for (uint8_t i = 0; i < 8; i++)
  {
    String dk = "brdSD" + String(i);
    pref.putUChar(dk.c_str(), broadcastSpeedData[i]);
  }
  pref.putBool("coilType", coilType);
  pref.putBool("convertToMPH", convertToMPH);

  pref.putBool("hasNeedleSweep", hasNeedleSweep);
  pref.putBool("linSpeedSweep", linearSpeedSweep);
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

  pref.putUChar("gpsUpdateRateHz", gpsUpdateRateHz);
  pref.putBool("useAftermarket", useAftermarket);
  pref.putUInt("amSpeedID", aftermarketSpeedID);
  pref.putUChar("amSpeedLow", aftermarketSpeedLowByte);
  pref.putUChar("amSpeedHigh", aftermarketSpeedHighByte);
  pref.putBool("amSpeedLE", aftermarketSpeedLittleEndian);
  pref.putFloat("amSpeedScale", aftermarketSpeedScale);
  pref.putShort("amSpeedOffset", aftermarketSpeedOffset);

#if serialDebugEEP
  Serial.println("[EEP] Written EEPROM with data:...");
  Serial.print("[EEP] testSpeedo: ");
  Serial.println(testSpeedo);
  Serial.print("[EEP] tempSpeed: ");
  Serial.println(tempSpeed);
  Serial.print("[EEP] hasNeedleSweep: ");
  Serial.println(hasNeedleSweep);
  Serial.print("[EEP] sweepSpeed: ");
  Serial.println(sweepSpeed);
  Serial.print("[EEP] maxFreqHall: ");
  Serial.println(maxFreqHall);
  Serial.print("[EEP] maxFreqCAN: ");
  Serial.println(maxFreqCAN);
  Serial.print("[EEP] maxSpeed: ");
  Serial.println(maxSpeed);
  Serial.print("[EEP] speedOffset: ");
  Serial.println(speedOffset);
  Serial.print("[EEP] speedOffsetPositive: ");
  Serial.println(speedOffsetPositive);
  Serial.print("[EEP] motorPerformanceVal: ");
  Serial.println(motorPerformanceVal);
#endif

  xSemaphoreGive(writeMutex);
}
