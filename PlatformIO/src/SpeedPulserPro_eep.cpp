#include "SpeedPulserPro_eep.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_voltage.h"
#include "SpeedPulserPro_driveCal.h"
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

// Keys older versions wrote that nothing reads any more (V5.09, mirrors SpeedPulser).
// Each stays until the setting that replaced it has been saved once, so a unit
// still migrates.
static void removeRetiredKeys()
{
  static const char *const RETIRED[] = {"avgFilterHall", "avgFilterVR", "fbMaxFreq", "pidKp", "pidKi", "pidKd",
                                        "vcEnable", "linSpeedSweep", "maxFreqCAN"};
  for (const char *key : RETIRED)
    if (settingsStore.isKey(key))
      settingsStore.remove(key);
  struct Replaced { const char *oldKey; const char *newKey; };
  static const Replaced REPLACED[] = {{"maxFreqHall", "hallPulsesKm"}, {"maxFreqVR", "vrPulsesKm"},
                                      {"speedOffset", "spdOffsetKph"}, {"spdOffsetPos", "spdOffsetKph"},
                                      {"useGlobalOffset", "spdOffsetKph"}, {"calTopSpd", "dcN"},
                                      {"calTopHz", "dcN"}, {"calTopV", "dcN"}, {"calTopPwm", "dcN"}};
  for (const Replaced &r : REPLACED)
    if (settingsStore.isKey(r.newKey) && settingsStore.isKey(r.oldKey))
      settingsStore.remove(r.oldKey);
}

#if SPP_HAS_VOLTAGE_CTRL
// A loaded drive table is checked (V5.09, mirrors SpeedPulser): marks in rising
// speed order, speeds 1-400, drives above 0 and no higher than the rail ceiling.
// Anything else is a corrupt table (wrong top-speed cap, wild drive): dropped.
static bool driveCalTableSane(const DriveCalPoint *pts, uint8_t n)
{
  for (uint8_t i = 0; i < n; i++)
  {
    const DriveCalPoint &p = pts[i];
    if (p.speed < 1 || p.speed > 400 || !(p.drive > 0.0f) || p.drive > RAIL_V_CEIL || !(p.hz >= 0.0f))
      return false;
    if (i > 0 && p.speed <= pts[i - 1].speed)
      return false;
  }
  return true;
}
#endif

void readEEP()
{
#if SPP_HAS_VOLTAGE_CTRL
  // V5.05 feedback defaults for the S3 (voltage) board, from the SpeedPulser V5.04
  // bench tuning (2026-11-15, TY3816B on a cluster): with the timed tacho reading and
  // the reference-model trim the loop holds 10-160 km/h within ~1 % and is stable, so
  // it ships ON with a tight deadband and works down to 8 (cluster units). The DevKit
  // (PWM-only) keeps its PID-path defaults. Only used where the unit has no saved value.
  feedbackEnable = true;
  feedbackDeadband = 0.3f;
  feedbackMinSpeed = 8;
#endif
  // Read GPS update rate (Hz), default 1
#if serialDebugEEP
  Serial.println("[EEP] EEPROM initialising!");
#endif

  // Every setting lives in ONE namespace, "useHall": up to V5.05 begin() was called
  // once per setting name, but only the first call opens a namespace. Key names are
  // kept as they were so every unit's saved settings still load.
  settingsStore.begin("useHall", false);

  if (settingsStore.getUChar("motorPerfVal", 255) == 255)
  {
#if serialDebugEEP
    Serial.println("[EEP] First run, set Bluetooth module, write Software Version etc");
    Serial.print("[EEP] motorPerfVal: ");
    Serial.println(settingsStore.getUChar("motorPerfVal", 255));
#endif
    writeEEP(); // first boot: store the defaults
  }
  else
  {
    useHall = settingsStore.getBool("useHall", true);
    useDSG = settingsStore.getBool("useDSG", false);
    useGPS = settingsStore.getBool("useGPS", false);
    useABS = settingsStore.getBool("useABS", false);
    useECU = settingsStore.getBool("useECU", false);
    useUDS = settingsStore.getBool("useUDS", false);
    useTP20 = settingsStore.getBool("useTP20", false);
    useRPMHall = settingsStore.getBool("useRPMHall", true);
    useRPMCAN = settingsStore.getBool("useRPMCAN", false);
    broadcastSpeedEnabled = settingsStore.getBool("brdSpeedEn", false);
    broadcastSpeedID = settingsStore.getUInt("brdSpeedID", HALDEX_ID) & 0x7FF;
    broadcastSpeedDLC = settingsStore.getUChar("brdSpeedDLC", 8);
    broadcastSpeedLowByte = settingsStore.getUChar("brdSpeedLoByte", 2);
    broadcastSpeedHighByte = settingsStore.getUChar("brdSpeedHiByte", 3);
    broadcastSpeedLittleEndian = settingsStore.getBool("brdSpeedLE", true);
    broadcastSpeedScale = constrain(settingsStore.getFloat("brdSpeedScale", 0.781f), -CAN_SCALE_MAX, CAN_SCALE_MAX);
    broadcastSpeedOffset = settingsStore.getShort("brdSpeedOffset", 0);
    for (uint8_t i = 0; i < 8; i++)
    {
      String dk = "brdSD" + String(i);
      broadcastSpeedData[i] = settingsStore.getUChar(dk.c_str(), 0);
    }
    coilType = settingsStore.getBool("coilType", true);
    clusterInMph = settingsStore.getBool("convertToMPH", false);

    needleSweepOnBoot = settingsStore.getBool("hasNeedleSweep", false);
    needleSweepSpeed = settingsStore.getUChar("sweepSpeed", 18);

    maxSpeed = constrain(settingsStore.getUShort("maxSpeed", 200), 20, 400);
    // Speed sensors (V5.06): pulses per km. Units from before carry their scale over
    // from Max Hall Frequency at Max Speed.
    if (settingsStore.isKey("hallPulsesKm"))
      hallPulsesPerKm = settingsStore.getFloat("hallPulsesKm", SENSOR_PULSES_PER_KM_DEFAULT);
    else if (settingsStore.getUShort("maxFreqHall", 0) > 0)
      hallPulsesPerKm = 3600.0f * settingsStore.getUShort("maxFreqHall", 200) / (float)maxSpeed;
    maxRPM = settingsStore.getUShort("maxRPM", 230);
    clusterRPMLimit = settingsStore.getUShort("clusterRPMLimit", 7000);

    // Global offset (V5.06): one signed value. Before: a size, a "positive" tick and an
    // on/off tick ("spdOffsetPos" since V5.03).
    if (settingsStore.isKey("spdOffsetKph"))
    {
      globalSpeedOffsetKph = settingsStore.getShort("spdOffsetKph", 0);
    }
    else if (settingsStore.getBool("useGlobalOffset", true))
    {
      const int16_t size = settingsStore.getUChar("speedOffset", 0);
      globalSpeedOffsetKph = settingsStore.getBool("spdOffsetPos", false) ? size : -size;
    }
    useSpeedOffsetCurve = settingsStore.getBool("useCurve", false);
    speedOffsetCurveOffsets[0] = settingsStore.getShort("curveO0", 0);
    speedOffsetCurveOffsets[1] = settingsStore.getShort("curveO1", 0);
    speedOffsetCurveOffsets[2] = settingsStore.getShort("curveO2", 0);
    speedOffsetCurveOffsets[3] = settingsStore.getShort("curveO3", 0);
    speedOffsetCurveOffsets[4] = settingsStore.getShort("curveO4", 0);
    presetId = settingsStore.getUChar("motorPerfVal", 0);

    stepRPM = settingsStore.getUShort("stepRPM", 14);
    stepSpeed = settingsStore.getUShort("stepSpeed", 17);

    uint8_t legacyAverage = settingsStore.getUChar("averageFilter", 6);
    inputSmoothingMs = settingsStore.getUShort("inSmoothMs", inputSmoothingMs);
    averageFilterRPM = settingsStore.getUChar("avgFilterRPM", legacyAverage);
    gpsUpdateRateHz = settingsStore.getUChar("gpsUpdateRateHz", 1);
    useAftermarket = settingsStore.getBool("useAftermarket", false);
    aftermarketSpeedID = settingsStore.getUInt("amSpeedID", 0x200) & 0x7FF;
    aftermarketSpeedLowByte = settingsStore.getUChar("amSpeedLow", 0);
    aftermarketSpeedHighByte = settingsStore.getUChar("amSpeedHigh", 1);
    aftermarketSpeedLittleEndian = settingsStore.getBool("amSpeedLE", true);
    aftermarketSpeedScale = constrain(settingsStore.getFloat("amSpeedScale", 1.0f), -CAN_SCALE_MAX, CAN_SCALE_MAX);
    aftermarketSpeedOffset = settingsStore.getShort("amSpeedOffset", 0);

    for (uint8_t i = 1; i <= 6; i++)
    {
      String rk = "dsgRatio" + String(i);
      float loaded = settingsStore.getFloat(rk.c_str(), dsgGearRatio[i]);
      if (loaded >= DSG_RATIO_MIN && loaded <= DSG_RATIO_MAX) dsgGearRatio[i] = loaded;
    }
    dsgFinalDrive14 = constrain(settingsStore.getFloat("dsgFinal14", dsgFinalDrive14), DSG_RATIO_MIN, DSG_RATIO_MAX);
    dsgFinalDrive56 = constrain(settingsStore.getFloat("dsgFinal56", dsgFinalDrive56), DSG_RATIO_MIN, DSG_RATIO_MAX);
    dsgTireCirc = constrain(settingsStore.getFloat("dsgTireCirc", dsgTireCirc), DSG_TIRE_CIRC_MIN, DSG_TIRE_CIRC_MAX);

    reverseDirection = settingsStore.getBool("revDir", false);

#if SPP_HAS_VOLTAGE_CTRL
    // S3 drive trim (the DevKit has no motor feedback: V5.06 removed the legacy PID)
    feedbackEnable = settingsStore.getBool("fbEnable", feedbackEnable);
    feedbackDeadband = constrain(settingsStore.getFloat("fbDeadband", feedbackDeadband), 0.0f, 5.0f);
    feedbackMinSpeed = constrain(settingsStore.getUShort("fbMinSpd", feedbackMinSpeed), 0, 100);
    pwmNominal = constrain(settingsStore.getFloat("vcPwmNom", 0.80f), 0.2f, 1.0f);
#endif

#if SPP_HAS_VR
    // VR (variable-reluctance) speed input
    useVR = settingsStore.getBool("useVR", false);
    if (settingsStore.isKey("vrPulsesKm"))
      vrPulsesPerKm = settingsStore.getFloat("vrPulsesKm", SENSOR_PULSES_PER_KM_DEFAULT);
    else if (settingsStore.getUShort("maxFreqVR", 0) > 0)
      vrPulsesPerKm = 3600.0f * settingsStore.getUShort("maxFreqVR", 200) / (float)maxSpeed;
#else
    // A settings image carried over from a VR-capable build may have "VR" as the
    // active source; with no VR input on this board make sure hall takes over.
    if (settingsStore.getBool("useVR", false) && !useDSG && !useGPS && !useABS && !useECU && !useUDS && !useTP20 && !useAftermarket)
      useHall = true;
#endif
#if SPP_HAS_SPEED_BIAS
    speedInputBias = settingsStore.getUChar("spdBias", SPEED_BIAS_NONE);
#endif
  }

#if SPP_HAS_VOLTAGE_CTRL
  // Test Board result and V5.01 drive settings + calibration marks. Read outside
  // the first-run branch: units updated from older firmware have none of these
  // keys yet, so they come up with the defaults / "not calibrated".
  railMaxV = constrain(settingsStore.getFloat("vcRailMax", railMaxV), RAIL_V_FLOOR, RAIL_V_CEIL);
  trimGain = constrain(settingsStore.getFloat("vcTrimG", trimGain), 0.0f, 5.0f);
  trimLimit = constrain(settingsStore.getFloat("vcTrimL", trimLimit), 0.0f, 0.5f);
  // The drive calibration table: the count is taken from the table's own size; the
  // stored count only says "cleared" (0). A non-zero count is written BEFORE the table
  // (V5.09), so a power cut between the two writes leaves the old table (or none),
  // never "0 marks" over a fresh one. A table cleared by V5.05 or older left its blob
  // behind with dcN = 0.
  driveCalCount = 0;
  const size_t dcBytes = settingsStore.getBytesLength("dcPts");
  uint8_t dcN = (dcBytes % sizeof(DriveCalPoint) == 0) ? (uint8_t)(dcBytes / sizeof(DriveCalPoint)) : 0;
  if (dcN > DRIVE_CAL_MAX || (settingsStore.isKey("dcN") && settingsStore.getUChar("dcN", 0) == 0))
    dcN = 0;
  if (dcN > 0)
  {
    settingsStore.getBytes("dcPts", driveCal, sizeof(DriveCalPoint) * dcN);
    driveCalCount = driveCalTableSane(driveCal, dcN) ? dcN : 0;
    if (!driveCalCount)
      DEBUG_EEP("drive cal table corrupt (%u marks): dropped", dcN);
    // V5.05 keeps each mark's source (eye / auto) in what used to be struct padding.
    // Tables saved before that ("dcSrcV" missing) hold whatever the padding was, and
    // every mark in them was set by eye.
    if (!settingsStore.isKey("dcSrcV"))
    {
      for (uint8_t i = 0; i < dcN; i++)
      {
        driveCal[i].src = DCAL_SRC_EYE;
        driveCal[i].rsv = 0;
      }
    }
  }
  else if (!settingsStore.isKey("dcN"))
  {
    // V5.00 stored one top-speed point; carry it over as the first mark.
    const uint16_t topSpd = settingsStore.getUShort("calTopSpd", 0);
    const float topHz = settingsStore.getFloat("calTopHz", 0.0f);
    const float topV = settingsStore.getFloat("calTopV", 0.0f);
    const float topPwm = settingsStore.getFloat("calTopPwm", 0.0f);
    if (topSpd > 0 && topV > 0.0f && topPwm > 0.0f)
    {
      driveCal[0] = {topSpd, DCAL_SRC_EYE, 0, topHz, topV * topPwm, topV, topPwm};
      driveCalCount = 1;
      DEBUG_EEP("migrated V5.00 top point %u = %.1f Hz @ %.2f V x %.0f%%", topSpd, topHz, topV, topPwm * 100.0f);
    }
  }
  // V5.05 Minimum Rail (SpeedPulser V5.04). A unit updated from V5.01-V5.04 has no
  // key: its marks were captured with the rail on the 3.0 V floor at the low end, so
  // keep that split (the same drive at 4.8 V would run the needle roughly twice as
  // fast) until the marks are recaptured. Uncalibrated units start on the 4.8 V default.
  if (settingsStore.isKey("vcRailMin"))
    railMinV = settingsStore.getFloat("vcRailMin", RAIL_V_MIN_DEFAULT);
  else
    railMinV = (driveCalCount > 0) ? RAIL_V_FLOOR : RAIL_V_MIN_DEFAULT;
  railMinV = constrain(railMinV, RAIL_V_FLOOR, railMaxV);
  driveCalBackupValid = settingsStore.isKey("dcBak");
  boardTestCode = settingsStore.getUChar("btCode", BT_NOT_RUN);
  for (uint8_t i = 0; i < BOARD_TEST_POINTS; i++)
  {
    char key[8];
    snprintf(key, sizeof key, "btV%u", (unsigned)i);
    boardTestMeasV[i] = settingsStore.getFloat(key, 0.0f);
  }
  railFitV0 = settingsStore.getFloat("btFitV0", RAIL_MODEL_V0);
  railFitSlope = settingsStore.getFloat("btFitK", RAIL_MODEL_SLOPE);
  railFitValid = (boardTestCode == BT_PASS && railFitSlope < -6.0f && railFitSlope > -16.0f &&
                  railFitV0 > 11.0f && railFitV0 < 16.0f); // V5.09: the 0 % duty end checked too
  if (!railFitValid)
  {
    railFitV0 = RAIL_MODEL_V0;
    railFitSlope = RAIL_MODEL_SLOPE;
  }
#endif

  hallPulsesPerKm = constrain(hallPulsesPerKm, SENSOR_PULSES_PER_KM_MIN, SENSOR_PULSES_PER_KM_MAX);
#if SPP_HAS_VR
  vrPulsesPerKm = constrain(vrPulsesPerKm, SENSOR_PULSES_PER_KM_MIN, SENSOR_PULSES_PER_KM_MAX);
#endif
  inputSmoothingMs = constrain(inputSmoothingMs, INPUT_SMOOTHING_MS_MIN, INPUT_SMOOTHING_MS_MAX);
  globalSpeedOffsetKph = constrain(globalSpeedOffsetKph, -50, 50);
  needleSweepSpeed = constrain(needleSweepSpeed, 1, 50);
  maxRPM = constrain(maxRPM, 10, 1000);
  clusterRPMLimit = constrain(clusterRPMLimit, 1000, 12000);
  averageFilterRPM = constrain(averageFilterRPM, 1, 10);
#if SPP_HAS_SPEED_BIAS
  if (speedInputBias > SPEED_BIAS_PULL_DOWN) speedInputBias = SPEED_BIAS_NONE;
#endif
  broadcastSpeedLowByte = constrain(broadcastSpeedLowByte, 0, 7);
  broadcastSpeedHighByte = constrain(broadcastSpeedHighByte, 0, 7);
  broadcastSpeedDLC = constrain(broadcastSpeedDLC, 0, 8);
  for (uint8_t i = 0; i < SPEED_OFFSET_CURVE_POINTS; i++)
    speedOffsetCurveOffsets[i] = constrain(speedOffsetCurveOffsets[i], -20, 20);
  removeRetiredKeys();
}

void writeEEP()
{
  SemaphoreHandle_t writeMutex = getEEPWriteMutex();
  if ((writeMutex == nullptr) || (xSemaphoreTake(writeMutex, pdMS_TO_TICKS(1000)) != pdTRUE))
  {
    return;
  }

#if serialDebugEEP
  Serial.println("[EEP] Writing EEPROM...");
#endif

  settingsStore.putBool("useHall", useHall);
  settingsStore.putBool("useDSG", useDSG);
  settingsStore.putBool("useGPS", useGPS);
  settingsStore.putBool("useABS", useABS);
  settingsStore.putBool("useECU", useECU);
  settingsStore.putBool("useUDS", useUDS);
  settingsStore.putBool("useTP20", useTP20);
  settingsStore.putBool("useRPMHall", useRPMHall);
  settingsStore.putBool("useRPMCAN", useRPMCAN);
  settingsStore.putBool("brdSpeedEn", broadcastSpeedEnabled);
  settingsStore.putUInt("brdSpeedID", broadcastSpeedID);
  settingsStore.putUChar("brdSpeedDLC", broadcastSpeedDLC);
  settingsStore.putUChar("brdSpeedLoByte", broadcastSpeedLowByte);
  settingsStore.putUChar("brdSpeedHiByte", broadcastSpeedHighByte);
  settingsStore.putBool("brdSpeedLE", broadcastSpeedLittleEndian);
  settingsStore.putFloat("brdSpeedScale", broadcastSpeedScale);
  settingsStore.putShort("brdSpeedOffset", broadcastSpeedOffset);
  for (uint8_t i = 0; i < 8; i++)
  {
    String dk = "brdSD" + String(i);
    settingsStore.putUChar(dk.c_str(), broadcastSpeedData[i]);
  }
  settingsStore.putBool("coilType", coilType);
  settingsStore.putBool("convertToMPH", clusterInMph);

  settingsStore.putBool("hasNeedleSweep", needleSweepOnBoot);
  settingsStore.putUChar("sweepSpeed", needleSweepSpeed);

  settingsStore.putFloat("hallPulsesKm", hallPulsesPerKm);
  settingsStore.putUShort("inSmoothMs", inputSmoothingMs);
  settingsStore.putUShort("maxSpeed", maxSpeed);
  settingsStore.putUShort("maxRPM", maxRPM);
  settingsStore.putUShort("clusterRPMLimit", clusterRPMLimit);

  settingsStore.putShort("spdOffsetKph", globalSpeedOffsetKph);
  settingsStore.putBool("useCurve", useSpeedOffsetCurve);
  settingsStore.putShort("curveO0", speedOffsetCurveOffsets[0]);
  settingsStore.putShort("curveO1", speedOffsetCurveOffsets[1]);
  settingsStore.putShort("curveO2", speedOffsetCurveOffsets[2]);
  settingsStore.putShort("curveO3", speedOffsetCurveOffsets[3]);
  settingsStore.putShort("curveO4", speedOffsetCurveOffsets[4]);
  settingsStore.putUChar("motorPerfVal", presetId);

  settingsStore.putUShort("stepRPM", stepRPM);
  settingsStore.putUShort("stepSpeed", stepSpeed);
  settingsStore.putUChar("avgFilterRPM", averageFilterRPM);

  settingsStore.putUChar("gpsUpdateRateHz", gpsUpdateRateHz);
  settingsStore.putBool("useAftermarket", useAftermarket);
  settingsStore.putUInt("amSpeedID", aftermarketSpeedID);
  settingsStore.putUChar("amSpeedLow", aftermarketSpeedLowByte);
  settingsStore.putUChar("amSpeedHigh", aftermarketSpeedHighByte);
  settingsStore.putBool("amSpeedLE", aftermarketSpeedLittleEndian);
  settingsStore.putFloat("amSpeedScale", aftermarketSpeedScale);
  settingsStore.putShort("amSpeedOffset", aftermarketSpeedOffset);

  for (uint8_t i = 1; i <= 6; i++)
  {
    String rk = "dsgRatio" + String(i);
    settingsStore.putFloat(rk.c_str(), dsgGearRatio[i]);
  }
  settingsStore.putFloat("dsgFinal14", dsgFinalDrive14);
  settingsStore.putFloat("dsgFinal56", dsgFinalDrive56);
  settingsStore.putFloat("dsgTireCirc", dsgTireCirc);

  settingsStore.putBool("revDir", reverseDirection);

#if SPP_HAS_VOLTAGE_CTRL
  // S3: drive trim + buck voltage control
  settingsStore.putBool("fbEnable", feedbackEnable);
  settingsStore.putFloat("fbDeadband", feedbackDeadband);
  settingsStore.putUShort("fbMinSpd", feedbackMinSpeed);
  settingsStore.putFloat("vcPwmNom", pwmNominal);
  settingsStore.putFloat("vcRailMax", railMaxV);
  settingsStore.putFloat("vcRailMin", railMinV);
  settingsStore.putFloat("vcTrimG", trimGain);
  settingsStore.putFloat("vcTrimL", trimLimit);
  {
    DriveCalPoint pts[DRIVE_CAL_MAX];
    const uint8_t marks = driveCalSnapshot(pts); // a consistent copy: another task may be changing it
    if (marks > 0)
    {
      settingsStore.putUChar("dcN", marks);                                // count first...
      settingsStore.putBytes("dcPts", pts, sizeof(DriveCalPoint) * marks); // ...then the table
    }
    else
    {
      settingsStore.putUChar("dcN", 0);
      if (settingsStore.isKey("dcPts"))
        settingsStore.remove("dcPts");
    }
    settingsStore.putUChar("dcSrcV", 1); // marks carry their source (V5.05)
  }
  if (!boardTestActive) // don't save a half-finished test
  {
    settingsStore.putUChar("btCode", boardTestCode);
    for (uint8_t i = 0; i < BOARD_TEST_POINTS; i++)
    {
      char key[8];
      snprintf(key, sizeof key, "btV%u", (unsigned)i);
      settingsStore.putFloat(key, boardTestMeasV[i]);
    }
    settingsStore.putFloat("btFitV0", railFitV0);
    settingsStore.putFloat("btFitK", railFitSlope);
  }
#endif

#if SPP_HAS_VR
  // VR (variable-reluctance) speed input
  settingsStore.putBool("useVR", useVR);
  settingsStore.putFloat("vrPulsesKm", vrPulsesPerKm);
#endif
#if SPP_HAS_SPEED_BIAS
  settingsStore.putUChar("spdBias", speedInputBias);
#endif


  xSemaphoreGive(writeMutex);
}
