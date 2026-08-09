#include "Arduino.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include "SpeedPulserPro_wifi.h"
#include "power_manager.h"
#include "ota_manager.h"
#include "SpeedPulserPro_motorCal.h"
#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_tasks.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_gps.h"
#include "SpeedPulserPro_savvycan.h"
#include "SpeedPulserPro_calBuilder.h"

static uint32_t parseHexCanId(const String& text, uint32_t defaultVal) {
  String s = text;
  s.trim();
  if (s.startsWith("0x") || s.startsWith("0X")) {
    s = s.substring(2);
  }
  if (s.length() == 0) {
    return defaultVal;
  }
  return (uint32_t)strtoul(s.c_str(), nullptr, 16) & 0x7FF;
}

void handleGetSettings(AsyncWebServerRequest *request)
{
  DEBUG_WIFI("GET /api/settings");

  JsonDocument doc;
  doc["hasNeedleSweep"] = hasNeedleSweep;
  doc["linearSpeedSweep"] = linearSpeedSweep;
  doc["coilType"] = coilType;
  doc["convertToMPH"] = convertToMPH;
  doc["broadcastSpeedEnabled"] = broadcastSpeedEnabled;
  doc["broadcastSpeedID"] = broadcastSpeedID;
  doc["broadcastSpeedDLC"] = broadcastSpeedDLC;
  doc["broadcastSpeedLowByte"] = broadcastSpeedLowByte;
  doc["broadcastSpeedHighByte"] = broadcastSpeedHighByte;
  doc["broadcastSpeedLittleEndian"] = broadcastSpeedLittleEndian;
  doc["broadcastSpeedScale"] = broadcastSpeedScale;
  doc["broadcastSpeedOffset"] = broadcastSpeedOffset;
  for (uint8_t i = 0; i < 8; i++) {
    String dk = "broadcastSpeedData" + String(i);
    doc[dk] = broadcastSpeedData[i];
  }
  doc["sweepSpeed"] = sweepSpeed;
  doc["stepRPM"] = stepRPM;
  doc["stepSpeed"] = stepSpeed;
  doc["testSpeedo"] = testSpeedo;
  doc["tempSpeed"] = tempSpeed;
  doc["testRPM"] = testRPM;
  doc["tempRPM"] = tempRPM;
  doc["testCal"] = testCal;
  doc["tempDutyCycle"] = tempDutyCycle;
  doc["maxRPM"] = maxRPM;
  doc["clusterRPMLimit"] = clusterRPMLimit;
  doc["motorCalibration"] = motorPerformanceVal;
  doc["maxSpeed"] = maxSpeed;
  doc["maxFreqHall"] = maxFreqHall;
  doc["speedOffset"] = speedOffset;
  doc["speedOffsetPositive"] = speedOffsetPositive;
  doc["useGlobalSpeedOffset"] = useGlobalSpeedOffset;
  doc["useSpeedOffsetCurve"] = useSpeedOffsetCurve;
  JsonArray speedCurveOffsets = doc["speedOffsetCurveOffsets"].to<JsonArray>();
  for (uint8_t i = 0; i < SPEED_OFFSET_CURVE_POINTS; i++)
  {
    speedCurveOffsets.add(speedOffsetCurveOffsets[i]);
  }
  doc["averageFilterHall"] = averageFilterHall;
  doc["averageFilterRPM"] = averageFilterRPM;
  doc["averageFilter"] = averageFilterHall;
  doc["gpsUpdateRateHz"] = gpsUpdateRateHz;
  doc["speedOffsetType"] = useSpeedOffsetCurve ? "Curve" : (useGlobalSpeedOffset ? "Global" : "Off");
  doc["currentSpeedOffset"] = currentSpeedOffset;

  // Speed type mapping
  if (useHall)
    doc["speedType"] = "Hall";
  else if (useECU)
    doc["speedType"] = "ECU";
  else if (useABS)
    doc["speedType"] = "ABS";
  else if (useDSG)
    doc["speedType"] = "DSG";
  else if (useGPS)
    doc["speedType"] = "GPS";
  else if (useTP20)
    doc["speedType"] = "TP2.0";
  else if (useUDS)
    doc["speedType"] = "UDS";
  else if (useAftermarket)
    doc["speedType"] = "Custom CAN";
  else
    doc["speedType"] = "Hall";

  doc["aftermarketSpeedID"] = aftermarketSpeedID;
  doc["aftermarketSpeedLowByte"] = aftermarketSpeedLowByte;
  doc["aftermarketSpeedHighByte"] = aftermarketSpeedHighByte;
  doc["aftermarketSpeedLittleEndian"] = aftermarketSpeedLittleEndian;
  doc["aftermarketSpeedScale"] = aftermarketSpeedScale;
  doc["aftermarketSpeedOffset"] = aftermarketSpeedOffset;

  if (useRPMCAN)
    doc["rpmType"] = "CAN";
  else
    doc["rpmType"] = "Hall";

  doc["analyzerMode"] = analyzerMode;
  doc["analyzerSerial"] = analyzerSerial;

  doc["reverseDirection"] = reverseDirection;

  // Closed-loop feedback (PID)
  doc["feedbackEnable"] = feedbackEnable;
  doc["pidKp"] = pidKp;
  doc["pidKi"] = pidKi;
  doc["pidKd"] = pidKd;
  doc["feedbackDeadband"] = feedbackDeadband;
  doc["feedbackMinSpeed"] = feedbackMinSpeed;
  doc["feedbackMaxFreq"] = feedbackMaxFreq;

  doc["FW_VERSION"] = FW_VERSION;

  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

void handleGetStatus(AsyncWebServerRequest *request)
{
  JsonDocument doc;

  // Live data
  doc["vehicleRPM"] = vehicleRPM;
  doc["hallRPM"] = vehicleRPMHall;
  doc["canRPM"] = vehicleRPMCAN;
  doc["vehicleSpeed"] = vehicleSpeed;
  doc["hallSpeed"] = hallSpeed;
  doc["vehicleSpeedHall"] = vehicleSpeedHall;
  doc["ecuSpeed"] = ecuSpeed;
  doc["absSpeed"] = absSpeed;
  doc["dsgSpeed"] = dsgSpeed;
  doc["gpsSpeed"] = gpsSpeed;
  doc["udsSpeed"] = udsSpeed;
  doc["tp20Speed"] = tp20Speed;

  // Test mode status
  doc["testSpeedo"] = testSpeedo;
  doc["tempSpeed"] = tempSpeed;
  doc["testRPM"] = testRPM;
  doc["tempRPM"] = tempRPM;
  doc["testCal"] = testCal;
  doc["tempDutyCycle"] = tempDutyCycle;
  doc["appliedDutyCycle"] = appliedDutyCycle;
  doc["speedOffsetType"] = useSpeedOffsetCurve ? "Curve" : (useGlobalSpeedOffset ? "Global" : "Off");
  doc["currentSpeedOffset"] = currentSpeedOffset;
  doc["averageFilterHall"] = averageFilterHall;
  doc["averageFilterRPM"] = averageFilterRPM;

  // System status
  doc["hasCAN"] = hasCAN;
  doc["hasGPS"] = hasGPS;
  doc["convertToMPH"] = convertToMPH;
  doc["broadcastSpeedEnabled"] = broadcastSpeedEnabled;
  doc["broadcastSpeedValue"] = broadcastSpeedValue;
  doc["aftermarketSpeed"] = aftermarketSpeed;
  doc["gpsUnavailable"] = gpsUnavailable;
  doc["gpsSatellites"] = gps.satellites.value();
  // GPS update frequency
  doc["gpsFrequency"] = getGPSUpdateFrequency();
  doc["gpsAutoApplySecs"] = gpsAutoApplySecondsRemaining();

  // Closed-loop feedback (PID) live status
  doc["feedbackEnable"] = feedbackEnable;
  doc["reverseDirection"] = reverseDirection;
  doc["measuredSpeed"] = measuredSpeed;
  doc["pidCorrection"] = pidCorrection;
  doc["measuredFreqHz"] = measuredFreqHz;
  doc["feedbackAvailable"] = feedbackAvailable;
  doc["feedbackMissing"] = feedbackMissing;

  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

void handleGetCalibrations(AsyncWebServerRequest *request)
{
  DEBUG_WIFI("GET /api/calibrations");

  JsonDocument doc;
  JsonArray calibrations = doc["calibrations"].to<JsonArray>();
  const uint8_t calibrationCount = getCalibrationCount();

  for (uint8_t index = 1; index <= calibrationCount; index++)
  {
    JsonObject item = calibrations.add<JsonObject>();
    item["id"] = index;
    item["name"] = getCalibrationText(index);
  }

  // Custom (SpeedPulser) calibration slot — only offered once it has anchors.
  if (customCalCount >= 2)
  {
    JsonObject item = calibrations.add<JsonObject>();
    item["id"] = CUSTOM_CAL_ID;
    item["name"] = String("\u2605 Custom: ") + customCalName;
  }

  doc["currentCalibrationId"] = motorPerformanceVal;

  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

// GET /api/calcurve - Return the active calibration curve (duty vs speed) for
// the live Dashboard graph. Samples the live feed-forward mapping so the trace
// reflects both the built-in presets and a custom-built calibration (12-bit duty).
void handleGetCalCurve(AsyncWebServerRequest *request)
{
  JsonDocument doc;
  doc["pwmMax"]          = PWM_DUTY_MAX;
  doc["maxSpeed"]        = maxSpeed;
  doc["calibrationText"] = getCalibrationText(motorPerformanceVal);
  doc["custom"]          = (motorPerformanceVal == CUSTOM_CAL_ID && customCalValid);

  JsonArray speeds = doc["speed"].to<JsonArray>();
  JsonArray duties = doc["duty"].to<JsonArray>();

  uint16_t top = (maxSpeed < 10) ? 200 : maxSpeed;
  uint16_t step = top / 80;
  if (step < 1) step = 1;
  for (uint16_t s = 0; s <= top; s += step)
  {
    speeds.add(s);
    duties.add((uint32_t)speedToPwmDuty(s));
  }
  if ((top % step) != 0) // always include the exact max-speed point
  {
    speeds.add(top);
    duties.add((uint32_t)speedToPwmDuty(top));
  }

  // Anchor points for the ACTIVE cal: the real captured points for a custom cal,
  // or reference marks sampled on the curve for a preset. Both lie on the curve.
  JsonArray anchorSpeeds = doc["anchorSpeed"].to<JsonArray>();
  JsonArray anchorDuties = doc["anchorDuty"].to<JsonArray>();
  if (motorPerformanceVal == CUSTOM_CAL_ID && customCalValid)
  {
    for (uint8_t i = 0; i < customCalCount; i++)
    {
      anchorSpeeds.add(customCalPoints[i].speed);
      anchorDuties.add(customCalPoints[i].duty);
    }
  }
  else
  {
    for (uint16_t s = 20; s <= top; s += 20)
    {
      uint32_t d = (uint32_t)speedToPwmDuty(s);
      if (d == 0) continue;
      anchorSpeeds.add(s);
      anchorDuties.add(d);
    }
  }

  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

// Serialise the current calibration-builder state. Shared by GET /api/cal and
// the POST responses so the UI always gets a fresh view.
static void fillCalState(JsonDocument &doc)
{
  doc["name"]     = customCalName;
  doc["unit"]     = customCalUnitMph ? "mph" : "kmh";
  doc["convertToMPH"] = convertToMPH;   // lets the UI mirror an auto-enabled MPH cluster
  doc["count"]    = customCalCount;
  doc["valid"]    = customCalValid;
  doc["selected"] = (motorPerformanceVal == CUSTOM_CAL_ID);
  doc["duty"]     = (uint16_t)tempDutyCycle;
  doc["pwmMax"]   = PWM_DUTY_MAX;
  doc["maxSpeed"] = maxSpeed;

  JsonArray pts = doc["points"].to<JsonArray>();
  for (uint8_t i = 0; i < customCalCount; i++)
  {
    JsonObject p = pts.add<JsonObject>();
    p["speed"] = customCalPoints[i].speed;
    p["duty"]  = customCalPoints[i].duty;
  }
}

// GET /api/cal - Return the custom calibration builder state
void handleGetCal(AsyncWebServerRequest *request)
{
  JsonDocument doc;
  fillCalState(doc);
  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

// POST /api/cal - Custom calibration builder operations
// { "op": "jog|setDuty|addPoint|deletePoint|clearPoints|setName|apply|save|export|import", ... }
void handlePostCal(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  if (index + len != total) return; // wait for complete payload

  JsonDocument doc;
  if (deserializeJson(doc, data, len))
  {
    request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  const char *op = doc["op"];
  if (!op)
  {
    request->send(400, "application/json", "{\"error\":\"Missing op\"}");
    return;
  }

  if (strcmp(op, "jog") == 0)
  {
    int32_t delta = doc["delta"] | 0;
    int32_t range = (int32_t)PWM_DUTY_MAX + 1;
    int32_t next = ((int32_t)tempDutyCycle + delta) % range;
    if (next < 0) next += range;
    tempDutyCycle = next;
  }
  else if (strcmp(op, "setDuty") == 0)
  {
    int32_t duty = doc["duty"] | 0;
    if (duty < 0) duty = 0;
    if (duty > (int32_t)PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
    tempDutyCycle = duty;
  }
  else if (strcmp(op, "addPoint") == 0)
  {
    uint16_t speed = doc["speed"] | 0;
    uint16_t duty  = doc["duty"].isNull() ? (uint16_t)tempDutyCycle : doc["duty"].as<uint16_t>();
    if (!calAddPoint(speed, duty))
    {
      request->send(409, "application/json", "{\"error\":\"Point list full\"}");
      return;
    }
  }
  else if (strcmp(op, "deletePoint") == 0)
  {
    uint8_t idx = doc["index"] | 0;
    calDeletePoint(idx);
  }
  else if (strcmp(op, "clearPoints") == 0)
  {
    calClearPoints();
  }
  else if (strcmp(op, "setName") == 0)
  {
    const char *name = doc["name"];
    if (name) calSetName(name);
    customCalUnitMph = convertToMPH;
  }
  else if (strcmp(op, "apply") == 0)
  {
    buildCustomCalTable();
    if (customCalValid)
    {
      motorPerformanceVal = CUSTOM_CAL_ID;
      updateMotorPerformance = true;
    }
  }
  else if (strcmp(op, "save") == 0)
  {
    customCalUnitMph = convertToMPH;
    calSaveToNvs();
    buildCustomCalTable();
    if (customCalValid)
    {
      motorPerformanceVal = CUSTOM_CAL_ID; // persisted by the periodic writeEEP
      updateMotorPerformance = true;
    }
  }
  else if (strcmp(op, "export") == 0)
  {
    JsonDocument out;
    String json, carray;
    calExportJson(json);
    calExportCArray(carray);
    out["json"]   = json;
    out["carray"] = carray;
    String response;
    serializeJson(out, response);
    request->send(200, "application/json", response);
    return;
  }
  else if (strcmp(op, "import") == 0)
  {
    const char *json = doc["json"];
    if (!json || !calImportJson(json))
    {
      request->send(400, "application/json", "{\"error\":\"Import failed\"}");
      return;
    }
  }
  else
  {
    request->send(400, "application/json", "{\"error\":\"Unknown op\"}");
    return;
  }

  // A custom cal captured in MPH implies the cluster reads MPH; auto-enable the
  // runtime "Cluster in MPH" conversion whenever such a cal is the active one.
  if (motorPerformanceVal == CUSTOM_CAL_ID && customCalValid && customCalUnitMph) {
    convertToMPH = true;
  }

  DEBUG_WIFI("cal op: %s (count=%u duty=%u)", op, (unsigned)customCalCount, (unsigned)tempDutyCycle);

  JsonDocument state;
  fillCalState(state);
  String response;
  serializeJson(state, response);
  request->send(200, "application/json", response);
}

void handlePostControl(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  if (index + len != total) return; // wait for complete body
  JsonDocument doc;
  deserializeJson(doc, data, len);

  String key = doc["key"];
  String value = doc["value"].as<String>();

  DEBUG_WIFI("POST /api/control - %s = %s", key.c_str(), value.c_str());

  // Parse and handle setting changes
  if (key == "hasNeedleSweep")
  {
    hasNeedleSweep = (value == "true" || value == "1");
  }

  if (key == "linearSpeedSweep")
  {
    linearSpeedSweep = (value == "true" || value == "1");
  }

  if (key == "coilType")
  {
    coilType = (value == "true" || value == "1");
  }

  if (key == "convertToMPH")
  {
    convertToMPH = (value == "true" || value == "1");
  }

  if (key == "broadcastSpeedEnabled")
  {
    broadcastSpeedEnabled = (value == "true" || value == "1");
  }

  if (key == "broadcastSpeedID")
  {
    broadcastSpeedID = parseHexCanId(value, broadcastSpeedID);
  }

  if (key == "broadcastSpeedDLC")
  {
    broadcastSpeedDLC = (uint8_t)constrain(value.toInt(), 0, 8);
  }

  if (key == "broadcastSpeedLowByte")
  {
    broadcastSpeedLowByte = (uint8_t)constrain(value.toInt(), 0, 7);
  }

  if (key == "broadcastSpeedHighByte")
  {
    broadcastSpeedHighByte = (uint8_t)constrain(value.toInt(), 0, 7);
  }

  if (key == "broadcastSpeedLittleEndian")
  {
    broadcastSpeedLittleEndian = (value == "true" || value == "1");
  }

  if (key == "broadcastSpeedScale")
  {
    broadcastSpeedScale = value.toFloat();
  }

  if (key == "broadcastSpeedOffset")
  {
    broadcastSpeedOffset = (int16_t)constrain(value.toInt(), -32768, 32767);
  }

  for (uint8_t i = 0; i < 8; i++) {
    String dk = "broadcastSpeedData" + String(i);
    if (key == dk) {
      broadcastSpeedData[i] = (uint8_t)constrain(value.toInt(), 0, 255);
    }
  }

  if (key == "aftermarketSpeedID")
  {
    aftermarketSpeedID = parseHexCanId(value, aftermarketSpeedID);
  }

  if (key == "aftermarketSpeedLowByte")
  {
    aftermarketSpeedLowByte = (uint8_t)constrain(value.toInt(), 0, 7);
  }

  if (key == "aftermarketSpeedHighByte")
  {
    aftermarketSpeedHighByte = (uint8_t)constrain(value.toInt(), 0, 7);
  }

  if (key == "aftermarketSpeedLittleEndian")
  {
    aftermarketSpeedLittleEndian = (value == "true" || value == "1");
  }

  if (key == "aftermarketSpeedScale")
  {
    aftermarketSpeedScale = value.toFloat();
  }

  if (key == "aftermarketSpeedOffset")
  {
    aftermarketSpeedOffset = (int16_t)constrain(value.toInt(), -32768, 32767);
  }

  if (key == "sweepSpeed")
  {
    sweepSpeed = value.toInt();
  }

  if (key == "stepRPM")
  {
    stepRPM = value.toInt();
  }

  if (key == "stepSpeed")
  {
    stepSpeed = value.toInt();
  }

  if (key == "testSpeedo")
  {
    testSpeedo = (value == "true" || value == "1");
  }

  if (key == "tempSpeed")
  {
    long requestedSpeed = value.toInt();
    if (requestedSpeed >= 0 && requestedSpeed <= maxSpeed)
    {
      tempSpeed = requestedSpeed;
    }
  }

  if (key == "testRPM")
  {
    testRPM = (value == "true" || value == "1");
  }

  if (key == "testCal")
  {
    testCal = (value == "true" || value == "1");
  }

  if (key == "maxSpeed")
  {
    maxSpeed = value.toInt();
  }

  if (key == "maxFreqHall")
  {
    maxFreqHall = value.toInt();
  }

  if (key == "speedOffset")
  {
    long newOffset = value.toInt();
    speedOffset = (uint8_t)constrain(newOffset, 0, 50);
  }

  if (key == "speedOffsetPositive")
  {
    speedOffsetPositive = (value == "true" || value == "1");
  }

  if (key == "useGlobalSpeedOffset")
  {
    useGlobalSpeedOffset = (value == "true" || value == "1");
  }

  if (key == "useSpeedOffsetCurve")
  {
    useSpeedOffsetCurve = (value == "true" || value == "1");
  }

  if (key == "curveOffset0")
  {
    speedOffsetCurveOffsets[0] = (int16_t)value.toInt();
    normaliseSpeedOffsetCurve();
  }

  if (key == "curveOffset1")
  {
    speedOffsetCurveOffsets[1] = (int16_t)value.toInt();
    normaliseSpeedOffsetCurve();
  }

  if (key == "curveOffset2")
  {
    speedOffsetCurveOffsets[2] = (int16_t)value.toInt();
    normaliseSpeedOffsetCurve();
  }

  if (key == "curveOffset3")
  {
    speedOffsetCurveOffsets[3] = (int16_t)value.toInt();
    normaliseSpeedOffsetCurve();
  }

  if (key == "curveOffset4")
  {
    speedOffsetCurveOffsets[4] = (int16_t)value.toInt();
    normaliseSpeedOffsetCurve();
  }

  if (key == "averageFilter" || key == "averageFilterHall")
  {
    long requestedSamples = value.toInt();
    averageFilterHall = (uint8_t)constrain(requestedSamples, 1, 10);
    resetHallMedianFilter();
  }

  if (key == "averageFilterRPM")
  {
    long requestedSamples = value.toInt();
    averageFilterRPM = (uint8_t)constrain(requestedSamples, 1, 10);
    resetRPMMedianFilter();
  }

  if (key == "tempRPM")
  {
    tempRPM = value.toInt();
  }

  if (key == "tempDutyCycle")
  {
    long requestedDuty = value.toInt();
    if (requestedDuty > 385)
    {
      requestedDuty = 0;
    }
    else if (requestedDuty < 0)
    {
      requestedDuty = 385;
    }
    tempDutyCycle = requestedDuty;
  }

  if (key == "maxRPM")
  {
    maxRPM = value.toInt();
  }

  if (key == "clusterRPMLimit")
  {
    clusterRPMLimit = value.toInt();
  }

  if (key == "motorCalibration")
  {
    motorPerformanceVal = value.toInt();
    updateMotorArray();
  }

  if (key == "motorCalSelection")
  {
    motorPerformanceVal = value.toInt();
    updateMotorArray();
  }

  if (key == "speedType")
  {
    // Handle speed source selection
    if (value == "Hall")
    {
      useHall = true;
      useDSG = false;
      useECU = false;
      useABS = false;
      useGPS = false;
      useUDS = false;
      useTP20 = false;
      useAftermarket = false;
    }

    if (value == "ECU")
    {
      useHall = false;
      useDSG = false;
      useECU = true;
      useABS = false;
      useGPS = false;
      useUDS = false;
      useTP20 = false;
      useAftermarket = false;
    }

    if (value == "DSG")
    {
      useHall = false;
      useDSG = true;
      useECU = false;
      useABS = false;
      useGPS = false;
      useUDS = false;
      useTP20 = false;
      useAftermarket = false;
    }

    if (value == "ABS")
    {
      useHall = false;
      useDSG = false;
      useECU = false;
      useABS = true;
      useGPS = false;
      useUDS = false;
      useTP20 = false;
      useAftermarket = false;
    }

    if (value == "GPS")
    {
      useHall = false;
      useDSG = false;
      useECU = false;
      useABS = false;
      useGPS = true;
      useUDS = false;
      useTP20 = false;
      useAftermarket = false;
    }

    if (value == "TP2.0")
    {
      useHall = false;
      useDSG = false;
      useECU = false;
      useABS = false;
      useGPS = false;
      useUDS = false;
      useTP20 = true;
      useAftermarket = false;
    }

    if (value == "UDS")
    {
      useHall = false;
      useDSG = false;
      useECU = false;
      useABS = false;
      useGPS = false;
      useUDS = true;
      useTP20 = false;
      useAftermarket = false;
    }

    if (value == "Custom CAN")
    {
      useHall = false;
      useDSG = false;
      useECU = false;
      useABS = false;
      useGPS = false;
      useUDS = false;
      useAftermarket = true;
    }
  }

  if (key == "rpmType")
  {
    if (value == "CAN")
    {
      useRPMHall = false;
      useRPMCAN = true;
    }
    else
    {
      useRPMHall = true;
      useRPMCAN = false;
    }
  }

  if (key == "analyzerMode") {
    analyzerMode = (value == "true" || value == "1");
    if (analyzerMode) analyzerSerial = false;  // mutually exclusive
    setAnalyzerMode(analyzerMode);
  }

  if (key == "analyzerSerial") {
    analyzerSerial = (value == "true" || value == "1");
    if (analyzerSerial) analyzerMode = false;  // mutually exclusive
    setAnalyzerSerialMode(analyzerSerial);
  }

  if (key == "reverseDirection")
  {
    reverseDirection = (value == "true" || value == "1");
    applyDirection(); // apply the new direction immediately
  }

  // ---- Closed-loop feedback (PID) ----
  if (key == "feedbackEnable")
  {
    feedbackEnable = (value == "true" || value == "1");
    resetPid(); // clear accumulators whenever the loop is toggled
  }

  if (key == "pidKp")
  {
    pidKp = constrain(value.toFloat(), 0.0f, 10.0f);
  }

  if (key == "pidKi")
  {
    pidKi = constrain(value.toFloat(), 0.0f, 20.0f);
  }

  if (key == "pidKd")
  {
    pidKd = constrain(value.toFloat(), 0.0f, 10.0f);
  }

  if (key == "feedbackDeadband")
  {
    feedbackDeadband = constrain(value.toFloat(), 0.0f, 20.0f);
  }

  if (key == "feedbackMinSpeed")
  {
    feedbackMinSpeed = (uint16_t)constrain(value.toInt(), 0, (long)maxSpeed);
  }

  if (key == "feedbackMaxFreq")
  {
    feedbackMaxFreq = (uint16_t)constrain(value.toInt(), 1, 2000);
  }

  request->send(200);
}

void handlePostAction(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  if (index + len != total) return; // wait for complete body
  JsonDocument doc;
  deserializeJson(doc, data, len);
  String action = doc["action"];

  if (action == "needleSweep")
  {
    tempNeedleSweep = true;
  }

  request->send(200);
}

void handlePostTestRPM(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  if (index + len != total) return; // wait for complete body
  JsonDocument doc;
  deserializeJson(doc, data, len);

  if (doc["enabled"].is<bool>())
  {
    testRPM = doc["enabled"];
  }

  if (doc["value"].is<int>())
  {
    tempRPM = doc["value"];
  }

  request->send(200);
}

void handlePostTestSpeed(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  if (index + len != total) return; // wait for complete body
  JsonDocument doc;
  deserializeJson(doc, data, len);

  if (doc["enabled"].is<bool>())
  {
    testSpeedo = doc["enabled"];
  }

  if (doc["value"].is<int>())
  {
    long requestedSpeed = doc["value"];
    if (requestedSpeed >= 0 && requestedSpeed <= maxSpeed)
    {
      tempSpeed = requestedSpeed;
    }
  }

  request->send(200);
}

void setupUI()
{
  DEBUG_WIFI("Setting up web server...");

  // Initialize LittleFS filesystem
  if (!LittleFS.begin(false))
  { // true = format if mount failed
    DEBUG_WIFI("LittleFS mount failed");
  }
  else
  {
    DEBUG_WIFI("LittleFS successfully mounted");
  }

  // Serve static files from LittleFS
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.serveStatic("/style.css", LittleFS, "/style.css");
  server.serveStatic("/app.js", LittleFS, "/app.js");

  // API routes for getting data
  server.on("/api/settings", HTTP_GET, handleGetSettings);
  server.on("/api/status", HTTP_GET, handleGetStatus);
  server.on("/api/calibrations", HTTP_GET, handleGetCalibrations);
  server.on("/api/calcurve", HTTP_GET, handleGetCalCurve);
  server.on("/api/cal", HTTP_GET, handleGetCal);

  // API routes for receiving commands
  server.on("/api/control", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostControl(request, data, len, index, total); });

  // Custom calibration builder operations
  server.on("/api/cal", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostCal(request, data, len, index, total); });

  server.on("/api/action", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostAction(request, data, len, index, total); });

  // Separate endpoints for RPM and Speed tests
  server.on("/api/testRPM", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostTestRPM(request, data, len, index, total); });

  server.on("/api/testSpeed", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostTestSpeed(request, data, len, index, total); });

  // OTA (firmware + LittleFS web UI) via the shared, project-agnostic module.
  // Registers POST /api/ota-update?mode=firmware|filesystem and GET /api/version.
  OtaInfo otaInfo;
  otaInfo.version = FW_VERSION;
  otaInfo.hardware = "ESP32";
  otaInfo.board = "DOIT ESP32 DEVKIT V1";
  otaBegin(server, otaInfo, (enableDebug && debugWifi));

    // New endpoint: Set GPS update rate
    server.on("/api/gpsRate", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (index + len != total) return;
      JsonDocument doc;
      deserializeJson(doc, data, len);
      uint8_t rate = doc["rate"];
      String resp;
      bool ok = setGPSUpdateRate(rate, resp);
      JsonDocument out;
      out["success"] = ok;
      out["message"] = resp;
      String response;
      serializeJson(out, response);
      request->send(ok ? 200 : 400, "application/json", response);
    });

  // Catch-all for 404
  server.onNotFound([](AsyncWebServerRequest *request)
                    { request->send(404, "text/plain", "Not Found"); });

  server.begin();

#if enableDebug && debugWifi
  DEBUG_WIFI("Web server started");
  DEBUG_WIFI("IP: %s", WiFi.softAPIP().toString().c_str());
#endif
}

void connectWifi()
{
  WiFi.setHostname(wifiHostName);
  DEBUG_WIFI("Beginning WiFi / creating Access Point...");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(wifiHostName);

#if enableDebug && debugWifi
  DEBUG_WIFI("AP SSID: %s", wifiHostName);
  DEBUG_WIFI("AP IP: %s", WiFi.softAPIP().toString().c_str());
#endif
}

void disconnectWifi()
{
  DEBUG_WIFI("Number of connections: %u", WiFi.softAPgetStationNum());

  if (WiFi.softAPgetStationNum() == 0)
  {
    DEBUG_WIFI("No connections, turning off");
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
  }
}

void updateLabels()
{
  // Update CAN health status based on last received message
  if ((millis() - lastCAN) > 500)
  {
    hasCAN = false;
  }
  else
  {
    hasCAN = true;
  }

  // Update motor performance if requested
  if (updateMotorPerformance)
  {
    updateMotorArray();
    updateMotorPerformance = false; // Reset flag
  }
}

// ----------------------------------------------------------------------------
// power_manager integration (universal reduced-power module)
// ----------------------------------------------------------------------------
// These override the weak hooks in power_manager.cpp. The device stays fully
// awake while ANY client is associated to the AP. Once the last client leaves,
// the manager's idle timer runs, then turns the radio off and drops the CPU
// clock. A power-cycle (ignition off/on) brings WiFi back automatically.

bool powerIsBusy()
{
  return WiFi.softAPgetStationNum() > 0;
}

// ACTIVE -> REDUCED: close the web server cleanly before the radio drops.
void powerOnEnterReduced()
{
  server.end();
}

// REDUCED -> ACTIVE: bring the AP and web server back. Routes are already
// registered (no need to re-run setupUI()), so we only restart the radio
// and the listener.
void powerOnExitReduced()
{
  connectWifi();
  server.begin();
}
