#include "Arduino.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include "SpeedPulserPro_wifi.h"
#include "power_manager.h"
#include "ota_manager.h"
#include "wifi_manager.h"
#include "SpeedPulserPro_motorCal.h"
#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_tasks.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_gps.h"
#include "SpeedPulserPro_savvycan.h"
#include "SpeedPulserPro_calBuilder.h"
#include "SpeedPulserPro_voltage.h"
#include "SpeedPulserPro_driveCal.h"

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

// What is actually driving the motor, for the "Cal:" banner. With voltage control
// on (S3 board), the PWM preset is idle and the drive marks decide the speed, so
// say so. calMode "pwm" leaves the banner on the selected preset.
static void addCalText(JsonDocument &doc)
{
#if SPP_HAS_VOLTAGE_CTRL
  if (voltageControlEnable)
  {
    doc["calMode"] = "voltage";
    if (driveCalCount == 0)
      doc["calibrationText"] = "Voltage (not calibrated)";
    else
      doc["calibrationText"] = "Voltage (" + String(driveCalCount) + (driveCalCount == 1 ? " mark, " : " marks, ") +
                               String(driveCal[0].speed) + "-" + String(driveCalTopSpeed()) + ")";
    return;
  }
#endif
  doc["calMode"] = "pwm";
}

#if SPP_HAS_VOLTAGE_CTRL
// The drive calibration marks (S3 board), for /api/settings and the
// capture/delete/import responses.
static void addDriveCal(JsonDocument &doc)
{
  JsonArray pts = doc["driveCal"].to<JsonArray>();
  for (uint8_t i = 0; i < driveCalCount; i++)
  {
    JsonObject p = pts.add<JsonObject>();
    p["speed"] = driveCal[i].speed;
    p["hz"] = driveCal[i].hz;
    p["drive"] = driveCal[i].drive;
    p["rail"] = driveCal[i].rail;
    p["pwm"] = driveCal[i].pwm;
  }
  doc["driveCalHz"] = driveCalHzValid();
}

// Live S3-board read-outs for /api/status: rail, drive and tacho keep updating
// in every mode (Calibration, Speed Test and normal running).
static void addVoltageStatus(JsonDocument &doc)
{
  doc["boardHasVoltageControl"] = true;
  doc["hasRailSense"] = true;
  doc["voltageControlEnable"] = voltageControlEnable;
  doc["buckEnabled"] = buckEnabled;
  doc["voltageCmd"] = lastVoltageCmd;   // 0..1 of the rail window
  doc["pwmFrac"] = lastPwmFrac;         // 0..1 motor PWM
  doc["driveV"] = lastDriveV;           // applied drive (V = rail x PWM)
  doc["driveMaxV"] = driveMaxV();
  doc["tempDriveV"] = tempDriveV;       // calibration drive the user set
  doc["driveTrimPct"] = driveTrim * 100.0f;
  doc["railCmdV"] = lastRailCmdV;       // commanded rail (V)
  doc["motorRailV"] = motorRailV;       // measured rail (V) — trust it once railSenseOk
  doc["railSenseOk"] = railFitValid;
  doc["railLimited"] = railLimited();
  doc["boardTestRunning"] = (bool)boardTestActive;
  float fbMeanHz, fbSpreadPct, fbDriftHz;
  const bool fbWin = feedbackHzWindow(fbMeanHz, fbSpreadPct, fbDriftHz);
  doc["fbHzAvg"] = fbWin ? fbMeanHz : 0.0f;          // capture-window mean (Hz)
  doc["fbHzSpread"] = fbWin ? fbSpreadPct : -1.0f;   // % (max-min)/mean; -1 = not turning
  doc["fbHzDrift"] = fbWin ? fbDriftHz : 0.0f;       // Hz the mean moved across the window
  doc["fbStable"] = fbWin && feedbackHzSteady(fbMeanHz, fbSpreadPct, fbDriftHz);
}
#endif

// Name of the speed source that is really driving the needle. Checked in the
// opposite order to taskProcessSpeed() (where the LAST matching source wins), so
// the web page always shows the source the speed task is using.
static const char *activeSpeedSourceName()
{
  if (useAftermarket)
  {
    return "Custom CAN";
  }
  if (useTP20)
  {
    return "TP2.0";
  }
  if (useUDS)
  {
    return "UDS";
  }
  if (useGPS)
  {
    return "GPS";
  }
  if (useDSG)
  {
    return "DSG";
  }
  if (useABS)
  {
    return "ABS";
  }
  if (useECU)
  {
    return "ECU";
  }
#if SPP_HAS_VR
  if (useVR)
  {
    return "VR";
  }
#endif
  return "Hall";
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

  // Speed source shown in the dropdown: the one the speed task is really using
  doc["speedType"] = activeSpeedSourceName();

  doc["aftermarketSpeedID"] = aftermarketSpeedID;
  doc["aftermarketSpeedLowByte"] = aftermarketSpeedLowByte;
  doc["aftermarketSpeedHighByte"] = aftermarketSpeedHighByte;
  doc["aftermarketSpeedLittleEndian"] = aftermarketSpeedLittleEndian;
  doc["aftermarketSpeedScale"] = aftermarketSpeedScale;
  doc["aftermarketSpeedOffset"] = aftermarketSpeedOffset;

  // DSG speed calculation (RPM & gear -> speed)
  for (uint8_t i = 1; i <= 6; i++) {
    String rk = "dsgRatio" + String(i);
    doc[rk] = dsgGearRatio[i];
  }
  doc["dsgFinal14"] = dsgFinalDrive14;
  doc["dsgFinal56"] = dsgFinalDrive56;
  doc["dsgTireCirc"] = dsgTireCirc;

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

  // Board identity + optional hardware blocks. The UI hides every control whose
  // hardware isn't on this board (voltage control, rail sense, VR, speed bias).
  doc["boardId"] = SPP_BOARD_ID;
  doc["boardName"] = SPP_BOARD_NAME;
  doc["hasVoltageCtrl"] = (bool)SPP_HAS_VOLTAGE_CTRL;
  doc["hasRailSense"] = (bool)SPP_HAS_RAIL_SENSE;
  doc["hasVR"] = (bool)SPP_HAS_VR;
  doc["hasSpeedBias"] = (bool)SPP_HAS_SPEED_BIAS;

#if SPP_HAS_VOLTAGE_CTRL
  // Buck voltage control (S3 board)
  doc["boardHasVoltageControl"] = true;
  doc["boardIdStrapOk"] = boardIdStrapOk;
  doc["voltageControlEnable"] = voltageControlEnable;
  doc["railFloorV"] = RAIL_V_FLOOR;   // hardware rail window (V)
  doc["railCeilV"] = RAIL_V_CEIL;
  doc["railSenseOk"] = railFitValid;
  doc["boardTestCode"] = boardTestCode;
  doc["topSpeedCaptured"] = (driveCalCount > 0);
  doc["vcPwmNominal"] = vcPwmNominal;
  doc["vcRailMaxV"] = vcRailMaxV;
  doc["vcTrimGain"] = vcTrimGain;
  doc["vcTrimLimit"] = vcTrimLimit;
  doc["driveMaxV"] = driveMaxV();
  addDriveCal(doc);
#else
  doc["boardHasVoltageControl"] = false;
#endif
  addCalText(doc);

#if SPP_HAS_VR
  // VR (variable-reluctance) speed input
  doc["maxFreqVR"] = maxFreqVR;
  doc["averageFilterVR"] = averageFilterVR;
#endif
#if SPP_HAS_SPEED_BIAS
  doc["speedInputBias"] = speedInputBias; // 0 none, 1 pull-up, 2 pull-down
#endif

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
#if SPP_HAS_VR
  doc["vrSpeed"] = vrSpeed;
  doc["vehicleSpeedVR"] = vehicleSpeedVR;
#endif
  doc["ecuSpeed"] = ecuSpeed;
  doc["absSpeed"] = absSpeed;
  doc["dsgSpeed"] = dsgSpeed;
  doc["gpsSpeed"] = gpsSpeed;
  doc["udsSpeed"] = udsSpeed;
  doc["tp20Speed"] = tp20Speed;

  // Active source identifiers so the Diagnostics live-data can highlight which
  // input is currently driving the Speed / RPM outputs.
  doc["activeSpeedSource"] = activeSpeedSourceName();
  doc["activeRpmSource"] = useRPMCAN ? "CAN" : "Hall";

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

#if SPP_HAS_VOLTAGE_CTRL
  // Buck voltage control + drive calibration (live, S3 board)
  addVoltageStatus(doc);
#else
  doc["boardHasVoltageControl"] = false;
#endif
  addCalText(doc);

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
  // An imported file can span several TCP chunks: collect the whole body before
  // parsing. The buffer rides on _tempObject, which the server frees with the request.
  const size_t CAL_BODY_MAX = 16384;
  if (total > CAL_BODY_MAX)
  {
    if (index == 0) request->send(413, "application/json", "{\"error\":\"File too large\"}");
    return;
  }
  if (index == 0)
  {
    request->_tempObject = malloc(total);
  }
  uint8_t *body = (uint8_t *)request->_tempObject;
  if (!body)
  {
    if (index + len == total) request->send(500, "application/json", "{\"error\":\"Out of memory\"}");
    return;
  }
  memcpy(body + index, data, len);
  if (index + len != total) return; // wait for complete payload

  JsonDocument doc;
  if (deserializeJson(doc, body, total))
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
    out["json"]   = json;     // PWM calibration (legacy)
    out["carray"] = carray;   // PWM calibration as a firmware preset
#if SPP_HAS_VOLTAGE_CTRL
    String voltage;
    driveCalExportJson(voltage);
    out["voltage"] = voltage;          // voltage (drive) calibration
    out["voltageMarks"] = driveCalCount;
#endif
    String response;
    serializeJson(out, response);
    request->send(200, "application/json", response);
    return;
  }
  else if (strcmp(op, "import") == 0)
  {
    const char *json = doc["json"];
    // One Import button for both kinds: the file's "type" says which it is.
    if (json && strstr(json, DRIVE_CAL_FILE_TYPE))
    {
#if SPP_HAS_VOLTAGE_CTRL
      const char *err = nullptr;
      if (!driveCalImportJson(json, err))
      {
        JsonDocument e;
        e["error"] = err;
        String errBody;
        serializeJson(e, errBody);
        request->send(400, "application/json", errBody);
        return;
      }
      JsonDocument out;
      out["kind"] = "voltage";
      addDriveCal(out);
      String outBody;
      serializeJson(out, outBody);
      request->send(200, "application/json", outBody);
#else
      request->send(409, "application/json",
                    "{\"error\":\"This is a voltage calibration: this board has no voltage control\"}");
#endif
      return;
    }
    if (!json || !calImportJson(json))
    {
      request->send(400, "application/json", "{\"error\":\"Import failed: not a SpeedPulser calibration file\"}");
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

  for (uint8_t i = 1; i <= 6; i++)
  {
    if (key == "dsgRatio" + String(i))
    {
      float v = value.toFloat();
      if (v > 0.0f) dsgGearRatio[i] = v;
    }
  }

  if (key == "dsgFinal14")
  {
    float v = value.toFloat();
    if (v > 0.0f) dsgFinalDrive14 = v;
  }

  if (key == "dsgFinal56")
  {
    float v = value.toFloat();
    if (v > 0.0f) dsgFinalDrive56 = v;
  }

  if (key == "dsgTireCirc")
  {
    float v = value.toFloat();
    if (v > 0.0f) dsgTireCirc = v;
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
#if SPP_HAS_VOLTAGE_CTRL
    const bool wasCal = testCal;
    if (!boardTestActive) // the Test Board owns the motor while it runs
      testCal = (value == "true" || value == "1");
    // S3 calibration always starts with the motor stopped (drive 0): the user
    // raises the drive until the needle sits on a mark.
    if (testCal && !wasCal)
    {
      tempDriveV = 0.0f;
      resetDriveControl();
    }
    if (!testCal && wasCal && voltageControlEnable)
    {
      applyMotorDrive(0.0f);
    }
#else
    testCal = (value == "true" || value == "1");
#endif
  }

  if (key == "maxSpeed")
  {
    maxSpeed = value.toInt();
#if SPP_HAS_VOLTAGE_CTRL
    syncFeedbackMaxFreq(); // the captured marks are in cluster units: keep Hz-at-maxSpeed in step
#endif
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
    // Speed source selection: switch every source off first, then turn on only the
    // one picked, so two sources can never be on at once (up to V5.02 "Custom CAN"
    // left TP2.0 switched on, and TP2.0 then won in the speed task)
    useHall = false;
#if SPP_HAS_VR
    useVR = false;
#endif
    useECU = false;
    useABS = false;
    useDSG = false;
    useGPS = false;
    useTP20 = false;
    useUDS = false;
    useAftermarket = false;

    if (value == "VR")
    {
#if SPP_HAS_VR
      useVR = true;
#else
      useHall = true; // no VR input on this board: fall back to the hall input
#endif
    }
    else if (value == "ECU")
    {
      useECU = true;
    }
    else if (value == "ABS")
    {
      useABS = true;
    }
    else if (value == "DSG")
    {
      useDSG = true;
    }
    else if (value == "GPS")
    {
      useGPS = true;
    }
    else if (value == "TP2.0")
    {
      useTP20 = true;
    }
    else if (value == "UDS")
    {
      useUDS = true;
    }
    else if (value == "Custom CAN")
    {
      useAftermarket = true;
    }
    else
    {
      useHall = true; // "Hall" (or anything unrecognised): the hall input is the default
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

#if SPP_HAS_VR
  // ---- VR (variable-reluctance) speed input ----
  if (key == "maxFreqVR")
  {
    maxFreqVR = value.toInt();
  }

  if (key == "averageFilterVR")
  {
    long requestedSamples = value.toInt();
    averageFilterVR = (uint8_t)constrain(requestedSamples, 1, 10);
    samplesVR.clear();
  }
#endif

#if SPP_HAS_SPEED_BIAS
  // ---- Speed-input bias (S3 board): 0 none / 1 pull-up / 2 pull-down ----
  if (key == "speedInputBias")
  {
    speedInputBias = (uint8_t)constrain(value.toInt(), (long)SPEED_BIAS_NONE, (long)SPEED_BIAS_PULL_DOWN);
    applySpeedInputBias();
  }
#endif

#if SPP_HAS_VOLTAGE_CTRL
  // ---- Buck voltage control (S3 board) ----
  if (key == "voltageControlEnable")
  {
    voltageControlEnable = (value == "true" || value == "1");
    resetDriveControl();
    // Disabling voltage control reverts to legacy PWM behaviour on the fixed ~9 V
    // the duty presets were captured on; the buck stays enabled so the rail is present.
    if (!voltageControlEnable)
    {
      setMotorRailVolts(RAIL_V_LEGACY);
    }
  }

  if (key == "tempDriveV")
  {
    // Calibration drive (V) from the UI's -/+ buttons and slider; applied by the
    // speed task on its next 100 ms tick.
    tempDriveV = constrain(value.toFloat(), 0.0f, driveMaxV());
  }

  if (key == "vcPwmNominal")
  {
    vcPwmNominal = constrain(value.toFloat(), 0.2f, 1.0f);
  }

  if (key == "vcRailMaxV")
  {
    vcRailMaxV = constrain(value.toFloat(), RAIL_V_FLOOR, RAIL_V_CEIL);
  }

  if (key == "vcTrimGain")
  {
    vcTrimGain = constrain(value.toFloat(), 0.0f, 5.0f);
  }

  if (key == "vcTrimLimit")
  {
    vcTrimLimit = constrain(value.toFloat(), 0.0f, 0.5f);
    driveTrim = constrain(driveTrim, -vcTrimLimit, vcTrimLimit);
  }
#endif // SPP_HAS_VOLTAGE_CTRL

  request->send(200);
}

#if SPP_HAS_VOLTAGE_CTRL
// Error reply in the same {"error":"..."} shape the UI shows as a notification.
static void sendActionError(AsyncWebServerRequest *request, int code, const char *msg)
{
  JsonDocument out;
  out["error"] = msg;
  String response;
  serializeJson(out, response);
  request->send(code, "application/json", response);
}

/**
 * GET /api/boardtest - S3 board: BOARD_ID strap (read at boot) and the last
 * Test Board result: per-step target/measured rail and the fitted duty->volts line.
 */
void handleGetBoardTest(AsyncWebServerRequest *request)
{
  JsonDocument doc;
  doc["boardHasVoltageControl"] = true;
  doc["boardIdStrapOk"] = boardIdStrapOk;
  doc["running"] = (bool)boardTestActive;
  doc["code"] = boardTestCode;
  doc["text"] = boardTestText(boardTestCode);
  doc["pass"] = (boardTestCode == BT_PASS);
  doc["railSenseOk"] = railFitValid;
  doc["fitV0"] = railFitV0;
  doc["fitSlope"] = railFitSlope;
  doc["motorRailV"] = motorRailV;
  JsonArray pts = doc["points"].to<JsonArray>();
  for (uint8_t i = 0; i < BOARD_TEST_POINTS; i++)
  {
    JsonObject p = pts.add<JsonObject>();
    p["target"] = boardTestTargetV[i];
    p["measured"] = boardTestMeasV[i];
  }
  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}
#endif

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

#if SPP_HAS_VOLTAGE_CTRL
  if (action == "boardTest")
  {
    if (testCal || testSpeedo || tempNeedleSweep)
    {
      sendActionError(request, 409, "Turn off Speed Test / Calibration Mode first");
      return;
    }
    if (!startBoardTest())
    {
      sendActionError(request, 409, "Board test already running");
      return;
    }
  }

  if (action == "calAddPoint" || action == "calDeletePoint" || action == "calClearPoints")
  {
    // Drive calibration marks. Each answers with the updated table.
    if (action == "calAddPoint")
    {
      if (!testCal)
      {
        sendActionError(request, 409, "Enable Calibration Mode first");
        return;
      }
      const uint8_t r = driveCalCapture(doc["speed"] | 0);
      if (r != DCAL_OK)
      {
        sendActionError(request, 409, driveCalErrorText(r));
        return;
      }
    }
    else if (action == "calDeletePoint")
    {
      driveCalDelete(doc["index"] | 255);
    }
    else
    {
      driveCalClear();
    }
    JsonDocument out;
    out["status"] = "ok";
    addDriveCal(out);
    String body;
    serializeJson(out, body);
    request->send(200, "application/json", body);
    return;
  }
#endif

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

  // The web UI filesystem was mounted (guarded) by wifiManagerInit(); if it is
  // missing or broken, wifiManagerAttachStatic() serves a recovery page at "/".

  // Shared OTA + Home WiFi routes FIRST: ota_manager's first route carries the
  // filter that notes web activity for every request (otaWebClientActive()),
  // and /api/wifi/sta must precede any /api/wifi... route of our own.
  otaManagerAttach(server);
  wifiManagerAttachSta(server);

  // API routes for getting data
  server.on("/api/settings", HTTP_GET, handleGetSettings);
  server.on("/api/status", HTTP_GET, handleGetStatus);
  server.on("/api/calibrations", HTTP_GET, handleGetCalibrations);
  server.on("/api/calcurve", HTTP_GET, handleGetCalCurve);
  server.on("/api/cal", HTTP_GET, handleGetCal);
#if SPP_HAS_VOLTAGE_CTRL
  server.on("/api/boardtest", HTTP_GET, handleGetBoardTest); // Test Board result (start it via /api/action)
#endif

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

  // "/" (the UI, or the recovery page when the filesystem holds no usable UI)
  // + static files with no-cache revalidation.
  wifiManagerAttachStatic(server);

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
  DEBUG_WIFI("Beginning WiFi / creating Access Point...");

  // Common SoftAP + mDNS + LittleFS front-end (reachable at speedpulserpro.local).
  wifimgr_config_t wcfg = wifiDefaultConfig();
  wcfg.hostName  = wifiHostName;
  wcfg.mdnsName  = "speedpulserpro"; // -> http://speedpulserpro.local
  wcfg.fwVersion = FW_VERSION;       // recovery page only; index.html bakes its own
  // MUST precede wifiManagerInit(): that mounts the web-UI filesystem via
  // otaFsMountSafe(), so ota_manager has to be configured first or a failed
  // mount passes silently.
  ota_config_t ocfg = otaDefaultConfig();
  ocfg.fwVersion  = FW_VERSION;
  ocfg.product    = "SpeedPulser Pro";
  ocfg.githubRepo = "adamforbes92/speedPulserPro"; // Releases/ + releases.json for "Check for updates"
  ocfg.boardId    = SPP_BOARD_ID;   // picks this board's images out of releases.json ("boards")
  ocfg.boardName  = SPP_BOARD_NAME; // shown on the OTA tab
  otaManagerInit(&ocfg);

  wifiManagerInit(&wcfg);

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
  // ... or a browser has hit us in the last 30 s (a phone on the home router
  // in bridge mode is not an AP station).
  return WiFi.softAPgetStationNum() > 0 || otaInProgress() || otaWebClientActive();
}

// ACTIVE -> REDUCED: close the web server cleanly before the radio drops.
void powerOnEnterReduced()
{
  server.end();
  wifiManagerStopAP();
}

// REDUCED -> ACTIVE: bring the AP and web server back. Routes are already
// registered (no need to re-run setupUI()), so we only restart the radio
// and the listener.
void powerOnExitReduced()
{
  wifiManagerStartAP();
  server.begin();
}
