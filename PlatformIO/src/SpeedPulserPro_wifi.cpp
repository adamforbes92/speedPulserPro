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
#include "SpeedPulserPro_io.h"
#include "SpeedPulserPro_resetLog.h"

// A POST with no body never reaches the body handler, so the request answers it
// (V5.09, as SpeedPulser: it used to hang until the browser timed out).
static void answerEmptyPost(AsyncWebServerRequest *request)
{
  if (request->contentLength() == 0)
    request->send(400, "application/json", "{\"error\":\"Empty request\"}");
}

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
  if (driveCalCount == 0)
    doc["calibrationText"] = "Not calibrated";
  else
    doc["calibrationText"] = String(driveCalCount) + (driveCalCount == 1 ? " mark, " : " marks, ") +
                             String(driveCal[0].speed) + "-" + String(driveCalTopSpeed());
  return;
#endif
  doc["calibrationText"] = getCalibrationText(presetId);
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
    p["src"] = driveCal[i].src == DCAL_SRC_AUTO ? "auto" : "eye";
  }
  doc["driveCalHz"] = driveCalHzValid();
  doc["calBackup"] = driveCalBackupValid; // Restore Previous available
}

// Live S3-board read-outs for /api/status: rail, drive and tacho keep updating
// in every mode (Calibration, Speed Test and normal running).
static void addVoltageStatus(JsonDocument &doc)
{
  doc["pwmFrac"] = lastPwmFrac;         // 0..1 motor PWM
  doc["driveV"] = lastDriveV;           // applied drive (V = rail x PWM)
  doc["driveMaxV"] = driveMaxV();
  doc["calibrationDriveV"] = calibrationDriveV;       // calibration drive the user set
  doc["driveTrimPct"] = driveTrim * 100.0f;
  // The trim has run out of room (V5.09): the calibration is off at this speed.
  doc["trimAtLimit"] = trimLimit > 0.0f && fabsf(driveTrim) >= trimLimit - 0.005f;
  doc["railCmdV"] = lastRailCmdV;       // commanded rail (V)
  doc["motorRailV"] = motorRailV;       // measured rail (V) — trust it once railSenseOk
  doc["railSenseOk"] = railFitValid;
  doc["railLimited"] = railLimited();
  if (railTripped)
    doc["railTrippedV"] = railTripV; // motor supply shut off: reboot to clear
  doc["boardTestRunning"] = (bool)boardTestActive;
  float fbMeanHz, fbSpreadPct, fbDriftHz;
  const bool fbWin = feedbackHzWindow(fbMeanHz, fbSpreadPct, fbDriftHz);
  doc["fbHzSpread"] = fbWin ? fbSpreadPct : -1.0f;   // % (max-min)/mean; -1 = not turning
  doc["fbHzDrift"] = fbWin ? fbDriftHz : 0.0f;       // Hz the mean moved across the window
  doc["fbStable"] = fbWin && feedbackHzSteady(fbMeanHz, fbSpreadPct, fbDriftHz);
  // Auto Fill progress (V5.05)
  JsonObject af = doc["autoFill"].to<JsonObject>();
  af["state"] = autoFillState; // 0 idle, 1 running, 2 done, 3 failed
  af["index"] = autoFillIndex;
  af["total"] = autoFillTotal;
  af["speed"] = autoFillSpeed;
  af["hz"] = autoFillTargetHz;
  af["msg"] = autoFillMsg;
  doc["calBackup"] = driveCalBackupValid;
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

// ===== POST bodies (V5.06, mirrors SpeedPulser V5.06) =====
// Collect a POST body that may arrive in several TCP chunks; the whole body once the
// last chunk is in, nullptr until then (or after an error reply). POSTs must be
// application/json: a page on another site can only send that after a preflight this
// server never answers, so it can't drive the motor or the tach.
static uint8_t *collectJsonBody(AsyncWebServerRequest *request, uint8_t *data, size_t len,
                                size_t index, size_t total, size_t maxBytes)
{
  if (!request->contentType().startsWith("application/json"))
  {
    if (index == 0)
      request->send(415, "application/json", "{\"error\":\"JSON only\"}");
    return nullptr;
  }
  if (total > maxBytes)
  {
    if (index == 0)
      request->send(413, "application/json", "{\"error\":\"Request too large\"}");
    return nullptr;
  }
  if (index == 0)
    request->_tempObject = malloc(total);
  uint8_t *body = (uint8_t *)request->_tempObject;
  if (!body)
  {
    if (index + len == total)
      request->send(500, "application/json", "{\"error\":\"Out of memory\"}");
    return nullptr;
  }
  memcpy(body + index, data, len);
  return (index + len == total) ? body : nullptr;
}

// The needle is at rest and nothing is driving it (direction changes, the needle
// sweep and OTA updates are only allowed then).
static bool motorIsStopped()
{
  // inputSpeedKph: the source's own speed, also in Calibration Mode (V5.09, as SpeedPulser)
  return motorDutyApplied == 0 && needleTargetSpeed == 0 && !speedTestOn && inputSpeedKph <= 0.0f
#if SPP_HAS_VOLTAGE_CTRL
         && lastDriveV <= 0.0f
#endif
      ;
}

// ota_manager hooks: an update restarts the unit, so only with the car stopped.
bool otaIsSafe(String &reason)
{
  if (!motorIsStopped())
  {
    reason = "Stop the car first: the speedo is running";
    return false;
  }
  if (calibrationModeOn || speedTestOn || testRPM || needleSweepRequested
#if SPP_HAS_VOLTAGE_CTRL
      || boardTestActive || autoFillRunning()
#endif
  )
  {
    reason = "Turn off Speed Test / RPM Test / Calibration Mode first";
    return false;
  }
  return true;
}

void otaOnStart(bool filesystem)
{
  (void)filesystem;
  speedTestOn = false; // nothing may drive the motor while the flash is rewritten
  testRPM = false;
  calibrationModeOn = false;
  setMotorDutyRaw(0);
#if SPP_HAS_VOLTAGE_CTRL
  applyMotorDrive(0.0f);
#endif
  setFrequencyRPM(0);
}

// Every setting, for GET /api/settings and as the reply to each /api/control change
// (so the page always shows the values the unit kept after range checks).
static void fillSettings(JsonDocument &doc)
{
  doc["needleSweepOnBoot"] = needleSweepOnBoot;
  doc["coilType"] = coilType;
  doc["clusterInMph"] = clusterInMph;
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
  doc["needleSweepSpeed"] = needleSweepSpeed;
  doc["stepRPM"] = stepRPM;
  doc["stepSpeed"] = stepSpeed;
  doc["speedTestOn"] = speedTestOn;
  doc["speedTestKph"] = speedTestKph;
  doc["rpmTestOn"] = testRPM;
  doc["rpmTestRpm"] = tempRPM;
  doc["calibrationModeOn"] = calibrationModeOn;
  doc["calibrationProbeDuty"] = calibrationProbeDuty;
  doc["maxRPM"] = maxRPM;
  doc["clusterRPMLimit"] = clusterRPMLimit;
  doc["presetId"] = presetId;
  doc["maxSpeed"] = maxSpeed;
  doc["sensorPulsesPerKm"] = hallPulsesPerKm;
  doc["inputSmoothingMs"] = inputSmoothingMs;
  doc["globalSpeedOffsetKph"] = globalSpeedOffsetKph;
  doc["useSpeedOffsetCurve"] = useSpeedOffsetCurve;
  JsonArray speedCurveOffsets = doc["speedOffsetCurveOffsets"].to<JsonArray>();
  for (uint8_t i = 0; i < SPEED_OFFSET_CURVE_POINTS; i++)
  {
    speedCurveOffsets.add(speedOffsetCurveOffsets[i]);
  }
  doc["averageFilterRPM"] = averageFilterRPM;
  doc["gpsUpdateRateHz"] = gpsUpdateRateHz;

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

  // S3 drive trim (the DevKit has no motor feedback)
  doc["feedbackEnable"] = feedbackEnable;
  doc["feedbackDeadband"] = feedbackDeadband;
  doc["feedbackMinSpeed"] = feedbackMinSpeed;

  // Board identity + optional hardware blocks. The UI hides every control whose
  // hardware isn't on this board (voltage control, rail sense, VR, speed bias).
  doc["boardName"] = SPP_BOARD_NAME;
  doc["isVoltageBoard"] = (bool)SPP_HAS_VOLTAGE_CTRL;
  doc["hasVR"] = (bool)SPP_HAS_VR;
  doc["hasSpeedBias"] = (bool)SPP_HAS_SPEED_BIAS;

#if SPP_HAS_VOLTAGE_CTRL
  // Buck voltage control (S3 board)
  doc["railFloorV"] = RAIL_V_FLOOR;   // hardware rail window (V)
  doc["railCeilV"] = RAIL_V_CEIL;
  doc["boardTestCode"] = boardTestCode;
  doc["pwmNominal"] = pwmNominal;
  doc["railMaxV"] = railMaxV;
  doc["railMinV"] = railMinV;
  doc["trimGain"] = trimGain;
  doc["trimLimit"] = trimLimit;
  doc["driveMaxV"] = driveMaxV();
  doc["calibrationDriveV"] = calibrationDriveV; // the calibration drive kept (each change is capped at +0.5 V)
  addDriveCal(doc);
#endif
  addCalText(doc);

#if SPP_HAS_VR
  // VR (variable-reluctance) speed input
  doc["vrPulsesPerKm"] = vrPulsesPerKm;
#endif
#if SPP_HAS_SPEED_BIAS
  doc["speedInputBias"] = speedInputBias; // 0 none, 1 pull-up, 2 pull-down
#endif

  doc["fwVersion"] = FW_VERSION;
}

void handleGetSettings(AsyncWebServerRequest *request)
{
  JsonDocument doc;
  fillSettings(doc);
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
  doc["inputSpeedKph"] = inputSpeedKph;   // the selected source, in every mode (V5.09)
  doc["vehicleSpeed"] = vehicleSpeed;     // what drives the needle (the test speed in Speed Test)
  doc["hallSpeed"] = hallSpeed;
#if SPP_HAS_VR
  doc["vrSpeed"] = vrSpeed;
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
  doc["speedTestOn"] = speedTestOn;
  doc["speedTestKph"] = speedTestKph;
  doc["rpmTestOn"] = testRPM;
  doc["rpmTestRpm"] = tempRPM;
  doc["calibrationModeOn"] = calibrationModeOn;
  doc["calibrationProbeDuty"] = calibrationProbeDuty;
  doc["motorDutyApplied"] = motorDutyApplied;
  doc["needleTargetSpeed"] = needleTargetSpeed;
  doc["hallFrequencyHz"] = hallFrequencyHz;
#if SPP_HAS_VR
  doc["vrFrequencyHz"] = vrFrequencyHz;
#endif
  doc["speedOffsetApplied"] = speedOffsetApplied;

  // System status
  doc["hasCAN"] = hasCAN;
  doc["hasGPS"] = hasGPS;
  doc["broadcastSpeedEnabled"] = broadcastSpeedEnabled;
  doc["broadcastSpeedValue"] = broadcastSpeedValue;
  doc["aftermarketSpeed"] = aftermarketSpeed;
  doc["gpsUnavailable"] = gpsUnavailable;
  doc["gpsSatellites"] = gps.satellites.value();
  // GPS update frequency
  doc["gpsFrequency"] = getGPSUpdateFrequency();
  doc["gpsAutoApplySecs"] = gpsAutoApplySecondsRemaining();

  // Motor feedback (S3 drive trim) live status
  doc["measuredSpeed"] = measuredSpeed;
  doc["measuredFreqHz"] = measuredFreqHz;
  doc["feedbackAvailable"] = feedbackAvailable;
  doc["feedbackMissing"] = feedbackMissing;

#if SPP_HAS_VOLTAGE_CTRL
  // Buck voltage control + drive calibration (live, S3 board)
  addVoltageStatus(doc);
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

  doc["selected"] = presetId;

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
  doc["calibrationText"] = getCalibrationText(presetId);
  doc["custom"]          = (presetId == CUSTOM_CAL_ID && customCalValid);

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
  if (presetId == CUSTOM_CAL_ID && customCalValid)
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
  doc["clusterInMph"] = clusterInMph;
  doc["count"]    = customCalCount;
  doc["selected"] = (presetId == CUSTOM_CAL_ID);
  doc["duty"]     = (uint16_t)calibrationProbeDuty;

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
  if (!request->contentType().startsWith("application/json"))
  {
    request->send(415, "application/json", "{\"error\":\"JSON only\"}");
    return;
  }

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

  // V5.08 (mirrors SpeedPulser): jogging only drives the motor in Calibration Mode,
  // so a point captured outside it stores a duty the needle never showed; and with
  // the custom cal selected any point change moves the running needle.
  const bool editOp = strcmp(op, "jog") == 0 || strcmp(op, "addPoint") == 0 ||
                      strcmp(op, "deletePoint") == 0 || strcmp(op, "clearPoints") == 0 ||
                      strcmp(op, "apply") == 0 || strcmp(op, "save") == 0;
#if SPP_HAS_VOLTAGE_CTRL
  // The Calibration Builder is the DevKit's: on the S3 it would select a PWM table
  // that never drives the motor (V5.09: only jog / addPoint were refused).
  if (editOp)
  {
    request->send(409, "application/json", "{\"error\":\"Not on the S3 board: use Drive Calibration\"}");
    return;
  }
#endif
  if (editOp && !calibrationModeOn)
  {
    request->send(409, "application/json", "{\"error\":\"Tick Calibration Mode first\"}");
    return;
  }
  if (strcmp(op, "jog") == 0)
  {
    // DevKit: past max wraps to 0, below 0 wraps to max, so the first calibration
    // step (pot at minimum, duty at maximum) is one press away.
    int32_t delta = doc["delta"] | 0;
    int32_t range = (int32_t)PWM_DUTY_MAX + 1;
    int32_t next = ((int32_t)calibrationProbeDuty + delta) % range;
    if (next < 0) next += range;
    calibrationProbeDuty = next;
  }
  else if (strcmp(op, "addPoint") == 0)
  {
    const long speedIn = doc["speed"] | 0L; // 65537 must not wrap round to 1
    const uint16_t speed = speedIn >= 1 && speedIn <= 400 ? (uint16_t)speedIn : 0;
    const uint16_t duty = (uint16_t)calibrationProbeDuty; // the duty the needle is showing now
    const char *err = nullptr;
    if (!calAddPoint(speed, duty, err))
    {
      JsonDocument e;
      e["error"] = err;
      String errBody;
      serializeJson(e, errBody);
      request->send(409, "application/json", errBody);
      return;
    }
  }
  else if (strcmp(op, "deletePoint") == 0)
  {
    if (doc["index"].isNull())
    {
      request->send(400, "application/json", "{\"error\":\"Missing index\"}");
      return;
    }
    calDeletePoint(doc["index"].as<uint8_t>());
  }
  else if (strcmp(op, "clearPoints") == 0)
  {
    calClearPoints();
  }
  else if (strcmp(op, "setName") == 0)
  {
    const char *name = doc["name"];
    if (name) calSetName(name);
    customCalUnitMph = clusterInMph;
  }
  else if (strcmp(op, "apply") == 0 || strcmp(op, "save") == 0)
  {
    // Both save the points and record the dial unit (V5.08, mirrors SpeedPulser):
    // Apply used to select the custom cal without saving the points (dead speedo at
    // the next ignition-on) and without the unit (the dial flipped back to mph).
    if (customCalCount < 2)
    {
      request->send(409, "application/json", "{\"error\":\"Capture at least 2 points first\"}");
      return;
    }
    customCalUnitMph = clusterInMph;
    calSaveToNvs();
    presetId = CUSTOM_CAL_ID; // persisted by the periodic writeEEP
    updateMotorPerformance = true;
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
      if (!calibrationModeOn)
      {
        request->send(409, "application/json", "{\"error\":\"Enable Calibration Mode first (Calibration page)\"}");
        return;
      }
      if (autoFillRunning())
      {
        request->send(409, "application/json", "{\"error\":\"Auto Fill is running: stop it first\"}");
        return;
      }
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
#if SPP_HAS_VOLTAGE_CTRL
    request->send(409, "application/json",
                  "{\"error\":\"This is a PWM calibration (DevKit): the S3 uses drive marks\"}");
    return;
#endif
    const char *importErr = nullptr;
    if (!json || !calImportJson(json, importErr))
    {
      JsonDocument e;
      e["error"] = importErr ? importErr : "Import failed: not a SpeedPulser calibration file";
      String errBody;
      serializeJson(e, errBody);
      request->send(400, "application/json", errBody);
      return;
    }
  }
  else
  {
    request->send(400, "application/json", "{\"error\":\"Unknown op\"}");
    return;
  }

  DEBUG_WIFI("cal op: %s (count=%u duty=%u)", op, (unsigned)customCalCount, (unsigned)calibrationProbeDuty);

  JsonDocument state;
  fillCalState(state);
  String response;
  serializeJson(state, response);
  request->send(200, "application/json", response);
}

void handlePostControl(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  uint8_t *body = collectJsonBody(request, data, len, index, total, 512);
  if (!body) return;
  JsonDocument doc;
  if (deserializeJson(doc, body, total))
  {
    request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  String key = doc["key"];
  String value = doc["value"].as<String>();

  DEBUG_WIFI("POST /api/control - %s = %s", key.c_str(), value.c_str());

  // Parse and handle setting changes
  if (key == "needleSweepOnBoot")
  {
    needleSweepOnBoot = (value == "true" || value == "1");
  }

  if (key == "coilType")
  {
    coilType = (value == "true" || value == "1");
  }

  if (key == "clusterInMph")
  {
    // A custom PWM cal's speeds are dial readings in the unit it was captured in.
    const bool mph = (value == "true" || value == "1");
    if (!SPP_HAS_VOLTAGE_CTRL && presetId == CUSTOM_CAL_ID && customCalValid && mph != customCalUnitMph)
    {
      request->send(409, "application/json", customCalUnitMph
          ? "{\"error\":\"The custom calibration was captured in mph: pick a preset or capture it again in km/h\"}"
          : "{\"error\":\"The custom calibration was captured in km/h: pick a preset or capture it again in mph\"}");
      return;
    }
    clusterInMph = mph;
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
    broadcastSpeedScale = constrain(value.toFloat(), -CAN_SCALE_MAX, CAN_SCALE_MAX); // V5.09: a typo can't overflow the speed
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
    aftermarketSpeedScale = constrain(value.toFloat(), -CAN_SCALE_MAX, CAN_SCALE_MAX);
  }

  if (key == "aftermarketSpeedOffset")
  {
    aftermarketSpeedOffset = (int16_t)constrain(value.toInt(), -32768, 32767);
  }

  for (uint8_t i = 1; i <= 6; i++)
  {
    if (key == "dsgRatio" + String(i))
    {
      dsgGearRatio[i] = constrain(value.toFloat(), DSG_RATIO_MIN, DSG_RATIO_MAX); // V5.09: was only > 0
    }
  }

  if (key == "dsgFinal14")
  {
    dsgFinalDrive14 = constrain(value.toFloat(), DSG_RATIO_MIN, DSG_RATIO_MAX);
  }

  if (key == "dsgFinal56")
  {
    dsgFinalDrive56 = constrain(value.toFloat(), DSG_RATIO_MIN, DSG_RATIO_MAX);
  }

  if (key == "dsgTireCirc")
  {
    dsgTireCirc = constrain(value.toFloat(), DSG_TIRE_CIRC_MIN, DSG_TIRE_CIRC_MAX);
  }

  // Every value is range-checked (V5.06): a cached old page or a hand-made request
  // can't stall the chip (tach timer) or over-drive the needle.
  if (key == "needleSweepSpeed")
  {
    needleSweepSpeed = (uint8_t)constrain(value.toInt(), 1L, 50L);
  }

  if (key == "stepRPM")
  {
    stepRPM = constrain(value.toInt(), 1L, 50L); // the page's slider range (V5.09: was 1-100)
  }

  if (key == "stepSpeed")
  {
    stepSpeed = constrain(value.toInt(), 1L, 50L);
  }

#if SPP_HAS_VOLTAGE_CTRL
  if ((key == "speedTestOn" || key == "calibrationModeOn" || key == "rpmTestOn") && boardTestActive)
  {
    // The Test Board owns the motor while it runs (V5.09, as SpeedPulser: Speed
    // Test used to be accepted and Calibration Mode silently ignored).
    request->send(409, "application/json", "{\"error\":\"Board test running\"}");
    return;
  }
#endif

  if (key == "speedTestOn")
  {
    speedTestOn = (value == "true" || value == "1");
    if (speedTestOn && calibrationModeOn)
    {
      // One test mode at a time: Speed Test switches Calibration Mode off.
#if SPP_HAS_VOLTAGE_CTRL
      autoFillStop("Stopped: Speed Test was switched on");
#endif
      calibrationModeOn = false;
    }
    if (!speedTestOn)
      speedTestKph = 0;
  }

  if (key == "speedTestKph")
  {
    speedTestKph = constrain(value.toInt(), 0L, (long)maxSpeed);
  }

  if (key == "rpmTestOn")
  {
    testRPM = (value == "true" || value == "1");
  }

  if (key == "calibrationModeOn")
  {
    const bool wasCal = calibrationModeOn;
    calibrationModeOn = (value == "true" || value == "1");
#if SPP_HAS_VOLTAGE_CTRL
    if (!calibrationModeOn)
      autoFillStop("Stopped: Calibration Mode was turned off");
#endif
    if (calibrationModeOn)
    {
      speedTestOn = false; // one test mode at a time
      speedTestKph = 0;
      if (!wasCal)
      {
        // Calibration always starts with the motor stopped: the user raises it.
        calibrationProbeDuty = 0;
#if SPP_HAS_VOLTAGE_CTRL
        calibrationDriveV = 0.0f;
        resetDriveControl();
#endif
      }
    }
#if SPP_HAS_VOLTAGE_CTRL
    if (!calibrationModeOn && wasCal)
      applyMotorDrive(0.0f);
#endif
  }

  if (key == "maxSpeed")
  {
    maxSpeed = (uint16_t)constrain(value.toInt(), 20L, 400L);
    updateInputGlitchFilters();
  }

  if (key == "sensorPulsesPerKm")
  {
    hallPulsesPerKm = constrain(value.toFloat(), SENSOR_PULSES_PER_KM_MIN, SENSOR_PULSES_PER_KM_MAX);
    updateInputGlitchFilters();
  }

  if (key == "inputSmoothingMs")
  {
    inputSmoothingMs = (uint16_t)constrain(value.toInt(), (long)INPUT_SMOOTHING_MS_MIN, (long)INPUT_SMOOTHING_MS_MAX);
  }

  if (key == "globalSpeedOffsetKph")
  {
    globalSpeedOffsetKph = (int16_t)constrain(value.toInt(), -50L, 50L);
  }

  if (key == "useSpeedOffsetCurve")
  {
    useSpeedOffsetCurve = (value == "true" || value == "1");
  }

  if (key == "curveOffset0")
  {
    speedOffsetCurveOffsets[0] = (int16_t)constrain(value.toInt(), -20L, 20L);
  }

  if (key == "curveOffset1")
  {
    speedOffsetCurveOffsets[1] = (int16_t)constrain(value.toInt(), -20L, 20L);
  }

  if (key == "curveOffset2")
  {
    speedOffsetCurveOffsets[2] = (int16_t)constrain(value.toInt(), -20L, 20L);
  }

  if (key == "curveOffset3")
  {
    speedOffsetCurveOffsets[3] = (int16_t)constrain(value.toInt(), -20L, 20L);
  }

  if (key == "curveOffset4")
  {
    speedOffsetCurveOffsets[4] = (int16_t)constrain(value.toInt(), -20L, 20L);
  }


  if (key == "averageFilterRPM")
  {
    long requestedSamples = value.toInt();
    averageFilterRPM = (uint8_t)constrain(requestedSamples, 1, 10);
    resetRPMMedianFilter();
  }

  if (key == "rpmTestRpm")
  {
    tempRPM = constrain(value.toInt(), 0L, 12000L); // RPM Test (the tach is scaled to clusterRPMLimit)
  }

  if (key == "maxRPM")
  {
    // Tach full scale (Hz). The tach timer interrupts twice per period, so this is
    // capped: 60000 here (from a hand-made request) stalled the chip.
    maxRPM = (uint16_t)constrain(value.toInt(), 10L, 1000L);
  }

  if (key == "clusterRPMLimit")
  {
    clusterRPMLimit = (uint16_t)constrain(value.toInt(), 1000L, 12000L);
  }

  if (key == "presetId")
  {
    const long id = value.toInt();
    if ((id >= 1 && id <= getCalibrationCount()) || (id == CUSTOM_CAL_ID && customCalCount >= 2))
    {
      presetId = (uint8_t)id;
      if (presetId == CUSTOM_CAL_ID)
        calSaveToNvs(); // the periodic save stores presetId: the points must be saved too
      updateMotorPerformance = true; // the speed task swaps the table (not mid-tick from here)
    }
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
    if (!motorIsStopped())
    {
      request->send(409, "application/json", "{\"error\":\"Stop the motor first (no speed, Speed Test off, drive 0)\"}");
      return;
    }
    reverseDirection = (value == "true" || value == "1");
    applyDirection(); // apply the new direction immediately
  }

#if SPP_HAS_VOLTAGE_CTRL
  // ---- S3 drive trim (the DevKit has no motor feedback: V5.06 removed the PID) ----
  if (key == "feedbackEnable")
  {
    feedbackEnable = (value == "true" || value == "1");
    resetDriveControl();
  }

  if (key == "feedbackDeadband")
  {
    feedbackDeadband = constrain(value.toFloat(), 0.0f, 5.0f);
  }

  if (key == "feedbackMinSpeed")
  {
    feedbackMinSpeed = (uint16_t)constrain(value.toInt(), 0L, 100L);
  }
#endif

#if SPP_HAS_VR
  // ---- VR (variable-reluctance) speed input ----
  if (key == "vrPulsesPerKm")
  {
    vrPulsesPerKm = constrain(value.toFloat(), SENSOR_PULSES_PER_KM_MIN, SENSOR_PULSES_PER_KM_MAX);
    updateInputGlitchFilters();
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
  if (key == "calibrationDriveV")
  {
    // Calibration drive (V) from the UI's -/+ buttons; applied by the speed task on
    // its next 100 ms tick. Lowering is unrestricted, but one request may raise it
    // by at most CAL_DRIVE_MAX_STEP_V (the largest button step): a cached old page
    // or a second browser can't jump the motor voltage and over-drive the gauge
    // (V5.05, from SpeedPulser V5.03). Refused while Auto Fill owns the drive.
    if (!calibrationModeOn)
    {
      request->send(409, "application/json", "{\"error\":\"Enable Calibration Mode first\"}");
      return;
    }
    if (autoFillRunning())
    {
      request->send(409, "application/json", "{\"error\":\"Auto Fill is running\"}");
      return;
    }
    float want = constrain(value.toFloat(), 0.0f, driveMaxV());
    if (want > calibrationDriveV + CAL_DRIVE_MAX_STEP_V)
    {
      DEBUG_WIFI("tempDriveV %.2f -> %.2f refused, limited to +%.1f V", calibrationDriveV, want, CAL_DRIVE_MAX_STEP_V);
      want = calibrationDriveV + CAL_DRIVE_MAX_STEP_V;
    }
    calibrationDriveV = want;
  }

  if ((key == "railMinV" || key == "pwmNominal" || key == "railMaxV") && (!calibrationModeOn || autoFillRunning()))
  {
    // They change how every mark's drive splits into rail x PWM, so the needle moves:
    // Calibration Mode only (V5.08, mirrors SpeedPulser), not while Auto Fill measures.
    request->send(409, "application/json", autoFillRunning() ? "{\"error\":\"Auto Fill is running\"}"
                                                              : "{\"error\":\"Tick Calibration Mode first (Setup, step 4)\"}");
    return;
  }

  if (key == "railMinV")
  {
    railMinV = constrain(value.toFloat(), RAIL_V_FLOOR, railMaxV);
    resetDriveControl();
  }

  if (key == "pwmNominal")
  {
    pwmNominal = constrain(value.toFloat(), 0.2f, 1.0f);
  }

  if (key == "railMaxV")
  {
    railMaxV = constrain(value.toFloat(), RAIL_V_FLOOR, RAIL_V_CEIL);
    if (railMinV > railMaxV)
      railMinV = railMaxV; // Minimum Rail can never sit above Maximum Rail
  }

  if (key == "trimGain")
  {
    trimGain = constrain(value.toFloat(), 0.0f, 5.0f);
  }

  if (key == "trimLimit")
  {
    trimLimit = constrain(value.toFloat(), 0.0f, 0.5f);
    driveTrim = constrain(driveTrim, -trimLimit, trimLimit);
  }
#endif // SPP_HAS_VOLTAGE_CTRL

  // The reply (V5.09, as SpeedPulser): {"status":"ok","value":<the value kept>}, or
  // 400 for a key this board doesn't have. It used to be the whole settings object
  // (~100 keys plus every mark on each drive-button press) and "ok" even for a typo.
  JsonDocument all;
  fillSettings(all);
  JsonDocument reply;
  if (key.startsWith("curveOffset") && key.length() == 12 && key[11] >= '0' && key[11] < '0' + SPEED_OFFSET_CURVE_POINTS)
    reply["value"] = speedOffsetCurveOffsets[key[11] - '0'];
  else if (!all[key].isNull())
    reply["value"] = all[key];
  else
  {
    request->send(400, "application/json", "{\"error\":\"Unknown setting\"}");
    return;
  }
  reply["status"] = "ok";
#if SPP_HAS_VOLTAGE_CTRL
  if (key == "railMaxV")
    reply["railMinV"] = railMinV; // Minimum Rail follows a lower Maximum Rail
#endif
  String response;
  serializeJson(reply, response);
  request->send(200, "application/json", response);
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
  doc["running"] = (bool)boardTestActive;
  doc["code"] = boardTestCode;
  doc["text"] = boardTestText(boardTestCode);
  doc["pass"] = (boardTestCode == BT_PASS);
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
  uint8_t *body = collectJsonBody(request, data, len, index, total, 512);
  if (!body) return;
  JsonDocument doc;
  if (deserializeJson(doc, body, total))
  {
    request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  String action = doc["action"];

  if (action == "needleSweep")
  {
    // The sweep suspends the tasks for a few seconds: only with the car stopped.
    if (!motorIsStopped() || calibrationModeOn
#if SPP_HAS_VOLTAGE_CTRL
        || boardTestActive || autoFillRunning()
#endif
    )
    {
      request->send(409, "application/json", "{\"error\":\"Only with the car stopped and no test running\"}");
      return;
    }
    needleSweepRequested = true;
  }

#if SPP_HAS_VOLTAGE_CTRL
  if (action == "boardTest")
  {
    if (calibrationModeOn || speedTestOn || needleSweepRequested || !motorIsStopped())
    {
      sendActionError(request, 409, "Stop the motor and turn off Speed Test / Calibration Mode first");
      return;
    }
    if (railMaxV < BOARD_TEST_MIN_TOP_V)
    {
      sendActionError(request, 409, "Test Board needs Maximum rail of 6 V or more (Advanced, Motor Voltage)");
      return;
    }
    if (!startBoardTest())
    {
      sendActionError(request, 409, "Board test already running");
      return;
    }
  }

  if (action == "calAddPoint" || action == "calDeletePoint" || action == "calClearPoints" || action == "calSetSource")
  {
    // Drive calibration marks. Each answers with the updated table. Changing the
    // table while driving would jump the needle: Calibration Mode only.
    if (autoFillRunning())
    {
      sendActionError(request, 409, "Auto Fill is running: stop it first");
      return;
    }
    if (!calibrationModeOn)
    {
      sendActionError(request, 409, "Enable Calibration Mode first");
      return;
    }
    if (action == "calAddPoint")
    {
      const long speed = doc["speed"] | 0L; // 65537 must not wrap round to 1
      const uint8_t r = driveCalCapture(speed >= 1 && speed <= 400 ? (uint16_t)speed : 0);
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
    else if (action == "calSetSource")
    {
      // Tap a mark's eye / auto pill: Auto Fill keeps eye marks and replaces auto ones.
      if (!driveCalSetSource(doc["index"] | 255, (doc["auto"] | false) ? DCAL_SRC_AUTO : DCAL_SRC_EYE))
      {
        sendActionError(request, 400, "No such mark");
        return;
      }
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

  if (action == "calAutoFill" || action == "calAutoFillStop" || action == "calRestore")
  {
    // V5.05 Auto Fill: start / stop, and Restore Previous (swap with the backup).
    const char *err = "OK";
    bool ok = true;
    if (action == "calAutoFill")
    {
      ok = autoFillStart(err);
    }
    else if (action == "calAutoFillStop")
    {
      autoFillStop("Stopped");
    }
    else if (autoFillRunning())
    {
      ok = false;
      err = "Auto Fill is running: stop it first";
    }
    else if (!calibrationModeOn)
    {
      ok = false;
      err = "Enable Calibration Mode first";
    }
    else
    {
      ok = driveCalRestoreBackup(err);
    }
    JsonDocument out;
    if (ok)
      out["status"] = "ok";
    else
      out["error"] = err;
    addDriveCal(out);
    String body;
    serializeJson(out, body);
    request->send(ok ? 200 : 409, "application/json", body);
    return;
  }
#endif

  if (action == "clearResetLog")
  {
    resetLogClear();
  }
  else if (action != "needleSweep" && action != "boardTest")
  {
    request->send(400, "application/json", "{\"error\":\"Unknown action\"}"); // V5.09: was "ok"
    return;
  }
  request->send(200, "application/json", "{\"status\":\"ok\"}");
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
  server.on("/api/resetlog", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc; // start-up log (V5.09, as SpeedPulser)
    resetLogToJson(doc);
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });
#if SPP_HAS_VOLTAGE_CTRL
  server.on("/api/boardtest", HTTP_GET, handleGetBoardTest); // Test Board result (start it via /api/action)
#endif

  // API routes for receiving commands
  server.on("/api/control", HTTP_POST, answerEmptyPost, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostControl(request, data, len, index, total); });

  // Custom calibration builder operations
  server.on("/api/cal", HTTP_POST, answerEmptyPost, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostCal(request, data, len, index, total); });

  server.on("/api/action", HTTP_POST, answerEmptyPost, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostAction(request, data, len, index, total); });

  // GPS update rate
    server.on("/api/gpsRate", HTTP_POST, answerEmptyPost, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      uint8_t *body = collectJsonBody(request, data, len, index, total, 256);
      if (!body) return;
      JsonDocument doc;
      if (deserializeJson(doc, body, total)) { request->send(400); return; }
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
  // Nobody can reach the page once the AP is off, so nothing may stay in a test
  // mode (V5.08, mirrors SpeedPulser): Speed Test would hold the needle off the
  // source, Calibration Mode the motor at the calibration drive, until a power cycle.
#if SPP_HAS_VOLTAGE_CTRL
  autoFillStop("Stopped: WiFi went to sleep");
#endif
  speedTestOn = false;
  speedTestKph = 0;
  testRPM = false;
  calibrationModeOn = false;
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
