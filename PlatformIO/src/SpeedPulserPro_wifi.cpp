#include "Arduino.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include "SpeedPulserPro_wifi.h"
#include "SpeedPulserPro_motorCal.h"
#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_tasks.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_gps.h"
#include "SpeedPulserPro_savvycan.h"

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
#ifdef serialDebugWifi
  Serial.println("GET /api/settings");
#endif

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

  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

void handleGetCalibrations(AsyncWebServerRequest *request)
{
#ifdef serialDebugWifi
  Serial.println("GET /api/calibrations");
#endif

  JsonDocument doc;
  JsonArray calibrations = doc["calibrations"].to<JsonArray>();
  const uint8_t calibrationCount = getCalibrationCount();

  for (uint8_t index = 1; index <= calibrationCount; index++)
  {
    JsonObject item = calibrations.add<JsonObject>();
    item["id"] = index;
    item["name"] = getCalibrationText(index);
  }

  doc["currentCalibrationId"] = motorPerformanceVal;

  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

void handlePostControl(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  if (index + len != total) return; // wait for complete body
  JsonDocument doc;
  deserializeJson(doc, data, len);

  String key = doc["key"];
  String value = doc["value"].as<String>();

#ifdef serialDebugWifi
  Serial.print("POST /api/control - ");
  Serial.print(key);
  Serial.print(" = ");
  Serial.println(value);
#endif

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
#ifdef serialDebugWifi
  Serial.println("[WiFi] Setting up web server...");
#endif

  // Initialize LittleFS filesystem
  if (!LittleFS.begin(false))
  { // true = format if mount failed
#ifdef serialDebugWifi
    Serial.println("[LittleFS] Mount failed");
#endif
  }
  else
  {
#ifdef serialDebugWifi
    Serial.println("[LittleFS] Successfully mounted");
#endif
  }

  // Serve static files from LittleFS
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.serveStatic("/style.css", LittleFS, "/style.css");
  server.serveStatic("/app.js", LittleFS, "/app.js");

  // API routes for getting data
  server.on("/api/settings", HTTP_GET, handleGetSettings);
  server.on("/api/status", HTTP_GET, handleGetStatus);
  server.on("/api/calibrations", HTTP_GET, handleGetCalibrations);

  // API routes for receiving commands
  server.on("/api/control", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostControl(request, data, len, index, total); });

  server.on("/api/action", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostAction(request, data, len, index, total); });

  // Separate endpoints for RPM and Speed tests
  server.on("/api/testRPM", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostTestRPM(request, data, len, index, total); });

  server.on("/api/testSpeed", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            { handlePostTestSpeed(request, data, len, index, total); });

  // OTA firmware upload route
  server.on("/api/ota", HTTP_POST, [](AsyncWebServerRequest *request)
            {
      bool success = !Update.hasError();
      request->send(success ? 200 : 500, "application/json", success ? "{\"success\":true}" : "{\"success\":false}");

      // Reboot only after a successful flash write.
      // Use a FreeRTOS task so the async TCP stack can flush the HTTP response
      // before the restart — delay() would block the Arduino task and prevent
      // ESPAsyncWebServer from transmitting the response.
      if (success) {
        xTaskCreate([](void*) {
          vTaskDelay(pdMS_TO_TICKS(1500));
          ESP.restart();
          vTaskDelete(nullptr);
        }, "ota_restart", 2048, nullptr, 1, nullptr);
      } }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
            {
      if (index == 0) {
#ifdef serialDebugWifi
        Serial.printf("[OTA] Upload start: %s\n", filename.c_str());
#endif
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
#ifdef serialDebugWifi
          Update.printError(Serial);
#endif
        }
      }

      if (!Update.hasError()) {
        if (Update.write(data, len) != len) {
#ifdef serialDebugWifi
          Update.printError(Serial);
#endif
        }
      }

      if (final) {
        if (!Update.end(true)) {
#ifdef serialDebugWifi
          Update.printError(Serial);
#endif
        }
#ifdef serialDebugWifi
        Serial.printf("[OTA] Upload complete: %u bytes\n", index + len);
#endif
      } });

  // OTA filesystem (LittleFS) upload route
  server.on("/api/ota/fs", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      bool success = !Update.hasError();
      request->send(success ? 200 : 500, "application/json", success ? "{\"success\":true}" : "{\"success\":false}");
      if (success) {
        xTaskCreate([](void*) {
          vTaskDelay(pdMS_TO_TICKS(1500));
          ESP.restart();
          vTaskDelete(nullptr);
        }, "ota_fs_restart", 2048, nullptr, 1, nullptr);
      }
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (index == 0) {
#ifdef serialDebugWifi
        Serial.printf("[OTA-FS] Upload start: %s\n", filename.c_str());
#endif
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_SPIFFS)) {
#ifdef serialDebugWifi
          Update.printError(Serial);
#endif
        }
      }
      if (!Update.hasError()) {
        if (Update.write(data, len) != len) {
#ifdef serialDebugWifi
          Update.printError(Serial);
#endif
        }
      }
      if (final) {
        if (!Update.end(true)) {
#ifdef serialDebugWifi
          Update.printError(Serial);
#endif
        }
#ifdef serialDebugWifi
        Serial.printf("[OTA-FS] Upload complete: %u bytes\n", index + len);
#endif
      }
    }
  );

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

#ifdef serialDebugWifi
  Serial.println("[WiFi] Web server started");
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.softAPIP());
#endif
}

void connectWifi()
{
  WiFi.setHostname(wifiHostName);
#ifdef serialDebugWifi
  DEBUG_PRINTLN("[WiFi] Beginning WiFi...");
  DEBUG_PRINTLN("[WiFi] Creating Access Point...");
#endif

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(wifiHostName);

#ifdef serialDebugWifi
  DEBUG_PRINT("[WiFi] AP SSID: ");
  DEBUG_PRINTLN(wifiHostName);
  DEBUG_PRINT("[WiFi] AP IP: ");
  DEBUG_PRINTLN(WiFi.softAPIP());
#endif
}

void disconnectWifi()
{
  DEBUG_PRINTF("[WiFi] Number of connections: ");
  DEBUG_PRINTLN(WiFi.softAPgetStationNum());

  if (WiFi.softAPgetStationNum() == 0)
  {
    DEBUG_PRINTLN("[WiFi] No connections, turning off");
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
