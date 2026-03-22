#include "Arduino.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include "SpeedPulserPro_wifi.h"
#include "SpeedPulserPro_motorCal.h"
#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_tasks.h"

void handleGetSettings(AsyncWebServerRequest *request)
{
#ifdef serialDebugWifi
  Serial.println("GET /api/settings");
#endif

  JsonDocument doc;
  doc["hasNeedleSweep"] = hasNeedleSweep;
  doc["coilType"] = coilType;
  doc["broadcastSpeed"] = broadcastSpeed;
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
  else if (useUDS)
    doc["speedType"] = "TP2.0-DSG";
  else
    doc["speedType"] = "Hall";

  if (useRPMCAN)
    doc["rpmType"] = "CAN";
  else
    doc["rpmType"] = "Hall";

  doc["FW_VERSION"] = "2.00";

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
  doc["ecuSpeed"] = ecuSpeed;
  doc["absSpeed"] = absSpeed;
  doc["dsgSpeed"] = dsgSpeed;
  doc["gpsSpeed"] = gpsSpeed;
  doc["udsSpeed"] = udsSpeed;

  // Test mode status
  doc["testSpeedo"] = testSpeedo;
  doc["tempSpeed"] = tempSpeed;
  doc["testRPM"] = testRPM;
  doc["tempRPM"] = tempRPM;
  doc["testCal"] = testCal;
  doc["tempDutyCycle"] = tempDutyCycle;

  // System status
  doc["hasCAN"] = hasCAN;
  doc["hasGPS"] = hasGPS;
  doc["broadcastSpeed"] = broadcastSpeed;
  doc["gpsTaskSuspended"] = gpsTaskSuspended;
  doc["gpsSatellites"] = gps.satellites.value();

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

  if (key == "coilType")
  {
    coilType = (value == "true" || value == "1");
  }

  if (key == "broadcastSpeed")
  {
    broadcastSpeed = (value == "true" || value == "1");
    setBroadcastSpeedTaskEnabled(broadcastSpeed);
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
    tempSpeed = value.toInt();
  }

  if (key == "testRPM")
  {
    testRPM = (value == "true" || value == "1");
  }

  if (key == "testCal")
  {
    testCal = (value == "true" || value == "1");
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
    }

    if (value == "ECU")
    {
      useHall = false;
      useDSG = false;
      useECU = true;
      useABS = false;
      useGPS = false;
      useUDS = false;
    }

    if (value == "DSG")
    {
      useHall = false;
      useDSG = true;
      useECU = false;
      useABS = false;
      useGPS = false;
      useUDS = false;
    }

    if (value == "ABS")
    {
      useHall = false;
      useDSG = false;
      useECU = false;
      useABS = true;
      useGPS = false;
      useUDS = false;
    }

    if (value == "GPS")
    {
      useHall = false;
      useDSG = false;
      useECU = false;
      useABS = false;
      useGPS = true;
      useUDS = false;
    }

    if (value == "TP2.0-DSG" || value == "TP/UDS DSG" || value == "UDS")
    {
      useHall = false;
      useDSG = false;
      useECU = false;
      useABS = false;
      useGPS = false;
      useUDS = true;
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
    tempSpeed = doc["value"];
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

      // Reboot only after a successful flash write
      if (success) {
        delay(500); // Short delay to ensure response is sent before rebooting
        ESP.restart();
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
