#ifndef WIFI_H
#define WIFI_H

#include "SpeedPulserPro_config.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ========== FUNCTION PROTOTYPES ==========

// Web server initialization
void setupUI();
void connectWifi();
void disconnectWifi();
void setupOTA();
void updateLabels();

// API Request handlers
void handleGetSettings(AsyncWebServerRequest *request);
void handleGetStatus(AsyncWebServerRequest *request);
void handleGetCalibrations(AsyncWebServerRequest *request);
void handleGetCalCurve(AsyncWebServerRequest *request);
void handlePostControl(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handlePostAction(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);

#endif // WIFI_H
