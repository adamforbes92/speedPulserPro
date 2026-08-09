#include "SpeedPulserPro_config.h"
#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_calBuilder.h"
#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
// SpeedPulserPro — User Calibration Builder implementation.
// See SpeedPulserPro_calBuilder.h for an overview.
// ---------------------------------------------------------------------------

CalPoint customCalPoints[MAX_CAL_POINTS];
uint8_t  customCalCount = 0;
char     customCalName[CAL_NAME_MAX] = "My Calibration";
bool     customCalUnitMph = false;
bool     customCalValid = false;

// Dedicated EEP handle so we never disturb the global settings namespace.
static Preferences calPref;
static const char *CAL_NS = "spCal";

// ===== Table generation ====================================================
// Expand the sorted set into the 385-entry motorPerformance[] table.
// Index i maps to hardware duty (i << DUTY_SCALE_SHIFT). The result is a
// speed curve, which findClosestMatch() relies on.
void buildCustomCalTable() {
  for (uint16_t i = 0; i < 385; i++) {
    motorPerformance[i] = 0;
  }

  if (customCalCount < 2) {
    customCalValid = false;
    DEBUG_CTRL("customCal: need >=2 points to build (have %u)", (unsigned)customCalCount);
    return;
  }

  const CalPoint &first = customCalPoints[0];
  const CalPoint &last  = customCalPoints[customCalCount - 1];

  // The 385-entry table is indexed by hardware duty (0..384). For each duty,
  // find the two anchor duties that bracket it and interpolate the speed.
  for (uint16_t i = 0; i < 385; i++) {
    uint32_t duty12 = (uint32_t)i << DUTY_SCALE_SHIFT;  // i * 4

    if (duty12 < first.duty) {
      motorPerformance[i] = 0;                // dead-zone below the first known point
      continue;
    }
    if (duty12 >= last.duty) {
      motorPerformance[i] = last.speed;       // flat hold above the last known point
      continue;
    }

    // Find the two key duty points and linearly interpolate between them.
    for (uint8_t p = 0; p < customCalCount - 1; p++) {
      uint16_t d0 = customCalPoints[p].duty;
      uint16_t d1 = customCalPoints[p + 1].duty;
      if (duty12 >= d0 && duty12 <= d1) {
        int32_t s0 = customCalPoints[p].speed;
        int32_t s1 = customCalPoints[p + 1].speed;
        int32_t sp = (d1 == d0)
                       ? s1
                       : s0 + (s1 - s0) * (int32_t)(duty12 - d0) / (int32_t)(d1 - d0);
        if (sp < 0) sp = 0;
        motorPerformance[i] = (uint16_t)sp;
        break;
      }
    }
  }

  // Force non-decreasing so lookups stay well behaved even if the user
  // captured slightly noisy / out-of-order anchors.
  for (uint16_t i = 1; i < 385; i++) {
    if (motorPerformance[i] < motorPerformance[i - 1]) {
      motorPerformance[i] = motorPerformance[i - 1];
    }
  }

  customCalValid = true;
  DEBUG_CTRL("customCal: built table from %u points (max speed=%u)",
             (unsigned)customCalCount, (unsigned)last.speed);
}

// ===== Runtime lookup ======================================================
// Interpolate a requested speed directly to a full 12-bit duty using the raw
// anchor points. Anchors are sorted by speed ascending and (being a real gauge
// response) monotonic, so we can bracket by speed and interpolate the duty.
// This preserves the full 0..PWM_DUTY_MAX range the anchors were captured at,
// unlike the 385-entry table whose index only reaches duty 384<<DUTY_SCALE_SHIFT.
uint32_t customSpeedToDuty12(uint16_t speed) {
  if (customCalCount < 2) return 0;

  const CalPoint &first = customCalPoints[0];
  const CalPoint &last  = customCalPoints[customCalCount - 1];

  if (speed <= first.speed) {
    return (speed < first.speed) ? 0 : first.duty;  // dead-zone below the first anchor
  }
  if (speed >= last.speed) {
    return last.duty;                               // hold the top anchor above its speed
  }

  for (uint8_t p = 0; p < customCalCount - 1; p++) {
    uint16_t s0 = customCalPoints[p].speed;
    uint16_t s1 = customCalPoints[p + 1].speed;
    if (speed >= s0 && speed <= s1) {
      uint16_t d0 = customCalPoints[p].duty;
      uint16_t d1 = customCalPoints[p + 1].duty;
      if (s1 == s0) return d1;
      return (uint32_t)d0 + (uint32_t)(d1 - d0) * (uint32_t)(speed - s0) / (uint32_t)(s1 - s0);
    }
  }
  return last.duty;
}

// ===== Anchor editing ======================================================
bool calAddPoint(uint16_t speed, uint16_t duty) {
  if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;

  // Re-capturing an existing speed just updates its duty (and stays in place,
  // since the list is keyed by speed). This is what the user expects when they
  // delete a point and add it again.
  for (uint8_t i = 0; i < customCalCount; i++) {
    if (customCalPoints[i].speed == speed) {
      customCalPoints[i].duty = duty;
      buildCustomCalTable();
      return true;
    }
  }

  if (customCalCount >= MAX_CAL_POINTS) {
    DEBUG_CTRL("customCal: point list full (%u)", (unsigned)customCalCount);
    return false;
  }

  // Keep the list sorted by speed ascending so points always appear
  // in numerical order regardless of the order they were captured in.
  uint8_t pos = customCalCount;
  for (uint8_t i = 0; i < customCalCount; i++) {
    if (speed < customCalPoints[i].speed) {
      pos = i;
      break;
    }
  }
  for (uint8_t i = customCalCount; i > pos; i--) {
    customCalPoints[i] = customCalPoints[i - 1];
  }
  customCalPoints[pos].speed = speed;
  customCalPoints[pos].duty  = duty;
  customCalCount++;

  buildCustomCalTable();
  return true;
}

bool calDeletePoint(uint8_t index) {
  if (index >= customCalCount) return false;
  for (uint8_t i = index; i < customCalCount - 1; i++) {
    customCalPoints[i] = customCalPoints[i + 1];
  }
  customCalCount--;
  buildCustomCalTable();
  return true;
}

void calClearPoints() {
  customCalCount = 0;
  customCalValid = false;
  buildCustomCalTable();
}

void calSetName(const char *name) {
  if (!name) return;
  strncpy(customCalName, name, CAL_NAME_MAX - 1);
  customCalName[CAL_NAME_MAX - 1] = '\0';
}

// ===== Save / load =========================================================
void calSaveToNvs() {
  calPref.begin(CAL_NS, false);
  calPref.putUChar("count", customCalCount);
  calPref.putString("name", customCalName);
  calPref.putBool("mph", customCalUnitMph);
  if (customCalCount > 0) {
    calPref.putBytes("pts", customCalPoints, sizeof(CalPoint) * customCalCount);
  }
  calPref.end();
  DEBUG_EEP("customCal: saved %u points (\"%s\")", (unsigned)customCalCount, customCalName);
}

void calBuilderInit() {
  calPref.begin(CAL_NS, true);  // read-only
  customCalCount = calPref.getUChar("count", 0);
  if (customCalCount > MAX_CAL_POINTS) customCalCount = 0;

  String nm = calPref.getString("name", "My Calibration");
  strncpy(customCalName, nm.c_str(), CAL_NAME_MAX - 1);
  customCalName[CAL_NAME_MAX - 1] = '\0';

  customCalUnitMph = calPref.getBool("mph", false);

  if (customCalCount > 0) {
    size_t got = calPref.getBytes("pts", customCalPoints, sizeof(CalPoint) * customCalCount);
    if (got != sizeof(CalPoint) * customCalCount) {
      customCalCount = 0;  // blob missing / corrupt — start clean
    }
  }
  calPref.end();

  if (customCalCount >= 2) {
    buildCustomCalTable();
  }
  DEBUG_EEP("customCal: loaded %u points (\"%s\")", (unsigned)customCalCount, customCalName);
}

// ===== Export / import =====================================================
void calExportJson(String &out) {
  JsonDocument doc;
  doc["name"]     = customCalName;
  doc["unit"]     = customCalUnitMph ? "mph" : "kmh";
  doc["pwmBits"]  = PWM_RESOLUTION;
  doc["maxSpeed"] = maxSpeed;

  JsonArray pts = doc["points"].to<JsonArray>();
  for (uint8_t i = 0; i < customCalCount; i++) {
    JsonObject p = pts.add<JsonObject>();
    p["duty"]  = customCalPoints[i].duty;
    p["speed"] = customCalPoints[i].speed;
  }

  serializeJsonPretty(doc, out);
}

void calExportCArray(String &out) {
  // Build the 385-entry table from the current anchors first.
  buildCustomCalTable();

  out  = "// ";
  out += customCalName;
  out += customCalUnitMph ? " - array in mph\n" : " - array in kmh\n";
  out += "uint16_t motorPerformance18[] PROGMEM = {";
  for (uint16_t i = 0; i < 385; i++) {
    if (i) out += ", ";
    out += String(motorPerformance[i]);
  }
  out += "};\n";
  out += "// add to calibrationProfiles[]:\n// {\"";
  out += customCalName;
  out += "\", motorPerformance18},";
}

bool calImportJson(const char *json) {
  if (!json) return false;

  JsonDocument doc;
  if (deserializeJson(doc, json)) {
    DEBUG_WIFI("customCal: import parse failed");
    return false;
  }

  JsonArray pts = doc["points"].as<JsonArray>();
  if (pts.isNull()) return false;

  const char *nm = doc["name"] | "Imported Calibration";
  calSetName(nm);
  const char *unit = doc["unit"] | "kmh";
  customCalUnitMph = (strcmp(unit, "mph") == 0);

  customCalCount = 0;
  for (JsonObject p : pts) {
    uint16_t duty  = p["duty"]  | 0;
    uint16_t speed = p["speed"] | 0;
    calAddPoint(speed, duty);  // keeps list sorted + rebuilds
  }

  DEBUG_WIFI("customCal: imported %u points (\"%s\")", (unsigned)customCalCount, customCalName);
  return customCalCount >= 2;
}
