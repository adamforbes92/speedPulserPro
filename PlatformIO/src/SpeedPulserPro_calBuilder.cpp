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
// Expand the sorted set into a 385-entry speed table (the presetSpeedTable[]
// layout). Index i maps to hardware duty (i << DUTY_SCALE_SHIFT). The result is a
// speed curve, which findClosestMatch() relies on. false = fewer than 2 points.
static bool fillCustomTable(uint16_t *table) {
  for (uint16_t i = 0; i < 385; i++) {
    table[i] = 0;
  }
  if (customCalCount < 2) {
    return false;
  }

  const CalPoint &first = customCalPoints[0];
  const CalPoint &last  = customCalPoints[customCalCount - 1];

  // The 385-entry table is indexed by hardware duty (0..384). For each duty,
  // find the two anchor duties that bracket it and interpolate the speed.
  for (uint16_t i = 0; i < 385; i++) {
    uint32_t duty12 = (uint32_t)i << DUTY_SCALE_SHIFT;  // i * 4

    if (duty12 < first.duty) {
      table[i] = 0;                           // dead-zone below the first known point
      continue;
    }
    if (duty12 >= last.duty) {
      table[i] = last.speed;                  // flat hold above the last known point
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
        table[i] = (uint16_t)sp;
        break;
      }
    }
  }

  // Force non-decreasing so lookups stay well behaved even if the user
  // captured slightly noisy / out-of-order anchors.
  for (uint16_t i = 1; i < 385; i++) {
    if (table[i] < table[i - 1]) {
      table[i] = table[i - 1];
    }
  }
  return true;
}

// The live table is only rebuilt while the custom cal is the selected one (V5.08,
// mirrors SpeedPulser): before, every Builder edit and every Export overwrote
// presetSpeedTable[], so a DevKit running a preset was left on the custom curve, or
// an all-zero table (needle dead), until the next reboot.
void buildCustomCalTable() {
  customCalValid = customCalCount >= 2;
  if (presetId == CUSTOM_CAL_ID)
    fillCustomTable(presetSpeedTable);
  DEBUG_CTRL("customCal: %u points%s", (unsigned)customCalCount,
             presetId == CUSTOM_CAL_ID ? " (selected: table rebuilt)" : "");
}

// ===== Runtime lookup ======================================================
// Interpolate a requested speed directly to a full 12-bit duty using the raw
// anchor points. Anchors are sorted by speed, and calAddPoint() only accepts a
// point whose duty also rises with speed, so bracketing by speed brackets the duty.
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
      const int32_t d0 = customCalPoints[p].duty;
      const int32_t d1 = customCalPoints[p + 1].duty;
      if (s1 == s0) return (uint32_t)d1;
      // Signed: a falling pair (only possible from a pre-V5.06 table) must not
      // wrap round to a huge duty.
      int32_t duty = d0 + (d1 - d0) * (int32_t)(speed - s0) / (int32_t)(s1 - s0);
      return (uint32_t)constrain(duty, 0, (int32_t)PWM_DUTY_MAX);
    }
  }
  return last.duty;
}

// ===== Anchor editing ======================================================
// A higher speed must need more duty than every lower one. A point that breaks
// that (needle misread, motor binding) is refused: between two such points the
// interpolated duty would run the wrong way.
bool calAddPoint(uint16_t speed, uint16_t duty, const char *&err) {
  if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
  err = "OK";
  if (speed < 1 || speed > 400) {
    err = "Speed must be 1-400";
    return false;
  }
  for (uint8_t i = 0; i < customCalCount; i++) {
    const CalPoint &q = customCalPoints[i];
    if (q.speed == speed) continue;
    if (q.speed < speed ? (q.duty >= duty) : (q.duty <= duty)) {
      err = "Out of order: a higher speed needs more duty than the points below it. Check the reading, or delete the old point";
      return false;
    }
  }

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
    err = "Point list full: delete one first";
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
// Points first, count second; the load takes the count from the points' own size
// (a power cut between the two writes can't lose the calibration).
// The load takes the count from the points' own size; the stored count only says
// "cleared" (0). So a non-zero count is written before the points and a cleared
// table's points are removed before the 0 (V5.09, mirrors SpeedPulser): a power cut
// between the writes leaves the old calibration, never "0 points" over a fresh one.
void calSaveToNvs() {
  calPref.begin(CAL_NS, false);
  const uint8_t n = customCalCount;
  if (n > 0) {
    calPref.putUChar("count", n);
    calPref.putBytes("pts", customCalPoints, sizeof(CalPoint) * n);
  } else {
    if (calPref.isKey("pts")) calPref.remove("pts");
    calPref.putUChar("count", 0);
  }
  calPref.putString("name", customCalName);
  calPref.putBool("mph", customCalUnitMph);
  calPref.end();
  DEBUG_EEP("customCal: saved %u points (\"%s\")", (unsigned)customCalCount, customCalName);
}

void calBuilderInit() {
  calPref.begin(CAL_NS, true);  // read-only
  const size_t blobBytes = calPref.getBytesLength("pts");
  customCalCount = (blobBytes % sizeof(CalPoint) == 0) ? (uint8_t)(blobBytes / sizeof(CalPoint)) : 0;
  if (customCalCount > MAX_CAL_POINTS) customCalCount = 0;
  if (calPref.isKey("count") && calPref.getUChar("count", 0) == 0) customCalCount = 0; // cleared table

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
  doc["type"]     = PWM_CAL_FILE_TYPE;
  doc["about"]    = "PWM calibration (DevKit board): needle speeds vs motor PWM duty. "
                    "Not a voltage calibration.";
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
  // Its own copy of the 385-entry table: the running one is left alone.
  static uint16_t table[385];
  fillCustomTable(table);

  out  = "// ";
  out += customCalName;
  out += customCalUnitMph ? " - array in mph\n" : " - array in kmh\n";
  out += "// PWM calibration preset (DevKit board): speed per PWM duty step\n";
  out += "uint16_t motorPerformance18[] PROGMEM = {";
  for (uint16_t i = 0; i < 385; i++) {
    if (i) out += ", ";
    out += String(table[i]);
  }
  out += "};\n";
  out += "// add to calibrationProfiles[]:\n// {\"";
  out += customCalName;
  out += "\", motorPerformance18},";
}

// The whole file is checked before anything changes: the points must be in
// order (duty rising with speed), or the current calibration is left alone.
bool calImportJson(const char *json, const char *&err) {
  err = "Import failed: not a SpeedPulser calibration file";
  if (!json) return false;

  JsonDocument doc;
  if (deserializeJson(doc, json)) return false;

  // Files from before V5.01 have no "type"; anything else tagged isn't ours.
  const char *type = doc["type"] | PWM_CAL_FILE_TYPE;
  if (strcmp(type, PWM_CAL_FILE_TYPE) != 0) return false;

  JsonArray pts = doc["points"].as<JsonArray>();
  if (pts.isNull() || pts.size() < 2) {
    err = "The file needs at least 2 points";
    return false;
  }
  if (pts.size() > MAX_CAL_POINTS) {
    err = "Too many points in the file";
    return false;
  }
  CalPoint tmp[MAX_CAL_POINTS];
  uint8_t n = 0;
  for (JsonObject p : pts) {
    CalPoint q;
    q.speed = p["speed"] | 0;
    q.duty = p["duty"] | 0;
    if (q.speed < 1 || q.speed > 400 || q.duty > PWM_DUTY_MAX) {
      err = "A point in the file has a bad speed or duty";
      return false;
    }
    uint8_t pos = n;
    while (pos > 0 && tmp[pos - 1].speed > q.speed) {
      tmp[pos] = tmp[pos - 1];
      pos--;
    }
    tmp[pos] = q;
    n++;
  }
  for (uint8_t i = 1; i < n; i++) {
    if (tmp[i].speed == tmp[i - 1].speed || tmp[i].duty <= tmp[i - 1].duty) {
      err = "The points in the file are out of order (duty must rise with speed)";
      return false;
    }
  }
  calSetName(doc["name"] | "Imported Calibration");
  customCalUnitMph = (strcmp(doc["unit"] | "kmh", "mph") == 0);
  memcpy(customCalPoints, tmp, sizeof(CalPoint) * n);
  customCalCount = n;
  buildCustomCalTable();
  err = "OK";

  DEBUG_WIFI("customCal: imported %u points (\"%s\")", (unsigned)customCalCount, customCalName);
  return customCalCount >= 2;
}
