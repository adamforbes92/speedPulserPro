#include "SpeedPulserPro_gps.h"
#include "SpeedPulserPro_eep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>

static unsigned long lastGPSData = 0;
static const unsigned long GPS_TIMEOUT = 10000; // 10 seconds
static unsigned long charsProcessedPrevious = 0;
static unsigned long passedChecksumPrevious = 0;
static unsigned long failedChecksumPrevious = 0;

// For GPS update frequency calculation
static unsigned long gpsUpdateCount = 0;
static unsigned long gpsFreqWindowStart = 0;
static float gpsUpdateFrequency = 0.0f;
static unsigned long lastGPSDiagLogMs = 0;
static const unsigned long GPS_COMMAND_DELAY_MS = 250;
static const unsigned long GPS_COMMAND_SETTLE_MS = 500;
static const unsigned long GPS_DIAG_LOG_INTERVAL_MS = 1000;
static const unsigned long GPS_DEFAULT_BAUD = 9600UL;
static const unsigned long GPS_HIGH_RATE_BAUD = 38400UL;
static SemaphoreHandle_t gpsSerialMutex = nullptr;

// Tracks the actual baud rate the ESP serial is currently running at.
// This is always GPS_DEFAULT_BAUD (9600) after initGPS(), regardless of what
// gpsUpdateRateHz was restored from EEPROM, because the GPS module itself
// always resets to 9600 on power-up.
static unsigned long gpsCurrentSerialBaud = GPS_DEFAULT_BAUD;

// Tracks whether the one-time auto baud switch (9600 → 38400) has been attempted.
static bool gpsBaudSwitchDone = false;

// Satellite stability timer and rate-apply flag.
static unsigned long gpsSatStableStartMs = 0;
static bool gpsRateApplied = false;
static bool gpsPendingRateApply = false;
static const unsigned long GPS_SAT_STABLE_MS = 20000;

// Persistent GPS update rate (Hz), default 1Hz
uint8_t gpsUpdateRateHz = 1;

float getGPSUpdateFrequency()
{
  return gpsUpdateFrequency;
}

// Returns seconds remaining until the auto rate apply will fire.
// Returns:
//   -1 if no pending auto-apply (rate already applied, or stored rate is 1Hz)
//    0 if the timer hasn't started yet (no satellites seen)
//   >0 = seconds left until it fires
int gpsAutoApplySecondsRemaining()
{
  if (gpsRateApplied || gpsUpdateRateHz <= 1)
  {
    return -1;
  }
  if (gpsSatStableStartMs == 0)
  {
    return (int)(GPS_SAT_STABLE_MS / 1000UL);
  }
  unsigned long elapsed = millis() - gpsSatStableStartMs;
  if (elapsed >= GPS_SAT_STABLE_MS)
  {
    return 0;
  }
  return (int)((GPS_SAT_STABLE_MS - elapsed + 999UL) / 1000UL);
}

bool gpsAutoRateApplyPending()
{
  if (gpsPendingRateApply)
  {
    gpsPendingRateApply = false;
    return true;
  }
  return false;
}

// UBX command bytes for different rates (from NeoGPS ubloxRate.ino)
static const uint8_t UBX_1HZ[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};
static const uint8_t UBX_5HZ[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
static const uint8_t UBX_10HZ[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
static const uint8_t UBX_16HZ[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x3E, 0x00, 0x01, 0x00, 0x01, 0x00, 0x54, 0xB6};
static const char PUBX_BAUD_9600[] = "PUBX,41,1,3,3,9600,0";
static const char PUBX_BAUD_38400[] = "PUBX,41,1,3,3,38400,0";
static const char PUBX_BAUD_57600[] = "PUBX,41,1,3,3,57600,0";
static const char PUBX_BAUD_115200[] = "PUBX,41,1,3,3,115200,0";

// Forward declaration: initGPS uses this helper before its full definition.
static bool applyGPSBaudRate(unsigned long baud, String &responseMsg);
static bool gpsProbeBaud(unsigned long baud);

// Startup behavior: always initialize GPS serial at default baud.
unsigned long initGPS()
{
#if serialDebugGPS
  Serial.print(F("[GPS Init] Starting GPS serial at default baud: "));
  Serial.println(GPS_DEFAULT_BAUD);
#endif

  ss.setRxBufferSize(1024); // Generous RX buffer for 38400-baud bursts at 5 Hz.

  // The GPS module keeps its last-configured baud until it actually loses power.
  // If a previous session left it at the high-rate baud (38400) and only the ESP
  // was reset, opening the hardware UART at 9600 produces continuous framing
  // errors. Unlike the old bit-banged SoftwareSerial (which passed the bad bytes
  // through), the hardware UART silently discards framing-error bytes, so almost
  // nothing is received. Probe the known bauds and lock onto whichever one
  // actually yields valid (checksum-passing) NMEA.
  const unsigned long candidateBauds[] = {GPS_DEFAULT_BAUD, GPS_HIGH_RATE_BAUD};
  unsigned long detectedBaud = GPS_DEFAULT_BAUD;
  bool baudDetected = false;
  for (size_t i = 0; i < sizeof(candidateBauds) / sizeof(candidateBauds[0]); ++i)
  {
    if (gpsProbeBaud(candidateBauds[i]))
    {
      detectedBaud = candidateBauds[i];
      baudDetected = true;
#if serialDebugGPS
      Serial.print(F("[GPS Init] Detected GPS at "));
      Serial.print(detectedBaud);
      Serial.println(F(" baud."));
#endif
      break;
    }
  }

  if (!baudDetected)
  {
    // No valid NMEA at any known baud (e.g. no module / no antenna yet). Fall
    // back to the default baud; the auto-rate logic will retry after lock.
    detectedBaud = GPS_DEFAULT_BAUD;
    ss.end();
    delay(20);
    ss.begin(detectedBaud, SERIAL_8N1, pinRX_GPS, pinTX_GPS);
#if serialDebugGPS
    Serial.println(F("[GPS Init] No NMEA detected; defaulting to 9600 baud."));
#endif
  }

  charsProcessedPrevious = gps.charsProcessed();
  passedChecksumPrevious = gps.passedChecksum();
  failedChecksumPrevious = gps.failedChecksum();
  gpsUpdateCount = 0;
  gpsFreqWindowStart = 0;
  gpsUpdateFrequency = 0.0f;
  gpsBaudSwitchDone = false;
  gpsSatStableStartMs = 0;
  gpsRateApplied = false;
  gpsCurrentSerialBaud = detectedBaud;
  lastGPSData = millis();
  return detectedBaud;
}

// Probe a candidate baud: open the UART, feed received bytes to TinyGPS for a
// short window, and report whether any checksum-passing NMEA sentence arrives.
// u-blox emits NMEA continuously even without a satellite fix, so this works
// before lock. Called only from initGPS() during setup(), so blocking delays
// here are safe (no FreeRTOS tasks are running yet).
static bool gpsProbeBaud(unsigned long baud)
{
  ss.end();
  delay(20);
  ss.begin(baud, SERIAL_8N1, pinRX_GPS, pinTX_GPS);

  // Discard any partial/garbage bytes already sitting in the FIFO.
  delay(20);
  while (ss.available() > 0)
  {
    ss.read();
  }

  const unsigned long probeWindowMs = 1500;
  const unsigned long okBefore = gps.passedChecksum();
  const unsigned long startMs = millis();
  while (millis() - startMs < probeWindowMs)
  {
    while (ss.available() > 0)
    {
      gps.encode(ss.read());
    }
    if (gps.passedChecksum() > okBefore)
    {
      return true;
    }
    delay(5);
  }
  return false;
}

static SemaphoreHandle_t getGPSSerialMutex()
{
  if (gpsSerialMutex == nullptr)
  {
    gpsSerialMutex = xSemaphoreCreateMutex();
  }

  return gpsSerialMutex;
}

static const char *getGPSBaudCommand(unsigned long baud)
{
  if (baud == 9600UL)
  {
    return PUBX_BAUD_9600;
  }
  if (baud == 38400UL)
  {
    return PUBX_BAUD_38400;
  }
  if (baud == 57600UL)
  {
    return PUBX_BAUD_57600;
  }
  if (baud == 115200UL)
  {
    return PUBX_BAUD_115200;
  }

  return nullptr;
}

static void sendPUBXCommand(const char *commandBody)
{
  uint8_t checksum = 0;

  ss.write('$');
  while (*commandBody != '\0')
  {
    checksum ^= static_cast<uint8_t>(*commandBody);
    ss.write(*commandBody++);
  }

  char checksumSuffix[8];
  snprintf(checksumSuffix, sizeof(checksumSuffix), "*%02X\r\n", checksum);
  ss.print(checksumSuffix);
}

static bool applyGPSBaudRate(unsigned long baud, String &responseMsg)
{
  const char *baudCommand = getGPSBaudCommand(baud);
  if (baudCommand == nullptr)
  {
    responseMsg = "Unsupported GPS baud rate.";
    return false;
  }

#if serialDebugGPS
  Serial.print(F("[GPS Baud] Sending PUBX baud change command for "));
  Serial.print(baud);
  Serial.println(F(" baud."));
#endif
  sendPUBXCommand(baudCommand);
  // Do NOT call ss.flush() here. On the ESP32 hardware UART, flush() blocks until
  // the TX FIFO drains and was observed to hang indefinitely on UART2, freezing
  // the calling task (loop/AsyncTCP) so serial and WiFi stop updating. The short
  // PUBX command (<30 bytes) is fully sent well within the delay below.
  delay(1000); // Allow GPS extra time to process PUBX and switch baud internally.

  while (ss.available() > 0)
  {
    ss.read();
  }

  ss.end();
  delay(GPS_COMMAND_SETTLE_MS);
  ss.begin(baud, SERIAL_8N1, pinRX_GPS, pinTX_GPS);
  delay(GPS_COMMAND_SETTLE_MS); // Let the UART settle before any write.

#if serialDebugGPS
  Serial.print(F("[GPS Baud] Stage complete. Local serial restarted at "));
  Serial.print(baud);
  Serial.println(F(" baud."));
#endif

  charsProcessedPrevious = gps.charsProcessed(); // snapshot, not 0 — TinyGPS counter never resets
  passedChecksumPrevious = gps.passedChecksum();
  failedChecksumPrevious = gps.failedChecksum();
  gpsUpdateCount = 0;
  gpsFreqWindowStart = 0;
  gpsUpdateFrequency = 0.0f;
  lastGPSData = millis();
  gpsCurrentSerialBaud = baud;

  responseMsg = "GPS serial switched to " + String(baud) + " baud.";
  return true;
}

// Send UBX command to GPS to set update rate
bool setGPSUpdateRate(uint8_t rateHz, String &responseMsg)
{
  SemaphoreHandle_t serialMutex = getGPSSerialMutex();
  if ((serialMutex == nullptr) || (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1500)) != pdTRUE))
  {
    responseMsg = "GPS serial busy.";
    return false;
  }

#if serialDebugGPS
  Serial.print(F("[GPS Rate] Requested update rate: "));
  Serial.print(rateHz);
  Serial.print(F(" Hz. Current baud: "));
  Serial.print(gpsCurrentSerialBaud);
  Serial.print(F(", target baud: "));
  Serial.println((rateHz == 1) ? GPS_DEFAULT_BAUD : GPS_HIGH_RATE_BAUD);
#endif

  const bool rateChanged = (gpsUpdateRateHz != rateHz);
  const unsigned long targetBaud = (rateHz == 1) ? GPS_DEFAULT_BAUD : GPS_HIGH_RATE_BAUD;
  const uint8_t *cmd = nullptr;
  size_t len = 0;

  if (rateHz == 1)
  {
    cmd = UBX_1HZ;
    len = sizeof(UBX_1HZ);
  }
  else if (rateHz == 5)
  {
    cmd = UBX_5HZ;
    len = sizeof(UBX_5HZ);
  }
  else if (rateHz == 10)
  {
    cmd = UBX_10HZ;
    len = sizeof(UBX_10HZ);
  }
  else if (rateHz == 16)
  {
    cmd = UBX_16HZ;
    len = sizeof(UBX_16HZ);
  }
  else
  {
    responseMsg = "Invalid rate. Choose 1, 5, 10, or 16 Hz.";
    xSemaphoreGive(serialMutex);
    return false;
  }

  String baudResponse;

  if (targetBaud != gpsCurrentSerialBaud)
  {
    // A baud change is required between current and target.
    if (targetBaud > gpsCurrentSerialBaud)
    {
      // Switching to a higher baud (e.g. 9600 → 38400).
      // First settle the GPS at 1 Hz so the baud-switch PUBX command is not
      // drowned out by the high-rate output the GPS would start producing
      // immediately after the rate command.  This mirrors the manual
      // 1 Hz → 5 Hz API sequence that is known to work reliably.
#if serialDebugGPS
      Serial.println(F("[GPS Rate] Settling at 1 Hz before baud switch."));
#endif
      for (size_t i = 0; i < sizeof(UBX_1HZ); ++i)
      {
        ss.write(UBX_1HZ[i]);
      }
      // No ss.flush(): HardwareSerial flush() can hang on UART2; the delay covers TX.
      delay(GPS_COMMAND_DELAY_MS);
    }

    // Apply the baud switch (sends PUBX to GPS, restarts serial, resets counters).
    if (!applyGPSBaudRate(targetBaud, baudResponse))
    {
#if serialDebugGPS
      Serial.print(F("[GPS Rate] Baud apply failed: "));
      Serial.println(baudResponse);
#endif
      responseMsg = baudResponse;
      xSemaphoreGive(serialMutex);
      return false;
    }

    // Serial is now at targetBaud.  Send the target rate command at the new baud.
#if serialDebugGPS
    Serial.print(F("[GPS Rate] Sending UBX rate cmd at new baud ("));
    Serial.print(len);
    Serial.println(F(" bytes)."));
#endif
    for (size_t i = 0; i < len; ++i)
    {
      ss.write(cmd[i]);
    }
    // No ss.flush(): HardwareSerial flush() can hang on UART2; the delay covers TX.
    delay(GPS_COMMAND_DELAY_MS);
  }
  else
  {
    // No baud change needed.  Send the rate command at the current baud,
    // then refresh the serial state (clears buffers, resets counters).
#if serialDebugGPS
    Serial.print(F("[GPS Rate] Sending UBX rate cmd at current baud ("));
    Serial.print(len);
    Serial.println(F(" bytes)."));
#endif
    for (size_t i = 0; i < len; ++i)
    {
      ss.write(cmd[i]);
    }
    // No ss.flush(): HardwareSerial flush() can hang on UART2; the delay covers TX.
    delay(GPS_COMMAND_DELAY_MS);
    // Same baud: applyGPSBaudRate sends a no-op PUBX, restarts SS, resets counters.
    applyGPSBaudRate(targetBaud, baudResponse);
  }

  gpsUpdateRateHz = rateHz;

  xSemaphoreGive(serialMutex);

  if (rateChanged)
  {
    writeEEP();
  }

  responseMsg = "GPS update rate set to " + String(rateHz) + "Hz and " + baudResponse;

#if serialDebugGPS
  Serial.print(F("[GPS Rate] Stage complete: "));
  Serial.println(responseMsg);
#endif

  return true;
}

void parseGPS()
{
  SemaphoreHandle_t serialMutex = getGPSSerialMutex();
  if ((serialMutex == nullptr) || (xSemaphoreTake(serialMutex, 0) != pdTRUE))
  {
    return;
  }

  // Always try to read serial data
  while (ss.available() > 0)
  {
    gps.encode(ss.read());
  }

  xSemaphoreGive(serialMutex);

  // Check if NEW characters have been processed (not just old cumulative count)
  unsigned long charsProcessedCurrent = gps.charsProcessed();
  bool gotNewData = false;
  unsigned long charsDelta = 0;
  if (charsProcessedCurrent > charsProcessedPrevious)
  {
    charsDelta = charsProcessedCurrent - charsProcessedPrevious;
    lastGPSData = millis();
    charsProcessedPrevious = charsProcessedCurrent;
    gotNewData = true;
  }

  // Frequency is based on completed fix sentences, not serial byte throughput.
  unsigned long now = millis();
  if (gpsFreqWindowStart == 0)
  {
    gpsFreqWindowStart = now;
    passedChecksumPrevious = gps.passedChecksum();
    failedChecksumPrevious = gps.failedChecksum();
  }

  unsigned long passedChecksumCurrent = gps.passedChecksum();
  unsigned long failedChecksumCurrent = gps.failedChecksum();
  unsigned long passedDelta = 0;
  unsigned long failedDelta = 0;
  unsigned long parsedSentencesCurrent = passedChecksumCurrent + failedChecksumCurrent;
  unsigned long parsedSentencesPrevious = passedChecksumPrevious + failedChecksumPrevious;
  if (parsedSentencesCurrent > parsedSentencesPrevious)
  {
    gpsUpdateCount += (parsedSentencesCurrent - parsedSentencesPrevious);
    if (passedChecksumCurrent > passedChecksumPrevious)
    {
      passedDelta = passedChecksumCurrent - passedChecksumPrevious;
    }
    if (failedChecksumCurrent > failedChecksumPrevious)
    {
      failedDelta = failedChecksumCurrent - failedChecksumPrevious;
    }
    passedChecksumPrevious = passedChecksumCurrent;
    failedChecksumPrevious = failedChecksumCurrent;
  }

#if serialDebugGPS
  if ((now - lastGPSDiagLogMs >= GPS_DIAG_LOG_INTERVAL_MS) && (gotNewData || passedDelta > 0 || failedDelta > 0))
  {
    Serial.print(F("[GPS RX] +chars="));
    Serial.print(charsDelta);
    Serial.print(F(" +ok="));
    Serial.print(passedDelta);
    Serial.print(F(" +fail="));
    Serial.print(failedDelta);
    Serial.print(F(" totals(ok/fail)="));
    Serial.print(passedChecksumCurrent);
    Serial.print('/');
    Serial.println(failedChecksumCurrent);
    lastGPSDiagLogMs = now;
  }
#endif
  if (now - gpsFreqWindowStart >= 1000)
  {
    if (gpsUpdateCount > 0)
    {
      // Divide sentences/sec by 10 (u-blox sends ~9 NMEA sentences per fix)
      // and ceiling so 1Hz→1, 5Hz→5. Only update when sentences received so
      // the value doesn't flicker to 0 between bursts.
      gpsUpdateFrequency = int(gpsUpdateCount * 100.0f / (now - gpsFreqWindowStart)+1);
    }
    gpsUpdateCount = 0;
    gpsFreqWindowStart = now;
#if serialDebugGPS
    Serial.print(F("[GPS] Update Frequency: "));
    Serial.println(gpsUpdateFrequency);
#endif
  }

  // 1. No GPS serial for 10s => no module fitted (Not Available). Detected
  //    regardless of useGPS so the status reflects the physical module, not
  //    whether GPS is the selected speed source.
  if (millis() - lastGPSData > GPS_TIMEOUT)
  {
    gpsUnavailable = true;
    hasError = true;
    hasGPS = false;
    return;
  }

  // 2. If new chars, but no satellites, not connected
  if (gotNewData && gps.satellites.value() == 0)
  {
    gpsSatStableStartMs = 0; // Reset stability timer if satellites lost.
    gpsUnavailable = false;
    hasError = false;
    hasGPS = false;
    return;
  }

  // 3. If satellites > 0, connected.
  if (gps.satellites.value() > 0)
  {
    // Start satellite stable timer on first lock.
    if (gpsSatStableStartMs == 0)
    {
      gpsSatStableStartMs = now;
#if serialDebugGPS
      Serial.println(F("[GPS Auto] Satellites seen - 60s stability timer started."));
#endif
    }

    // Once stable for 60s, flag a rate apply (handled outside parseGPS in taskParseGPS).
    if (!gpsRateApplied && (now - gpsSatStableStartMs >= GPS_SAT_STABLE_MS))
    {
#if serialDebugGPS
      Serial.print(F("[GPS Auto] 60s satellite lock - queuing rate apply for "));
      Serial.print(gpsUpdateRateHz);
      Serial.println(F(" Hz."));
#endif
      gpsRateApplied = true;
      gpsPendingRateApply = true;
    }

    gpsUnavailable = false;
    hasError = false;
    hasGPS = true;

    gpsSpeed = int(gps.speed.kmph());
#if serialDebugGPS
    Serial.print(F("[GPS] Satellites: "));
    Serial.println(gps.satellites.value());
    Serial.print(F("[GPS] HDOP: "));
    Serial.println(gps.hdop.hdop());

    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);

    Serial.print(F("[GPS] Speed: "));
    Serial.println(gpsSpeed);
    Serial.print(F("[GPS] Updates/sec: "));
    Serial.println(gpsUpdateFrequency, 2);
#endif
  }

}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                         : vi >= 10    ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
}
