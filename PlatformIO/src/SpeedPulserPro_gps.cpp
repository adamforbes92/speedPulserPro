#include "SpeedPulserPro_gps.h"

static unsigned long lastGPSData = 0;
static const unsigned long GPS_TIMEOUT = 10000; // 10 seconds
static unsigned long charsProcessedPrevious = 0;

void parseGPS()
{


  // Always try to read serial data
  while (ss.available() > 0)
  {
    gps.encode(ss.read());
  }

  // Check if NEW characters have been processed (not just old cumulative count)
  unsigned long charsProcessedCurrent = gps.charsProcessed();
  bool gotNewData = false;
  if (charsProcessedCurrent > charsProcessedPrevious)
  {
    lastGPSData = millis();
    charsProcessedPrevious = charsProcessedCurrent;
    gotNewData = true;
  }

  // 1. If no new GPS characters in 10s, unavailable
  if (useGPS && (millis() - lastGPSData > GPS_TIMEOUT)) {
    gpsUnavailable = true;
    hasError = true;
    hasGPS = false;
    return;
  }

  // 2. If new chars, but no satellites, not connected
  if (gotNewData && gps.satellites.value() == 0) {
    gpsUnavailable = false;
    hasError = false;
    hasGPS = false;
    return;
  }

  // 3. If satellites > 0, connected
  if (gps.satellites.value() > 0) {
    gpsUnavailable = false;
    hasError = false;
    hasGPS = true;
    return;
  }

  if (gps.speed.isUpdated())
  {
    gpsSpeed = int(gps.speed.kmph());
#if serialDebugGPS
    Serial.println(gps.satellites.value());
    Serial.println(gps.hdop.hdop());

    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);

    Serial.print(F("GPS Speed: "));
    Serial.println(gpsSpeed);
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
