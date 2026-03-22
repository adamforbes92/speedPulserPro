#include "SpeedPulserPro_gps.h"

static unsigned long lastGPSData = 0;
static const unsigned long GPS_TIMEOUT = 10000;  // 10 seconds

void parseGPS() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  if (gps.satellites.value() == 0) {
    hasError = true;
    hasGPS = false;
  } else {
    hasError = false;
    hasGPS = true;
    lastGPSData = millis();  // Update last GPS data time when data is received
  }

  // Check if no GPS data received within 10 seconds
  if (useGPS && (millis() - lastGPSData > GPS_TIMEOUT)) {
    gpsTaskSuspended = true;
  } else if (hasGPS) {
    gpsTaskSuspended = false;
  }

  if (gps.speed.isUpdated()) {
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

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
}
