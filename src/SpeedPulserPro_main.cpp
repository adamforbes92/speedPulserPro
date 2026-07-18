#include "Arduino.h"
#include "SpeedPulserPro_version.h"
#include "SpeedPulserPro_config.h"
#include "SpeedPulserPro_globals.h"
#include "SpeedPulserPro_can.h"
#include "SpeedPulserPro_wifi.h"
#include "SpeedPulserPro_io.h"
#include "SpeedPulserPro_gps.h"
#include "SpeedPulserPro_eep.h"
#include "SpeedPulserPro_dsg.h"
#include "SpeedPulserPro_motorCal.h"
#include "SpeedPulserPro_tasks.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_savvycan.h"
#include "power_manager.h"

// Forward declarations for main.cpp functions
void setup();
void loop();

void setup()
{
  // Always begin Serial - many GPS/rate diagnostic prints are unconditional
  // and parts of the framework misbehave writing to a never-begun UART.
  Serial.begin(115200);
  Serial.setTimeout(10);
#if serialDebug || serialDebugIncoming || serialDebugWifi || serialDebugEEP || serialDebugGPS || ChassisCANDebug
  Serial.println("[Main] Initialising SpeedPulser Pro...");
#endif

  basicInit();        // Initialize hardware, interrupts, CAN, GPS, etc.
  setupTimer();       // Set up hardware timer for RPM output
  updateMotorArray(); // Load motor calibration data into array for quick lookup

  if (hasNeedleSweep)
  {
    needleSweep(); // Perform initial needle sweep on startup if enabled
  }

  tasksInit();                               // Initialize FreeRTOS tasks for background operations
  setMotorDuty(dutyCycle);                   // Ensure initial duty cycle is set to zero to turn off motor

  connectWifi();    // Start WiFi
  setupUI();        // Set up web server and API
  setupAnalyzer();  // Start SavvyCAN analyzer task (idle until mode is enabled)

  // Universal reduced-power module: turns WiFi off 1 min after the last client
  // disconnects, scales CPU 240->80 MHz, releases Bluetooth and kills the
  // onboard LED to cut current draw (and therefore linear-regulator heat).
  power_config_t pcfg = powerDefaultConfig();
  pcfg.verbose = serialDebugWifi;
  powerInit(&pcfg);
}

void loop()
{
  // Apply GPS rate from EEP after satellite lock (flagged by parseGPS).
  // Suspend all background tasks for the duration so the outgoing PUBX bytes
  // on SoftwareSerial aren't disrupted by other core-1 tasks or the speed-input
  // pin ISR during the blocking delay() calls inside setGPSUpdateRate. The
  // API path doesn't need this because AsyncTCP callbacks run isolated on core 0.
  if (gpsAutoRateApplyPending())
  {
    tasksSuspendAll();
    String resp;
    bool ok = setGPSUpdateRate(gpsUpdateRateHz, resp);
    tasksResumeAll();
    (void)ok; // only referenced by the serialDebugGPS log below
#if serialDebugGPS
    Serial.print(F("[GPS Auto] Rate apply "));
    Serial.print(ok ? F("OK: ") : F("FAILED: "));
    Serial.println(resp);
#endif
  }

  if (tempNeedleSweep)
  {
    tasksSuspendAll(); // Suspend all tasks to prevent interference with needle sweep
    needleSweep();     // Perform needle sweep
    tasksResumeAll();  // Resume all tasks after sweep completes
    tempNeedleSweep = false;
  }

  if (ledCounter > averageFilterHall)
  {
    ledOnboard = !ledOnboard;
    digitalWrite(pinOnboardLED, ledOnboard);
    ledCounter = 0;
  }
}
