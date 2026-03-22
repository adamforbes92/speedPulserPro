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

// Forward declarations for main.cpp functions
void setup();
void loop();

void setup()
{
#if serialDebug || serialDebugIncoming || serialDebugWifi || serialDebugEEP || serialDebugGPS || ChassisCANDebug
  Serial.begin(115200);
  Serial.setTimeout(10); // non-blocking TX: don't stall if no USB host is connected
  DEBUG_PRINTLN("Initialising SpeedPulser Pro...");
#endif

  basicInit();        // Initialize hardware, interrupts, CAN, GPS, etc.
  setupTimer();       // Set up hardware timer for RPM output
  updateMotorArray(); // Load motor calibration data into array for quick lookup

  if (hasNeedleSweep)
  {
    needleSweep(); // Perform initial needle sweep on startup if enabled
  }

  tasksInit();                               // Initialize FreeRTOS tasks for background operations
  ledcWrite(LEDC_OUTPUT_CHANNEL, dutyCycle); // Ensure initial duty cycle is set to zero to turn off motor

  connectWifi(); // Start WiFi
  setupUI();     // Set up web server and API
}

void loop()
{
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
