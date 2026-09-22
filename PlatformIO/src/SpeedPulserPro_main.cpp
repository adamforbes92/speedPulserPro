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
#include "SpeedPulserPro_calBuilder.h"
#include "SpeedPulserPro_tasks.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_savvycan.h"
#include "SpeedPulserPro_voltage.h"
#include "power_manager.h"
#include "wifi_manager.h"

// Forward declarations for main.cpp functions
void setup();
void loop();

void setup()
{
  // Always begin Serial - many GPS/rate diagnostic prints are unconditional
  // and parts of the framework misbehave writing to a never-begun UART.
  Serial.begin(baudSerial);
  Serial.setTimeout(10);
  DEBUG("SpeedPulser Pro booting  |  FW %s  |  debug=%d", FW_VERSION, enableDebug);

  // Stop the motor drive IMMEDIATELY: pinMotorOutput floats at reset, so drive it
  // low to prevent random motor movement on boot. Then sample the board-version
  // strap (GPIO25) before anything else uses the pin.
  pinMode(pinMotorOutput, OUTPUT);
  digitalWrite(pinMotorOutput, LOW);
  detectBoardVersion();

  basicInit();        // Initialize hardware, interrupts, CAN, GPS, etc. (also readEEP)
  setupTimer();       // Set up hardware timer for RPM output
  calBuilderInit();   // Load any user (SpeedPulser) custom calibration from NVS
  updateMotorArray(); // Load motor calibration data into array for quick lookup

  // V4 board only: bring up the adjustable motor supply. The buck must be enabled
  // for the motor to have any voltage, so enable it regardless of the control mode —
  // starting at the minimum (safe) rail with the motor PWM already off.
  initVoltageControl();
  if (boardHasVoltageControl)
  {
    enableBuck();
    if (!voltageControlEnable)
    {
      setMotorVoltageCmd(vcVoltMax); // legacy PWM behaviour: hold full motor volts
    }
  }

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
  pcfg.verbose = (enableDebug && debugPower);
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
    (void)ok;
    DEBUG_GPS("Auto rate apply %s: %s", ok ? "OK" : "FAILED", resp.c_str());
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

  wifiManagerTick(); // Home WiFi (bridge mode): connection tracking + retry back-off
}
