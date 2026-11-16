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
#include "SpeedPulserPro_resetLog.h"

// Forward declarations for main.cpp functions
void setup();
void loop();

void setup()
{
  // Always begin Serial - many GPS/rate diagnostic prints are unconditional
  // and parts of the framework misbehave writing to a never-begun UART.
  Serial.begin(baudSerial);
  Serial.setTimeout(10);
  DEBUG("SpeedPulser Pro booting  |  FW %s  |  %s (%s)  |  debug=%d", FW_VERSION, SPP_BOARD_NAME, SPP_BOARD_ID, enableDebug);
  resetLogInit(); // log why it started (V5.09, as SpeedPulser): power-on, brown-out, crash...

  // Stop the motor drive IMMEDIATELY: pinMotorOutput floats at reset, so drive it
  // low to prevent random motor movement on boot.
  pinMode(pinMotorOutput, OUTPUT);
  digitalWrite(pinMotorOutput, LOW);
#if SPP_HAS_VOLTAGE_CTRL
  detectBoardVersion(); // BOARD_ID strap sanity check, before anything else touches IO7
#endif

  basicInit();        // Initialize hardware, interrupts, CAN, GPS, etc. (also readEEP)
  updateInputGlitchFilters(); // speed-input glitch limits from the loaded sensor settings
  setupTimer();       // Set up hardware timer for RPM output
  calBuilderInit();   // Load any user (SpeedPulser) custom calibration from NVS
  updateMotorArray(); // Load motor calibration data into array for quick lookup

#if SPP_HAS_VOLTAGE_CTRL
  // S3 board: bring up the adjustable motor supply, starting at the minimum (safe)
  // rail with the motor PWM already off.
  initVoltageControl();
  enableBuck();
#endif

  if (needleSweepOnBoot)
  {
    needleSweep(); // Perform initial needle sweep on startup if enabled
  }

  tasksInit();                               // Initialize FreeRTOS tasks for background operations
  setMotorDutyRaw(0);                        // motor off

  connectWifi();    // Start WiFi
  setupUI();        // Set up web server and API
  setupAnalyzer();  // Start SavvyCAN analyzer task (idle until mode is enabled)

  // Universal reduced-power module: turns WiFi off 1 min after the last client
  // disconnects, scales CPU 240->80 MHz, releases Bluetooth and kills the
  // onboard LED to cut current draw (and therefore linear-regulator heat).
  power_config_t pcfg = powerDefaultConfig();
  pcfg.verbose = (enableDebug && debugPower);
  // The module's default is the DevKit's GPIO2; the S3 board drives its own LED
  // from ESP_LED, so hand it the board's pin rather than a spare GPIO.
  pcfg.onboardLedPin = pinOnboardLED;
  pcfg.ledActiveHigh = true;
  powerInit(&pcfg);
}

void loop()
{
  // Apply the GPS rate from EEP after satellite lock (flagged by parseGPS). V5.08:
  // the other tasks keep running. The suspend was for SoftwareSerial timing; the GPS
  // is on a hardware UART now (the web API calls the same function without it), and
  // suspending froze the speed and RPM needles for ~2 s about 20 s after every lock.
  if (gpsAutoRateApplyPending())
  {
    String resp;
    bool ok = setGPSUpdateRate(gpsUpdateRateHz, resp);
    (void)ok;
    DEBUG_GPS("Auto rate apply %s: %s", ok ? "OK" : "FAILED", resp.c_str());
  }

#if SPP_HAS_VOLTAGE_CTRL
  if (needleSweepRequested && boardTestActive)
  {
    needleSweepRequested = false; // the Test Board owns the motor; drop the sweep request
  }
#endif
  if (needleSweepRequested)
  {
    tasksSuspendAll(); // Suspend all tasks to prevent interference with needle sweep
    needleSweep();     // Perform needle sweep
    tasksResumeAll();  // Resume all tasks after sweep completes
    needleSweepRequested = false;
  }

  // Flash the LED once per 6 hall pulses, but not in reduced power: the power
  // manager turns the LED off there to save current (V5.09).
  if (ledCounter > 6 && powerGetState() == POWER_STATE_ACTIVE)
  {
    ledOnboard = !ledOnboard;
    digitalWrite(pinOnboardLED, ledOnboard);
    ledCounter = 0;
  }

  resetLogTick();    // marks this start "ran OK" once it has run 10 s
  wifiManagerTick(); // Home WiFi (bridge mode): connection tracking + retry back-off

  // Rest between passes (V5.09): without it this loop spun core 1 at 100 % and its
  // idle task never ran, costing current and heat. Nothing here needs < 10 ms.
  vTaskDelay(pdMS_TO_TICKS(LOOP_REST_MS));
}
