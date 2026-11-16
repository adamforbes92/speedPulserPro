#include "SpeedPulserPro_tasks.h"
#include "SpeedPulserPro_config.h"
#include "SpeedPulserPro_eep.h"
#include "SpeedPulserPro_wifi.h"
#include "SpeedPulserPro_gps.h"
#include "SpeedPulserPro_dsg.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_can.h"
#include "SpeedPulserPro_uds.h"
#include "SpeedPulserPro_io.h"
#include "SpeedPulserPro_voltage.h"
#include "SpeedPulserPro_driveCal.h"

// Task handles
TaskHandle_t taskEEPHandle = NULL;
TaskHandle_t taskUIUpdateHandle = NULL;
TaskHandle_t taskGPSHandle = NULL;
TaskHandle_t taskDSGHandle = NULL;
TaskHandle_t taskSpeedHandle = NULL;
TaskHandle_t taskRPMHandle = NULL;
TaskHandle_t taskBroadcastSpeedHandle = NULL;
TaskHandle_t taskTP20Handle = NULL;
TaskHandle_t taskUDSHandle  = NULL;
TaskHandle_t taskDiagHandle = NULL;

/**
 * Task: Write EEPROM/Preferences at regular intervals
 * Called every DELAY_EEPROM milliseconds (5000ms = 5 seconds)
 */
void taskWriteEEP(void *parameter)
{
  DEBUG_EEP("Write task started");

  while (1)
  {
    // Write current settings to EEPROM
    writeEEP();
    DEBUG_EEP("Settings written to EEPROM");

    // Wait for the next write; requestSettingsSave() cuts the wait short (V5.09).
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DELAY_EEPROM));
  }
}

// Save now rather than within 5 s (V5.09): a board test result or a calibration
// table change was lost if the power went in those 5 s.
void requestSettingsSave()
{
  if (taskEEPHandle != NULL)
    xTaskNotifyGive(taskEEPHandle);
}

/**
 * Task: Update WiFi UI labels with current system status
 * Called every DELAY_UI milliseconds (200ms)
 */
void taskUpdateUI(void *parameter)
{
  DEBUG_WIFI("UI update task started");

  while (1)
  {
    // Update WiFi interface labels
    updateLabels();

    // Delay before next update
    vTaskDelay(pdMS_TO_TICKS(DELAY_UI));
  }
}

/**
 * Task: Parse GPS data continuously
 * Called every DELAY_GPS milliseconds (100ms)
 * Reads GPS serial data and updates position/speed
 */
void taskParseGPS(void *parameter)
{
  DEBUG_GPS("Parse task started");

  while (1)
  {
    // Parse incoming GPS data
    parseGPS();

#if enableDebug && debugGPS
    if (gps.speed.isUpdated())
    {
      DEBUG_GPS("Speed updated to %d km/h", (int)gpsSpeed);
    }
#endif

    // Delay before next parse
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/**
 * Task: Calculate DSG transmission speed
 * Called every DELAY_DSG milliseconds (50ms)
 * Uses RPM and gear information to calculate vehicle speed
 */
void taskParseDSG(void *parameter)
{
  DEBUG_DSG("Parse task started");

  while (1)
  {
    // Calculate DSG speed from RPM and gear ratio
    parseDSG();

#if enableDebug && debugDSG
    if (dsgSpeed > 0)
    {
      DEBUG_DSG("Speed %u km/h, Gear %d", dsgSpeed, gear);
    }
#endif

    // Delay before next parse
    vTaskDelay(pdMS_TO_TICKS(DELAY_DSG));
  }
}

/**
 * Task: speed source -> needle target -> motor, every DELAY_SPEED (50 ms).
 *
 * V5.06 (mirrors SpeedPulser V5.06): the hall / VR inputs are timed pulse by pulse
 * and averaged over the Smoothing time, km/h = Hz x 3600 / pulses per km (no more
 * integer map() onto Max Speed + block median). The offset, then km/h -> mph, are
 * applied in floating point and rounded once (toNeedleTarget). On the S3 the drive
 * calibration runs on the 100 ms tick; the DevKit writes the preset duty. The DevKit's
 * legacy PID and the S3's "Voltage Control off" mode are gone.
 */
void taskProcessSpeed(void *parameter)
{
  DEBUG_SPD("Speed processing task started");
  float lastValidSpeedKph = 0.0f;
  uint8_t overRangeCount = 0;                 // loops in a row the selected source has read above Max Speed
  TickType_t lastTick = xTaskGetTickCount();  // 100 ms measure + control cadence
  TickType_t motorRunSince = 0;               // when the motor started running with no tacho seen yet

  while (1)
  {
#if SPP_HAS_VOLTAGE_CTRL
    // Test Board owns the motor PWM and the rail while it runs — stay hands-off.
    if (boardTestActive)
    {
      vTaskDelay(pdMS_TO_TICKS(DELAY_SPEED));
      continue;
    }
#endif

    // One 100 ms tick for everything that measures or closes a loop: the tacho
    // (and on the S3 the rail) are read here in EVERY mode, so the read-outs never
    // freeze, and the drive control below runs on the same tick with the fresh reading.
    const bool tick = (xTaskGetTickCount() - lastTick) >= pdMS_TO_TICKS(100);
    if (tick)
    {
      lastTick = xTaskGetTickCount();
#if SPP_HAS_VOLTAGE_CTRL // the DevKit has no tacho or rail to measure
      updateMotorRail();
      updateMeasuredFreq();
      autoFillTick(); // Auto Fill sets the calibration drive (no-op unless running)
#endif
    }

    // Speed sensors, measured in every mode so the read-outs never freeze.
    hallFrequencyHz = measureHallFrequency();
    const float hallKph = hallFrequencyHz * 3600.0f / hallPulsesPerKm;
    hallSpeed = (uint16_t)lroundf(hallKph);
    vehicleSpeedHall = hallSpeed;
#if SPP_HAS_VR
    vrFrequencyHz = measureVRFrequency();
    const float vrKph = vrFrequencyHz * 3600.0f / vrPulsesPerKm;
    vrSpeed = (uint16_t)lroundf(vrKph);
    vehicleSpeedVR = vrSpeed;
#endif

#if SPP_HAS_VOLTAGE_CTRL
    // S3: once the motor has run for a moment with no tacho seen, tell the UI the
    // feedback signal is missing. The DevKit has no tacho wired.
    if (motorDutyApplied > 0)
    {
      if (motorRunSince == 0)
        motorRunSince = xTaskGetTickCount();
    }
    else
    {
      motorRunSince = 0;
    }
    feedbackMissing = motorRunSince != 0 &&
                      ((!feedbackAvailable && (xTaskGetTickCount() - motorRunSince) > pdMS_TO_TICKS(1500)) ||
                       (tachoSignalLost() && !calibrationModeOn));
#else
    (void)motorRunSince;
#endif

    // The selected source's speed, in every mode (V5.09): the CAN broadcast, the
    // "car stopped" checks and the dashboard use this, not the Speed Test value or
    // a speed frozen by Calibration Mode. Only one source should be on, but if two
    // are, the LAST one below wins; activeSpeedSourceName() in _wifi.cpp checks them
    // in the same order so the web page agrees.
    float sourceKph = 0.0f;
    if (useHall)
      sourceKph = hallKph;
#if SPP_HAS_VR
    if (useVR)
      sourceKph = vrKph;
#endif
    if (useECU)
      sourceKph = ecuSpeed;
    if (useABS)
      sourceKph = absSpeed;
    if (useDSG)
      sourceKph = dsgSpeed;
    if (useGPS)
      sourceKph = gpsSpeed;
    if (useUDS)
      sourceKph = udsSpeed;
    if (useTP20)
      sourceKph = tp20Speed;
    if (useAftermarket)
      sourceKph = aftermarketSpeed;
    inputSpeedKph = sourceKph > (float)maxSpeed ? (float)maxSpeed : sourceKph;

    // Calibration Mode: the user (or Auto Fill) sets the motor directly — the drive
    // in volts on the S3, the raw 12-bit duty on the DevKit.
    if (calibrationModeOn)
    {
      needleTargetSpeed = 0;
#if SPP_HAS_VOLTAGE_CTRL
      if (tick)
        applyMotorDrive(calibrationDriveV);
#else
      setMotorDutyRaw((uint32_t)constrain(calibrationProbeDuty, 0L, (long)PWM_DUTY_MAX));
#endif
      vTaskDelay(pdMS_TO_TICKS(DELAY_SPEED));
      continue;
    }

    float speedKph = 0.0f;
    if (speedTestOn)
    {
      speedKph = (float)constrain(speedTestKph, 0L, (long)maxSpeed);
    }
    else
    {
      if (sourceKph <= (float)maxSpeed)
      {
        speedKph = sourceKph;
        lastValidSpeedKph = speedKph;
        overRangeCount = 0;
      }
      else if (overRangeCount < SPEED_OVER_RANGE_LOOPS)
      {
        // Above Max Speed for a moment: treat it as a spike (CAN error value, GPS glitch)
        // and hold the last good speed
        overRangeCount++;
        speedKph = lastValidSpeedKph;
      }
      else
      {
        // Still above Max Speed after 0.5 s: it's real, so park the needle at Max Speed.
        speedKph = (float)maxSpeed;
      }
    }
    vehicleSpeed = (uint16_t)lroundf(speedKph);
    needleTargetSpeed = toNeedleTarget(speedKph);

#if enableDebug && debugSpeed
    if (vehicleSpeed > 0)
      DEBUG_SPD("vehicleSpeed=%u km/h  needle=%u", vehicleSpeed, needleTargetSpeed);
#endif

#if SPP_HAS_VOLTAGE_CTRL
    // Drive from the calibration marks (+ the slow feedback trim), on the 100 ms
    // tick. Runs at target 0 too so the motor stops and the rail parks at the floor.
    if (tick)
      applyDriveControl(needleTargetSpeed);
#else
    setMotorDutyRaw(needleTargetSpeed == 0 ? 0 : speedToPwmDuty(needleTargetSpeed));
#endif

    vTaskDelay(pdMS_TO_TICKS(DELAY_SPEED));
  }
}

/**
 * Task: Process RPM from selected source
 * Called every DELAY_RPM milliseconds (50ms)
 * Handles Hall sensor RPM, CAN RPM, test mode, and PWM output mapping
 */
void taskProcessRPM(void *parameter)
{
  DEBUG_RPM("RPM processing task started");

  while (1)
  {
    // Hz <-> RPM scale: Cluster Frequency Limit (maxRPM, Hz) = Cluster RPM Limit.
    // Only used to show the hall RPM and to turn CAN / Test RPM into Hz. The hall
    // input itself is passed through in Hz, so this scale can't touch it — up to
    // V5.01 it went Hz -> RPM -> Hz through two truncating map()s and came out ~1 Hz
    // (~30 RPM) low.
    const float hzPerRpm = (clusterRPMLimit > 0) ? (float)maxRPM / (float)clusterRPMLimit : 0.0f;
    const float rpmPerHz = (maxRPM > 0) ? (float)clusterRPMLimit / (float)maxRPM : 0.0f;

    // Windowed frequency: read-and-clear the RPM accumulator once per loop so the
    // reading is a true average over the window rather than one jittery period.
    float rpmHz = readRPMHz();
    if (rpmHz >= 0.0f)
      rpmInputHz = rpmHz;

    // Always compute Hall RPM for live display, but clear stale data after timeout
    bool rpmInputStale = (millis() + 10 - lastPulseRPM) > durationReset;
    if (rpmInputStale)
    {
      rpmInputHz = 0.0f;
      vehicleRPMHall = 0;
      resetRPMPulseCounter();
    }
    else
    {
      vehicleRPMHall = (uint16_t)constrain(lroundf(rpmInputHz * rpmPerHz), 0L, 65535L);
    }

    // Handle RPM input - test mode or selected live source. Each branch sets the
    // output frequency (Hz) and the RPM shown/broadcast.
    float targetFreqRPM = 0.0f;
    if (testRPM)
    {
      vehicleRPM = (uint16_t)constrain(tempRPM, 0L, (long)clusterRPMLimit);
      targetFreqRPM = vehicleRPM * hzPerRpm;
    }
    else if (useRPMCAN)
    {
      vehicleRPM = min(vehicleRPMCAN, clusterRPMLimit);
      targetFreqRPM = vehicleRPM * hzPerRpm;
    }
    else
    {
      if (rpmInputStale)
      {
        resetRPMMedianFilter();
      }
      else if (averageFilterRPM <= 1)
      {
        // Reset first: it clears filteredRPMHz too (up to V5.01 the reset came
        // second, so 1 filter sample always output 0 RPM).
        resetRPMMedianFilter();
        filteredRPMHz = rpmInputHz;
      }
      else if (rpmHz >= 0.0f)
      {
        // Window-driven median top-up: feed one averaged sample per capture
        // window. The windowed frequency has already averaged out tooth-spacing
        // jitter, so the median just trims the occasional outlier — unlike the old
        // single-period path where one latched glitch filled several median slots
        // and slipped through (10k, 2k, ...).
        if (rawCountRPM < averageFilterRPM)
        {
          samplesRPM.add(rpmInputHz);
          rawCountRPM++;
        }

        if (rawCountRPM >= averageFilterRPM)
        {
          filteredRPMHz = samplesRPM.getMedian();
          rawCountRPM = 0;
          samplesRPM.clear();
        }
      }

      // Hall input -> output 1:1 in Hz, capped at the cluster limit like CAN / Test.
      targetFreqRPM = min(filteredRPMHz, (float)maxRPM);
      vehicleRPM = (uint16_t)constrain(lroundf(targetFreqRPM * rpmPerHz), 0L, (long)clusterRPMLimit);
    }

    // Drive the coil only when coilType is enabled. setFrequencyRPM() skips the
    // timer re-arm when the period hasn't changed.
    if (!coilType)
      targetFreqRPM = 0.0f;
    setFrequencyRPM(targetFreqRPM);
    frequencyRPM = lroundf(targetFreqRPM);

#if enableDebug && debugRPM
    if (vehicleRPM > 0)
    {
      DEBUG_RPM("vehicleRPM=%d  frequencyRPM=%ld", vehicleRPM, frequencyRPM);
    }
#endif

    // Delay before next processing
    vTaskDelay(pdMS_TO_TICKS(DELAY_RPM));
  }
}

/**
 * Task: Broadcast final vehicle speed over CAN at 20ms intervals.
 */
void taskBroadcastSpeed(void *parameter)
{
  DEBUG_CAN("Speed broadcast task started");

  while (1)
  {
    sendBroadcastSpeedFrame();
    vTaskDelay(pdMS_TO_TICKS(DELAY_BROADCAST_SPEED));
  }
}

/**
 * Task: 1 Hz diagnostics telemetry
 * Prints one concise line per subsystem so the live system state is easy to
 * follow on the serial monitor. Self-gated: each line only compiles when its
 * subsystem debug flag (and enableDebug) is on, so at release (enableDebug 0)
 * the whole loop body collapses to nothing.
 */
void taskDiagnostics(void *parameter)
{
  DEBUG("Diagnostics task started (1 Hz telemetry)");

  while (1)
  {
    DEBUG("uptime=%lus  heap=%u  minHeap=%u",
          (unsigned long)(millis() / 1000UL),
          (unsigned)ESP.getFreeHeap(),
          (unsigned)ESP.getMinFreeHeap());
    DEBUG_SPD("vehicle=%u  hall=%u (%.1f Hz)  gps=%u  dsg=%u  needle=%u  duty=%u",
              vehicleSpeed, hallSpeed, hallFrequencyHz, gpsSpeed, dsgSpeed,
              needleTargetSpeed, motorDutyApplied);
    DEBUG_RPM("vehicleRPM=%u  hallRPM=%u  canRPM=%u  freqRPM=%ld",
              vehicleRPM, vehicleRPMHall, vehicleRPMCAN, frequencyRPM);
    DEBUG_FB("enable=%d  measured=%u  freq=%.1f Hz",
             feedbackEnable ? 1 : 0, measuredSpeed, measuredFreqHz);
    DEBUG_DSG("speed=%u kph  gear=%u", dsgSpeed, gear);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * Initialize all FreeRTOS tasks
 * Called from setup()
 */
void tasksInit()
{
  DEBUG("Initialising FreeRTOS tasks...");

  // Create EEPROM write task
  BaseType_t status = xTaskCreate(
      taskWriteEEP,         // Task function
      "TaskEEP",            // Task name
      STACK_SIZE_EEPROM,    // Stack size (words)
      NULL,                 // Parameter
      TASK_PRIORITY_EEPROM, // Priority
      &taskEEPHandle        // Task handle
  );
  if (status != pdPASS)
  {
    DEBUG("ERROR: Failed to create EEPROM task");
  }

  // Create UI update task
  status = xTaskCreate(
      taskUpdateUI,
      "TaskUI",
      STACK_SIZE_UI,
      NULL,
      TASK_PRIORITY_UI,
      &taskUIUpdateHandle);
  if (status != pdPASS)
  {
    DEBUG("ERROR: Failed to create UI update task");
  }

  // Create GPS parse task
  status = xTaskCreate(
      taskParseGPS,
      "TaskGPS",
      STACK_SIZE_GPS,
      NULL,
      TASK_PRIORITY_GPS,
      &taskGPSHandle);
  if (status != pdPASS)
  {
    DEBUG("ERROR: Failed to create GPS task");
  }

  // Create DSG parse task
  status = xTaskCreate(
      taskParseDSG,
      "TaskDSG",
      STACK_SIZE_DSG,
      NULL,
      TASK_PRIORITY_DSG,
      &taskDSGHandle);
  if (status != pdPASS)
  {
    DEBUG("ERROR: Failed to create DSG task");
  }

  // Create Speed processing task
  status = xTaskCreate(
      taskProcessSpeed,
      "TaskSpeed",
      STACK_SIZE_SPEED,
      NULL,
      TASK_PRIORITY_SPEED,
      &taskSpeedHandle);
  if (status != pdPASS)
  {
    DEBUG("ERROR: Failed to create Speed task");
  }

  // Create RPM processing task
  status = xTaskCreate(
      taskProcessRPM,
      "TaskRPM",
      STACK_SIZE_RPM,
      NULL,
      TASK_PRIORITY_RPM,
      &taskRPMHandle);
  if (status != pdPASS)
  {
    DEBUG("ERROR: Failed to create RPM task");
  }

  status = xTaskCreate(
      taskBroadcastSpeed,
      "TaskBroadcast",
      STACK_SIZE_BROADCAST,
      NULL,
      TASK_PRIORITY_BROADCAST,
      &taskBroadcastSpeedHandle);
  if (status != pdPASS)
  {
    DEBUG("ERROR: Failed to create Broadcast Speed task");
  }

  DEBUG("All tasks initialised successfully");

  status = xTaskCreate(
      taskTP20,
      "TaskTP20",
      4096,
      NULL,
      2,
      &taskTP20Handle);
  if (status != pdPASS)
  {
    DEBUG("ERROR: Failed to create TP2.0 task");
  }

  status = xTaskCreate(
      taskUDS,
      "TaskUDS",
      4096,
      NULL,
      2,
      &taskUDSHandle);
  if (status != pdPASS)
  {
    DEBUG("ERROR: Failed to create UDS task");
  }

  // Create 1 Hz diagnostics/telemetry task (compiles to a no-op body at release)
  status = xTaskCreate(
      taskDiagnostics,
      "TaskDiag",
      4096,
      NULL,
      1,
      &taskDiagHandle);
  if (status != pdPASS)
  {
    DEBUG("ERROR: Failed to create Diagnostics task");
  }
}

/**
 * Suspend all background tasks
 * Useful when performing critical operations
 */
void tasksSuspendAll()
{
  if (taskEEPHandle != NULL)
    vTaskSuspend(taskEEPHandle);
  if (taskUIUpdateHandle != NULL)
    vTaskSuspend(taskUIUpdateHandle);
  if (taskGPSHandle != NULL)
    vTaskSuspend(taskGPSHandle);
  if (taskDSGHandle != NULL)
    vTaskSuspend(taskDSGHandle);
  if (taskSpeedHandle != NULL)
    vTaskSuspend(taskSpeedHandle);
  if (taskRPMHandle != NULL)
    vTaskSuspend(taskRPMHandle);
  if (taskBroadcastSpeedHandle != NULL)
    vTaskSuspend(taskBroadcastSpeedHandle);
  if (taskDiagHandle != NULL)
    vTaskSuspend(taskDiagHandle);
  DEBUG("All tasks suspended");
}

/**
 * Resume all background tasks
 */
void tasksResumeAll()
{
  if (taskEEPHandle != NULL)
    vTaskResume(taskEEPHandle);
  if (taskUIUpdateHandle != NULL)
    vTaskResume(taskUIUpdateHandle);
  if (taskGPSHandle != NULL)
    vTaskResume(taskGPSHandle);
  if (taskDSGHandle != NULL)
    vTaskResume(taskDSGHandle);
  if (taskSpeedHandle != NULL)
    vTaskResume(taskSpeedHandle);
  if (taskRPMHandle != NULL)
    vTaskResume(taskRPMHandle);
  if (taskBroadcastSpeedHandle != NULL)
    vTaskResume(taskBroadcastSpeedHandle);
  if (taskDiagHandle != NULL)
    vTaskResume(taskDiagHandle);
  DEBUG("All tasks resumed");
}

