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

    // Delay before next write
    vTaskDelay(pdMS_TO_TICKS(DELAY_EEPROM));
  }
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
 * Task: Process vehicle speed from selected source
 * Called every DELAY_SPEED milliseconds (50ms)
 * Handles Hall sensor averaging, source selection, speed offset, and test mode
 */
void taskProcessSpeed(void *parameter)
{
  DEBUG_SPD("Speed processing task started");
#if SPP_HAS_VR
  int rawCountVR = 0;                              // VR median-filter fill counter
#endif
  uint16_t lastValidVehicleSpeed = 0;
  uint8_t overRangeCount = 0;                     // loops in a row the selected source has read above Max Speed
  bool prevTestSpeedo = false;                    // Speed Test edge detect (global on/off gate)
  TickType_t lastTick = xTaskGetTickCount();     // 100 ms measure + control cadence
  TickType_t motorRunSince = 0;                  // when the motor started running with no feedback yet

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
    // freeze, and the control laws below run on the same tick with the fresh reading.
    const bool tick = (xTaskGetTickCount() - lastTick) >= pdMS_TO_TICKS(100);
    if (tick)
    {
      lastTick = xTaskGetTickCount();
#if SPP_HAS_VOLTAGE_CTRL
      updateMotorRail();
#endif
      updateMeasuredFreq();
    }

#if SPP_HAS_VOLTAGE_CTRL
    // S3 board with voltage control engaged: the motor is driven from the drive
    // calibration (rail + PWM together) instead of the legacy duty tables.
    const bool voltageMode = voltageControlEnable;
#else
    const bool voltageMode = false; // DevKit: PWM motor drive only
    (void)voltageMode;
#endif

    // Speed Test acts as a global on/off: the instant it is switched OFF, drop the
    // held test speed and clear the hall state so the needle returns to rest (or the
    // live source) instead of sitting at the last test value.
    if (prevTestSpeedo && !testSpeedo)
    {
      lastValidVehicleSpeed = 0;
      vehicleSpeed = 0;
      dutyCycle = 0;
      dutyCycleIncoming = 0;
      vehicleSpeedHall = 0;
      hallSpeed = 0;
#if SPP_HAS_VR
      vehicleSpeedVR = 0;
      vrSpeed = 0;
      dutyCycleIncomingVR = 0;
      rawCountVR = 0;
      samplesVR.clear();
      resetVRPulseCounter();
#endif
      currentSpeedOffset = 0;
      resetHallMedianFilter();
      resetHallPulseCounter();
#if SPP_HAS_VOLTAGE_CTRL
      if (voltageMode)
      {
        resetDriveControl();
        applyMotorDrive(0.0f); // motor off, rail back to minimum
      }
      else
#endif
      {
        setMotorDutyRaw(0);
      }
    }
    prevTestSpeedo = testSpeedo;

    // PID may only run when feedback is actually available. If the user enables it
    // but the PCB has no feedback trace (legacy board) the loop stays open-loop, so
    // an unaware user can't accidentally drive a closed loop with no measurement.
    const bool pidActive = feedbackEnable && feedbackAvailable;

    // Feedback-availability signalling for the UI: once the motor has been running
    // for a moment with no tacho signal seen, mark it missing so the dashboard can
    // show "N/A" instead of "--". Clears automatically once feedback appears.
    const TickType_t FB_DETECT_TICKS = pdMS_TO_TICKS(1500);
    if (appliedDutyCycle > 0)
    {
      if (motorRunSince == 0)
      {
        motorRunSince = xTaskGetTickCount();
      }
    }
    else
    {
      motorRunSince = 0;
    }
    feedbackMissing = (!feedbackAvailable && motorRunSince != 0 &&
                       (xTaskGetTickCount() - motorRunSince) > FB_DETECT_TICKS);

    // Handle Hall sensor pulse processing and averaging
    if (!testSpeedo && !testCal)
    {
      // Windowed frequency: read-and-clear the accumulator once per loop so the
      // reading is a true average over the window instead of one jittery edge-to-edge
      // period. Returns <0 when no fresh edges arrived — hold the last value then.
      float hallHz = readHallHz();

      if ((millis() + 10 - lastPulse) > durationReset)
      {
        dutyCycle = 0;
        dutyCycleIncoming = 0;
        vehicleSpeedHall = 0;
        hallSpeed = 0;
        currentSpeedOffset = 0;
        resetHallMedianFilter();
        resetHallPulseCounter();
      }
      else if (hallHz >= 0.0f)
      {
        dutyCycleIncoming = (unsigned long)(hallHz + 0.5f);
        uint16_t mappedSpeed = map(dutyCycleIncoming, 0, maxFreqHall, 0, maxSpeed);

        if (averageFilterHall <= 1)
        {
          hallSpeed = mappedSpeed;
          vehicleSpeedHall = hallSpeed;
          resetHallMedianFilter();
        }
        else
        {
          if (rawCount < averageFilterHall)
          {
            samples.add(mappedSpeed);
            rawCount++;
          }

          if (rawCount >= averageFilterHall)
          {
            hallSpeed = (uint16_t)samples.getMedian();
            vehicleSpeedHall = hallSpeed;
            resetHallMedianFilter();
          }
        }
      }
    }

#if SPP_HAS_VR
    // Handle VR (variable-reluctance) pulse processing and averaging — counted and
    // mapped exactly like the hall input, but with its own frequency/filter state.
    if (!testSpeedo && !testCal)
    {
      float vrHz = readVRHz();

      if ((millis() + 10 - lastPulseVR) > durationReset)
      {
        vehicleSpeedVR = 0;
        vrSpeed = 0;
        dutyCycleIncomingVR = 0;
        rawCountVR = 0;
        samplesVR.clear();
        resetVRPulseCounter();
      }
      else if (vrHz >= 0.0f)
      {
        dutyCycleIncomingVR = (unsigned long)(vrHz + 0.5f);
        uint16_t mappedSpeed = map(dutyCycleIncomingVR, 0, maxFreqVR, 0, maxSpeed);

        if (averageFilterVR <= 1)
        {
          vrSpeed = mappedSpeed;
          vehicleSpeedVR = vrSpeed;
          rawCountVR = 0;
          samplesVR.clear();
        }
        else
        {
          if (rawCountVR < averageFilterVR)
          {
            samplesVR.add(mappedSpeed);
            rawCountVR++;
          }

          if (rawCountVR >= averageFilterVR)
          {
            vrSpeed = (uint16_t)samplesVR.getMedian();
            vehicleSpeedVR = vrSpeed;
            rawCountVR = 0;
            samplesVR.clear();
          }
        }
      }
    }
#endif // SPP_HAS_VR

    // Calibration mode drives the raw 12-bit hardware duty directly (0..PWM_DUTY_MAX)
    if (testCal)
    {
#if SPP_HAS_VOLTAGE_CTRL
      if (voltageMode)
      {
        // Drive calibration: the user sets the drive directly (rail first, then
        // PWM below Minimum Rail) until the needle sits on a mark, then captures it.
        if (tick)
        {
          const uint16_t d = applyMotorDrive(tempDriveV);
          dutyCycle = d;
          tempDutyCycle = d;
          pidCorrection = 0;
        }
      }
      else
#endif
      {
        dutyCycle = constrain(tempDutyCycle, 0, (long)PWM_DUTY_MAX);
        setMotorDutyRaw((uint32_t)dutyCycle);
      }
      // The tacho readout stays live (measured on the tick above) so measured
      // speed shows and feedbackAvailable can latch while building a calibration.
      vTaskDelay(pdMS_TO_TICKS(DELAY_SPEED));
      continue;
    }

    // Process test mode or select active speed source
    if (testSpeedo)
    {
      long requestedTestSpeed = tempSpeed;
      if (requestedTestSpeed >= 0 && requestedTestSpeed <= maxSpeed)
      {
        vehicleSpeed = (uint16_t)requestedTestSpeed;
        lastValidVehicleSpeed = vehicleSpeed;
      }
      else
      {
        vehicleSpeed = constrain(lastValidVehicleSpeed, 0, maxSpeed);
      }
      dutyCycle = applyConfiguredSpeedOffset(vehicleSpeed);
      if (convertToMPH)
      {
        dutyCycle = dutyCycle * mphFactor;
      }
    }
    else
    {
      uint16_t rawVehicleSpeed = 0;
      // Only one source should be on, but if two are, the LAST one below wins.
      // activeSpeedSourceName() in _wifi.cpp checks them in the same order so the web page agrees.
      if (useHall)
      {
        rawVehicleSpeed = hallSpeed;
      }
#if SPP_HAS_VR
      if (useVR)
      {
        rawVehicleSpeed = vrSpeed;
      }
#endif
      if (useECU)
      {
        rawVehicleSpeed = ecuSpeed;
      }
      if (useABS)
      {
        rawVehicleSpeed = absSpeed;
      }
      if (useDSG)
      {
        rawVehicleSpeed = dsgSpeed;
      }
      if (useGPS)
      {
        rawVehicleSpeed = gpsSpeed;
      }
      if (useUDS)
      {
        rawVehicleSpeed = udsSpeed;
      }
      if (useTP20)
      {
        rawVehicleSpeed = tp20Speed;
      }
      if (useAftermarket)
      {
        rawVehicleSpeed = aftermarketSpeed;
      }

      if (rawVehicleSpeed <= maxSpeed)
      {
        vehicleSpeed = rawVehicleSpeed;
        lastValidVehicleSpeed = vehicleSpeed;
        overRangeCount = 0;
      }
      else if (overRangeCount < SPEED_OVER_RANGE_LOOPS)
      {
        // Above Max Speed for a moment: treat it as a spike (CAN error value, GPS glitch)
        // and hold the last good speed
        overRangeCount++;
        vehicleSpeed = constrain(lastValidVehicleSpeed, 0, maxSpeed);
      }
      else
      {
        // Still above Max Speed after 0.5 s: it's real, so park the needle at Max Speed.
        // Up to V5.02 it held the last good speed for ever, which is 0 straight after
        // boot, so an ECU sending more than Max Speed never moved the needle at all
        vehicleSpeed = maxSpeed;
      }

      dutyCycle = applyConfiguredSpeedOffset(vehicleSpeed);
      if (convertToMPH)
      {
        dutyCycle = dutyCycle * mphFactor;
      }
    }

#if enableDebug && debugSpeed
    if (vehicleSpeed > 0)
    {
      DEBUG_SPD("vehicleSpeed=%d km/h  dutyCycle=%ld", vehicleSpeed, dutyCycle);
    }
#endif

    // Feed-forward base duty: interpolate the requested speed to a 12-bit hardware
    // duty (finer than the raw calibration grid), mirroring SpeedPulser.
    // A resolved speed of 0 must park the needle at rest: applyConfiguredSpeedOffset()
    // adds a positive offset even at 0, so gate on vehicleSpeed to keep the motor off
    // when Speed Test is disabled (or the slider/source reads 0).
    uint32_t baseDuty = (vehicleSpeed == 0) ? 0 : speedToPwmDuty((uint16_t)dutyCycle);

    // Drive calibration (S3, voltage control on), closed-loop PID trim, or plain
    // open-loop write. The drive and the PID run on the fixed 100 ms tick regardless
    // of the 50 ms task period, so the integral maths stay time-consistent.
#if SPP_HAS_VOLTAGE_CTRL
    if (voltageMode)
    {
      // Open-loop drive from the calibration marks (+ the optional slow feedback
      // trim). Runs even at target 0 so the motor stops and the rail parks at min.
      if (tick)
      {
        uint16_t vcTarget = (vehicleSpeed == 0) ? 0 : (uint16_t)dutyCycle;
        applyDriveControl(vcTarget);
      }
    }
    else
#endif
    if (pidActive)
    {
      if (tick)
      {
        int16_t trimmedDuty = applyFeedbackTrim((uint16_t)dutyCycle, (uint16_t)baseDuty);
        setMotorDutyRaw((uint32_t)trimmedDuty);
      }
    }
    else
    {
      setMotorDutyRaw(baseDuty);
    }

    // Delay before next processing
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
    DEBUG_SPD("vehicle=%u  hall=%u  can=%u  gps=%u  dsg=%u  duty=%ld",
              vehicleSpeed, vehicleSpeedHall, vehicleSpeedCAN,
              vehicleSpeedGPS, dsgSpeed, dutyCycle);
    DEBUG_RPM("vehicleRPM=%u  hallRPM=%u  canRPM=%u  freqRPM=%ld",
              vehicleRPM, vehicleRPMHall, vehicleRPMCAN, frequencyRPM);
    DEBUG_FB("enable=%d  measured=%u kph  freq=%.1f Hz  correction=%d",
             feedbackEnable ? 1 : 0, measuredSpeed, measuredFreqHz, pidCorrection);
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

void setBroadcastSpeedTaskEnabled(bool /*enabled*/)
{
  // No-op: the task runs continuously and checks broadcastSpeedEnabled each cycle.
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

/**
 * Clean up and delete all tasks
 * Called at shutdown
 */
void tasksCleanup()
{
  if (taskEEPHandle != NULL)
  {
    vTaskDelete(taskEEPHandle);
    taskEEPHandle = NULL;
  }
  if (taskUIUpdateHandle != NULL)
  {
    vTaskDelete(taskUIUpdateHandle);
    taskUIUpdateHandle = NULL;
  }
  if (taskGPSHandle != NULL)
  {
    vTaskDelete(taskGPSHandle);
    taskGPSHandle = NULL;
  }
  if (taskDSGHandle != NULL)
  {
    vTaskDelete(taskDSGHandle);
    taskDSGHandle = NULL;
  }
  if (taskSpeedHandle != NULL)
  {
    vTaskDelete(taskSpeedHandle);
    taskSpeedHandle = NULL;
  }
  if (taskRPMHandle != NULL)
  {
    vTaskDelete(taskRPMHandle);
    taskRPMHandle = NULL;
  }
  if (taskBroadcastSpeedHandle != NULL)
  {
    vTaskDelete(taskBroadcastSpeedHandle);
    taskBroadcastSpeedHandle = NULL;
  }
  if (taskDiagHandle != NULL)
  {
    vTaskDelete(taskDiagHandle);
    taskDiagHandle = NULL;
  }
  DEBUG("All tasks cleaned up");
}
