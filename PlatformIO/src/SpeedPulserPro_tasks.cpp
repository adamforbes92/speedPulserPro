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
  unsigned long lastIncomingHallHz = 0;
  uint16_t lastValidVehicleSpeed = 0;
  TickType_t lastPidTick = xTaskGetTickCount();  // closed-loop PID cadence
  TickType_t lastMeasTick = xTaskGetTickCount(); // open-loop tacho readout cadence
  TickType_t motorRunSince = 0;                  // when the motor started running with no feedback yet

  while (1)
  {
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
      if ((millis() + 10 - lastPulse) > durationReset)
      {
        dutyCycle = 0;
        dutyCycleIncoming = 0;
        vehicleSpeedHall = 0;
        hallSpeed = 0;
        currentSpeedOffset = 0;
        lastIncomingHallHz = 0;
        resetHallMedianFilter();
      }

      if (lastIncomingHallHz != dutyCycleIncoming)
      {
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

        lastIncomingHallHz = dutyCycleIncoming;
      }
    }

    // Calibration mode drives the raw 12-bit hardware duty directly (0..PWM_DUTY_MAX)
    if (testCal)
    {
      dutyCycle = constrain(tempDutyCycle, 0, (long)PWM_DUTY_MAX);
      setMotorDutyRaw((uint32_t)dutyCycle);
      // Keep the tacho readout live so measured speed shows and feedbackAvailable
      // can latch while jogging the motor to build a calibration.
      if ((xTaskGetTickCount() - lastMeasTick) >= pdMS_TO_TICKS(100))
      {
        lastMeasTick = xTaskGetTickCount();
        updateMeasuredFreq();
      }
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
      if (useHall)
      {
        rawVehicleSpeed = hallSpeed;
      }
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

#if enableDebug && debugSpeed
    if (vehicleSpeed > 0)
    {
      DEBUG_SPD("vehicleSpeed=%d km/h  dutyCycle=%ld", vehicleSpeed, dutyCycle);
    }
#endif

    // Feed-forward base duty: interpolate the requested speed to a 12-bit hardware
    // duty (finer than the raw calibration grid), mirroring SpeedPulser.
    uint32_t baseDuty = speedToPwmDuty((uint16_t)dutyCycle);

    // Closed-loop PID trim, or plain open-loop write.
    // The PID and the tacho readout run on a fixed 100 ms cadence regardless of the
    // 50 ms task period, so the derivative/integral maths stay time-consistent.
    if (pidActive)
    {
      if ((xTaskGetTickCount() - lastPidTick) >= pdMS_TO_TICKS(100))
      {
        lastPidTick = xTaskGetTickCount();
        int16_t trimmedDuty = applyFeedbackTrim((uint16_t)dutyCycle, (uint16_t)baseDuty);
        setMotorDutyRaw((uint32_t)trimmedDuty);
      }
    }
    else
    {
      setMotorDutyRaw(baseDuty);
      // Keep the measured-speed/tacho readout live even when the loop is off (or
      // feedback isn't available yet, so it can latch as soon as a signal appears).
      if ((xTaskGetTickCount() - lastMeasTick) >= pdMS_TO_TICKS(100))
      {
        lastMeasTick = xTaskGetTickCount();
        updateMeasuredFreq();
      }
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
    // Always compute Hall RPM for live display, but clear stale data after timeout
    if ((millis() + 10 - lastPulseRPM) > durationReset)
    {
      dutyCycleMotor = 0;
      vehicleRPMHall = 0;
    }
    else
    {
      vehicleRPMHall = map(dutyCycleMotor, 0, maxRPM, 0, clusterRPMLimit);
    }

    // Handle RPM input - test mode or selected live source
    if (testRPM)
    {
      vehicleRPM = tempRPM;
    }
    else
    {
      if (useRPMCAN)
      {
        vehicleRPM = vehicleRPMCAN;
      }
      else
      {
        if ((millis() + 10 - lastPulseRPM) > durationReset)
        {
          vehicleRPM = 0;
          vehicleRPMHall = 0;
          resetRPMMedianFilter();
        }
        else
        {
          if (averageFilterRPM <= 1)
          {
            filteredRPM = vehicleRPMHall;
            rawCountRPM = 0;
            samplesRPM.clear();
          }
          else
          {
            if (rawCountRPM < averageFilterRPM)
            {
              samplesRPM.add(vehicleRPMHall);
              rawCountRPM++;
            }

            if (rawCountRPM >= averageFilterRPM)
            {
              filteredRPM = (uint16_t)samplesRPM.getMedian();
              rawCountRPM = 0;
              samplesRPM.clear();
            }
          }

          vehicleRPM = filteredRPM;
        }
      }
    }

    // Clamp RPM so both Hall and CAN use the same configured cluster limit
    vehicleRPM = constrain(vehicleRPM, 0, clusterRPMLimit);

    // Map RPM to PWM frequency and output — only drive coil when coilType is enabled.
    // Only reprogram the LEDC timer when the target frequency actually changes:
    // ledc_set_freq() reconfigures the timer inside a critical section, so
    // skipping redundant calls keeps interrupt-masking to a minimum.
    static long lastAppliedFreqRPM = -1;
    long targetFreqRPM = coilType ? map(vehicleRPM, 0, clusterRPMLimit, 0, maxRPM) : 0;
    if (targetFreqRPM != lastAppliedFreqRPM)
    {
      setFrequencyRPM(targetFreqRPM);
      lastAppliedFreqRPM = targetFreqRPM;
    }
    frequencyRPM = targetFreqRPM;

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
