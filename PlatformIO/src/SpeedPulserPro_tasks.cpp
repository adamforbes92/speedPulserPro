#include "SpeedPulserPro_tasks.h"
#include "SpeedPulserPro_config.h"
#include "SpeedPulserPro_eep.h"
#include "SpeedPulserPro_wifi.h"
#include "SpeedPulserPro_gps.h"
#include "SpeedPulserPro_dsg.h"
#include "SpeedPulserPro_control.h"
#include "SpeedPulserPro_can.h"
#include "SpeedPulserPro_uds.h"

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

/**
 * Task: Write EEPROM/Preferences at regular intervals
 * Called every DELAY_EEPROM milliseconds (5000ms = 5 seconds)
 */
void taskWriteEEP(void *parameter)
{
  DEBUG_PRINTLN("[TASK] EEP: Starting EEPROM write task");

  while (1)
  {
    // Write current settings to EEPROM
    writeEEP();

#if serialDebugEEP
    DEBUG_PRINTLN("[TASK] EEP: Settings written to EEPROM");
#endif

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
  DEBUG_PRINTLN("[TASK] UI: Starting UI update task");

  while (1)
  {
    // Update WiFi interface labels
    updateLabels();

#if serialDebugWifi
    DEBUG_PRINTLN("[TASK] UI: Labels updated");
#endif

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
#if serialDebugGPS
  DEBUG_PRINTLN("[TASK] GPS: Starting GPS parse task");
#endif

  while (1)
  {
    // Parse incoming GPS data
    parseGPS();

#if serialDebugGPS
    if (gps.speed.isUpdated())
    {
      DEBUG_PRINTF("[TASK] GPS: Speed updated to %d km/h\n", (int)gpsSpeed);
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
  DEBUG_PRINTLN("[TASK] DSG: Starting DSG parse task");

  while (1)
  {
    // Calculate DSG speed from RPM and gear ratio
    parseDSG();

#if serialDebugIncoming
    if (dsgSpeed > 0)
    {
      DEBUG_PRINTF("[TASK] DSG: Speed calculated to %u km/h, Gear: %d\n", dsgSpeed, gear);
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
  DEBUG_PRINTLN("[TASK] Speed: Starting speed processing task");
  unsigned long lastIncomingHallHz = 0;
  uint16_t lastValidVehicleSpeed = 0;

  while (1)
  {
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

    // Calibration mode drives direct duty output (0..385)
    if (testCal)
    {
      dutyCycle = constrain(tempDutyCycle, 0, 385);
      ledcWrite(LEDC_OUTPUT_CHANNEL, dutyCycle);
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

#if serialDebugIncoming
    if (vehicleSpeed > 0)
    {
      DEBUG_PRINTF("[TASK] Speed: vehicleSpeed=%d, dutyCycle=%d\n", vehicleSpeed, dutyCycle);
    }
#endif

    // Apply motor calibration lookup and write to PWM output
    uint16_t calibratedDuty = findClosestMatch(dutyCycle);
    ledcWrite(LEDC_OUTPUT_CHANNEL, calibratedDuty);

    // Delay before next processing
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/**
 * Task: Process RPM from selected source
 * Called every DELAY_RPM milliseconds (50ms)
 * Handles Hall sensor RPM, CAN RPM, test mode, and PWM output mapping
 */
void taskProcessRPM(void *parameter)
{
  DEBUG_PRINTLN("[TASK] RPM: Starting RPM processing task");

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

    // Map RPM to PWM frequency and output — only drive coil when coilType is enabled
    if (coilType)
    {
      frequencyRPM = map(vehicleRPM, 0, clusterRPMLimit, 0, maxRPM);
      setFrequencyRPM(frequencyRPM);
    }
    else
    {
      setFrequencyRPM(0);
      frequencyRPM = 0;
    }

#if serialDebugIncoming
    if (vehicleRPM > 0)
    {
      DEBUG_PRINTF("[TASK] RPM: vehicleRPM=%d, frequencyRPM=%d\n", vehicleRPM, frequencyRPM);
    }
#endif

    // Delay before next processing
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/**
 * Task: Broadcast final vehicle speed over CAN at 20ms intervals.
 */
void taskBroadcastSpeed(void *parameter)
{
  DEBUG_PRINTLN("[TASK] Broadcast: Starting speed broadcast task");

  while (1)
  {
    sendBroadcastSpeedFrame();
    vTaskDelay(pdMS_TO_TICKS(DELAY_BROADCAST_SPEED));
  }
}

/**
 * Initialize all FreeRTOS tasks
 * Called from setup()
 */
void tasksInit()
{
  DEBUG_PRINTLN("[TASK] Initializing FreeRTOS tasks...");

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
    DEBUG_PRINTLN("[TASK] ERROR: Failed to create EEPROM task");
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
    DEBUG_PRINTLN("[TASK] ERROR: Failed to create UI update task");
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
    DEBUG_PRINTLN("[TASK] ERROR: Failed to create GPS task");
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
    DEBUG_PRINTLN("[TASK] ERROR: Failed to create DSG task");
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
    DEBUG_PRINTLN("[TASK] ERROR: Failed to create Speed task");
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
    DEBUG_PRINTLN("[TASK] ERROR: Failed to create RPM task");
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
    DEBUG_PRINTLN("[TASK] ERROR: Failed to create Broadcast Speed task");
  }

  DEBUG_PRINTLN("[TASK] All tasks initialized successfully");

  status = xTaskCreate(
      taskTP20,
      "TaskTP20",
      4096,
      NULL,
      2,
      &taskTP20Handle);
  if (status != pdPASS)
  {
    DEBUG_PRINTLN("[TASK] ERROR: Failed to create TP2.0 task");
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
    DEBUG_PRINTLN("[TASK] ERROR: Failed to create UDS task");
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
  DEBUG_PRINTLN("[TASK] All tasks suspended");
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
  DEBUG_PRINTLN("[TASK] All tasks resumed");
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
  DEBUG_PRINTLN("[TASK] All tasks cleaned up");
}
