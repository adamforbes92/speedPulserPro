#include "SpeedPulserPro_can.h"
#include "SpeedPulserPro_uds.h"
#include "SpeedPulserPro_savvycan.h"
#include <driver/twai.h>

// TWAI configuration constants
#define TWAI_RX_QUEUE_LEN 256
#define TWAI_TX_QUEUE_LEN 16

/**
 * Initialize TWAI (CAN bus) driver
 * Configures pins, baud rate, and starts the driver
 */
void canInit()
{
#if ChassisCANDebug
  Serial.println("[CAN] Initializing TWAI driver...");
#endif

  // General configuration (pins, TX/RX queues)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)pinTX_CAN, (gpio_num_t)pinRX_CAN, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = TWAI_RX_QUEUE_LEN;
  g_config.tx_queue_len = TWAI_TX_QUEUE_LEN;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  // Filter configuration - accept all messages
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
  if (ret != ESP_OK)
  {
#if ChassisCANDebug
    Serial.println("[CAN] Failed to install TWAI driver");
#endif
    return;
  }

  // Start TWAI driver
  ret = twai_start();
  if (ret != ESP_OK)
  {
#if ChassisCANDebug
    Serial.println("[CAN] Failed to start TWAI driver");
#endif
    return;
  }

#if ChassisCANDebug
  Serial.println("[CAN] TWAI driver initialized successfully");
#endif

  // Create CAN receive task
  xTaskCreate(
      taskCANRx,       // Task function
      "TWAI_RX",       // Task name
      4096,            // Stack size
      NULL,            // Parameters
      3,               // Priority
      &taskCANRxHandle // Task handle
  );
}

/**
 * FreeRTOS task for receiving CAN messages
 * Runs continuously and processes incoming TWAI frames
 */
void taskCANRx(void *parameter)
{
#if ChassisCANDebug
  Serial.println("[CAN] CAN receive task started");
#endif

  twai_message_t frame;

  while (1)
  {
    // Wait for message with timeout
    esp_err_t ret = twai_receive(&frame, pdMS_TO_TICKS(10));

    if (ret == ESP_OK)
    {
      // Message received - process it
      processTWAIMessage(frame);

      // Forward to SavvyCAN analyzer if active (single bus = 0)
      analyzerQueueFrame(frame, 0);

      // Route frames to protocol task queues
      if (tp20RxQueue &&
          (frame.identifier == TP20_DSG_SETUP_RX ||
           frame.identifier == TP20_RX_ID)) {
        xQueueSendToBack(tp20RxQueue, &frame, 0);
      }
      if (udsRxQueue && frame.identifier == UDS_RX_ID) {
        xQueueSendToBack(udsRxQueue, &frame, 0);
      }
    }

    // Give other tasks a chance to run
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/**
 * Process an incoming TWAI message
 * Extracts CAN ID and data buffer, updates global variables
 */
void processTWAIMessage(const twai_message_t &frame)
{
#if ChassisCANDebug
  Serial.print("[CAN RX] Length Recv: ");
  Serial.print(frame.data_length_code);
  Serial.print(" CAN ID: ");
  Serial.print(frame.identifier, HEX);
  Serial.print(" Buffer: ");
  for (uint8_t i = 0; i < frame.data_length_code; i++)
  {
    Serial.print(frame.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif

  lastCAN = millis();

  // Extract data from TWAI message
  uint32_t id = frame.identifier;
  uint8_t len = frame.data_length_code;
  uint8_t *buf = (uint8_t *)frame.data;

  switch (id)
  {
  case MOTOR1_ID:
    vehicleRPMCAN = ((buf[3] << 8) | buf[2]) * 0.25;
    break;

  case MOTOR2_ID:
    ecuSpeed = (buf[3] * 100 * 128) / 10000;
    break;

  case MOTOR5_ID:
    vehicleEML = bitRead(buf[1], 5);
    vehicleEPC = bitRead(buf[1], 6);
    break;

  case MOTOR6_ID:
    if (buf[0] == 0x73 || buf[0] == 0x72)
    {
      // vehicleReverse = true;  // managed by mWaehlhebel_1_ID / gearLever_ID
    }
    else
    {
      // vehicleReverse = false; // managed by mWaehlhebel_1_ID / gearLever_ID
    }
    if (buf[0] == 0x83 || buf[0] == 0x82)
    {
      vehiclePark = true;
    }
    else
    {
      vehiclePark = false;
    }
    break;

  case BRAKES3_ID:
  {
    const uint16_t br3_speed_raw = (((uint16_t)buf[1] << 8) | buf[0]) >> 1;
    absSpeed = (uint16_t)(br3_speed_raw * 0.01f + 0.5f);
  }
    break;

  case mWaehlhebel_1_ID:
    gear_raw = ((buf[7] & 0b01110000) >> 4) - 1;
    lever_raw = (buf[7] & 0b00000001);

    if (lever_raw)
    {
      gear = gear_raw;

      switch (gear)
      {
      case 3:
        vehicleReverse = true;
        break;
      default:
        vehicleReverse = false;
        break;
      }

      if (gear == 0xFF)
      {
        gear = 1;
      }
    }
    break;

  case gearLever_ID:
    lever = (buf[0] & 0b11110000) >> 4;
    break;

  case emeraldECU1_ID:
    vehicleRPM = ((buf[0] << 8) | buf[1]);
    break;

  case emeraldECU2_ID:
    ecuSpeed = (uint16_t)(((buf[2] << 8) | buf[3]) * (2.25f / 256.0f) + 0.5f);
    break;

  default:
    break;
  }

  // Aftermarket / Custom CAN speed input
  if (useAftermarket && id == (aftermarketSpeedID & 0x7FF)) {
    uint8_t lowIdx  = constrain(aftermarketSpeedLowByte,  0, 7);
    uint8_t highIdx = constrain(aftermarketSpeedHighByte, 0, 7);
    uint16_t rawValue;
    if (aftermarketSpeedLittleEndian) {
      rawValue = (uint16_t)buf[lowIdx] | ((uint16_t)buf[highIdx] << 8);
    } else {
      rawValue = (uint16_t)buf[highIdx] | ((uint16_t)buf[lowIdx] << 8);
    }
    float scaled = (rawValue * aftermarketSpeedScale) + aftermarketSpeedOffset;
    aftermarketSpeed = (uint16_t)constrain((int32_t)(scaled + 0.5f), 0, 65535);
  }
}

/**
 * Send vehicle speed over CAN at a configurable frame format.
 * All frame fields (ID, DLC, byte indices, endianness, scale, offset,
 * template bytes) are runtime-configurable via the UI.
 */
void sendBroadcastSpeedFrame()
{
  if (!broadcastSpeedEnabled)
    return;

  twai_message_t speedFrame{};
  speedFrame.identifier = broadcastSpeedID & 0x7FF;
  speedFrame.extd = 0;
  speedFrame.data_length_code = constrain(broadcastSpeedDLC, 0, 8);

  for (uint8_t i = 0; i < 8; i++)
  {
    speedFrame.data[i] = broadcastSpeedData[i];
  }

  int32_t scaledSpeed = static_cast<int32_t>((vehicleSpeed * broadcastSpeedScale) + broadcastSpeedOffset);
  scaledSpeed = constrain(scaledSpeed, 0, 65535);
  broadcastSpeedValue = static_cast<uint16_t>(scaledSpeed);

  uint8_t lowByteIndex = constrain(broadcastSpeedLowByte, 0, 7);
  uint8_t highByteIndex = constrain(broadcastSpeedHighByte, 0, 7);
  uint8_t lowByte = static_cast<uint8_t>(broadcastSpeedValue & 0xFF);
  uint8_t highByte = static_cast<uint8_t>((broadcastSpeedValue >> 8) & 0xFF);

  if (broadcastSpeedLittleEndian)
  {
    speedFrame.data[lowByteIndex] = lowByte;
    speedFrame.data[highByteIndex] = highByte;
  }
  else
  {
    speedFrame.data[lowByteIndex] = highByte;
    speedFrame.data[highByteIndex] = lowByte;
  }

#if ChassisCANDebug
  Serial.printf("[CAN TX] ID:0x%03X DLC:%u Data:", speedFrame.identifier, speedFrame.data_length_code);
  for (uint8_t i = 0; i < speedFrame.data_length_code; i++) {
    Serial.printf(" %02X", speedFrame.data[i]);
  }
  Serial.printf("  Value:%u (vehicleSpeed:%u)\n", broadcastSpeedValue, vehicleSpeed);
#endif

  if (twai_transmit(&speedFrame, pdMS_TO_TICKS(10)) != ESP_OK)
  {
    // frame dropped if TX queue is full — not fatal
  }
}