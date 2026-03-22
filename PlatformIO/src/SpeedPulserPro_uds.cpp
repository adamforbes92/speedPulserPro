#include "SpeedPulserPro_uds.h"
#include "SpeedPulserPro_globals.h"

// TP2.0 frame buffer for multi-frame reception
static TP2Frame tp2RxFrame = {0};
static unsigned long lastUDSRequest = 0;
static const unsigned long UDS_REQUEST_INTERVAL = 1000;  // Request every 1 second

/**
 * Send UDS request to read wheel speed from DSG/gearbox
 * Sends a ReadDataByIdentifier (0x22) request for wheel speed DID
 */
void udsRequestWheelSpeed() {
  // Rate limit UDS requests
  if (millis() - lastUDSRequest < UDS_REQUEST_INTERVAL) {
    return;
  }
  lastUDSRequest = millis();

  twai_message_t udsRequest = {};
  udsRequest.identifier = UDS_TX_ID;
  udsRequest.data_length_code = 3;
  
  // Single frame TP2.0 header (data length=3)
  udsRequest.data[0] = 0x03;  // TP2.0 single frame with 3 bytes
  
  // UDS ReadDataByIdentifier (0x22) service
  udsRequest.data[1] = UDS_READ_DATA_BY_ID;
  
  // Data Identifier (F190 = Wheel Speed)
  udsRequest.data[2] = (DID_WHEEL_SPEED >> 8) & 0xFF;  // F1
  udsRequest.data[3] = DID_WHEEL_SPEED & 0xFF;         // 90
  
  // Send the request
  if (twai_transmit(&udsRequest, pdMS_TO_TICKS(10)) != ESP_OK) {
#ifdef ChassisCANDebug
    Serial.println("[UDS] Failed to send wheel speed request");
#endif
  }
}

/**
 * Process incoming UDS/TP2.0 messages
 * Handles multi-frame reception and extracts wheel speed data
 */
void processUDSMessage(const twai_message_t& frame) {
  if (frame.identifier != UDS_RX_ID) {
    return;  // Not a UDS response
  }

  uint8_t pciType = (frame.data[0] >> 4) & 0x0F;
  
  switch (pciType) {
    case TP2_SINGLE_FRAME: {
      // Single frame response
      uint8_t dataLength = frame.data[0] & 0x0F;
      if (dataLength >= 4) {  // Min: SID(1) + DID(2) + Speed(1)
        udsSpeed = parseWheelSpeedFromResponse(&frame.data[1], dataLength);
#ifdef ChassisCANDebug
        Serial.print("[UDS] Wheel Speed (Single Frame): ");
        Serial.println(udsSpeed);
#endif
      }
      break;
    }
    
    case TP2_FIRST_FRAME: {
      // First frame of multi-frame response
      tp2RxFrame.expectedLength = ((frame.data[0] & 0x0F) << 8) | frame.data[1];
      tp2RxFrame.receivedLength = 0;
      tp2RxFrame.inProgress = true;
      tp2RxFrame.lastFrameTime = millis();
      
      // Copy first frame data (6 bytes available)
      memcpy(tp2RxFrame.buffer, &frame.data[2], 6);
      tp2RxFrame.receivedLength = 6;
#ifdef ChassisCANDebug
      Serial.print("[UDS] First frame received, total length: ");
      Serial.println(tp2RxFrame.expectedLength);
#endif
      break;
    }
    
    case TP2_CONSECUTIVE_FRAME: {
      // Consecutive frame in multi-frame sequence
      if (!tp2RxFrame.inProgress) {
        return;
      }
      
      // Check timeout (5 seconds)
      if (millis() - tp2RxFrame.lastFrameTime > 5000) {
        tp2RxFrame.inProgress = false;
#ifdef ChassisCANDebug
        Serial.println("[UDS] Multi-frame timeout");
#endif
        return;
      }
      
      uint8_t dataLength = 7;  // Consecutive frames have 7 bytes data
      uint8_t bytesNeeded = tp2RxFrame.expectedLength - tp2RxFrame.receivedLength;
      if (bytesNeeded < 7) {
        dataLength = bytesNeeded;
      }
      
      // Copy consecutive frame data
      memcpy(&tp2RxFrame.buffer[tp2RxFrame.receivedLength], &frame.data[1], dataLength);
      tp2RxFrame.receivedLength += dataLength;
      tp2RxFrame.lastFrameTime = millis();
      
      // Check if reception is complete
      if (tp2RxFrame.receivedLength >= tp2RxFrame.expectedLength) {
        udsSpeed = parseWheelSpeedFromResponse(tp2RxFrame.buffer, tp2RxFrame.receivedLength);
        tp2RxFrame.inProgress = false;
#ifdef ChassisCANDebug
        Serial.print("[UDS] Wheel Speed (Multi-Frame): ");
        Serial.println(udsSpeed);
#endif
      }
      break;
    }
  }
}

/**
 * Parse wheel speed from UDS response data
 * Extracts the speed value from the payload
 * Format typically: ServiceID(1) + DID(2) + SpeedHi(1) + SpeedLo(1) ...
 */
uint16_t parseWheelSpeedFromResponse(const uint8_t* data, uint8_t length) {
  if (length < 4) {
    return 0;  // Invalid response
  }
  
  // Check if this is a positive response (0x62 = ReadDataByIdentifier response)
  if (data[0] != (UDS_RESPONSE_READ_DATA)) {
    return 0;
  }
  
  // Extract DID (should be F190)
  uint16_t did = ((uint16_t)data[1] << 8) | data[2];
  if (did != DID_WHEEL_SPEED) {
    return 0;  // Wrong DID
  }
  
  // Extract speed value (typically 2 bytes in 0.01 km/h units)
  // Some VW systems use: SpeedHi * 256 + SpeedLo, then divide by 100
  uint16_t speedRaw = ((uint16_t)data[3] << 8) | data[4];
  uint16_t speed = speedRaw / 100;  // Convert to km/h
  
  return speed;
}

/**
 * Send a CAN frame via TP2.0 protocol
 * Simple wrapper for transmitting formatted frames
 */
void sendTPFrame(twai_message_t& msg) {
  if (twai_transmit(&msg, pdMS_TO_TICKS(10)) != ESP_OK) {
#ifdef ChassisCANDebug
    Serial.println("[UDS] Failed to send TP frame");
#endif
  }
}
