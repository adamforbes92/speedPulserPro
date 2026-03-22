#ifndef UDS_H
#define UDS_H

#include "SpeedPulserPro_config.h"
#include <driver/twai.h>

// UDS Service IDs
#define UDS_READ_DATA_BY_ID 0x22
#define UDS_RESPONSE_READ_DATA (UDS_READ_DATA_BY_ID + 0x40)

// TP2.0 Protocol constants
#define TP2_SINGLE_FRAME 0x00
#define TP2_FIRST_FRAME 0x10
#define TP2_CONSECUTIVE_FRAME 0x20
#define TP2_FLOW_CONTROL 0x30

// DSG/Gearbox UDS request/response IDs
#define UDS_TX_ID 0x6F1          // Transmission request
#define UDS_RX_ID 0x6F9          // Transmission response

// Data Identifiers for wheelchair speed
#define DID_WHEEL_SPEED 0xF190    // Typical VW DID for wheel speed

// Structure for TP2.0 multi-frame handling
typedef struct {
  uint32_t expectedLength;
  uint32_t receivedLength;
  uint8_t buffer[256];
  uint32_t lastFrameTime;
  bool inProgress;
} TP2Frame;

// Function declarations
void udsRequestWheelSpeed();
void processUDSMessage(const twai_message_t& frame);
uint16_t parseWheelSpeedFromResponse(const uint8_t* data, uint8_t length);
void sendTPFrame(twai_message_t& msg);

#endif // UDS_H
