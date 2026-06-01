#ifndef UDS_H
#define UDS_H

#include "SpeedPulserPro_config.h"
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ---------------------------------------------------------------------------
// VW TP2.0 (KWP2000 over TP2.0) — DSG module 0x02, MK5/PQ35 platform
// CAN IDs confirmed from VCDS SavvyCAN capture:
//   Broadcast (channel setup) : 0x200  (ESP32 → ECU)
//   DSG setup response         : 0x202  (ECU → ESP32)
//   Tester data channel TX     : 0x760  (ESP32 → DSG, negotiated)
//   DSG data channel TX        : 0x300  (DSG → ESP32, negotiated)
// ---------------------------------------------------------------------------
#define TP20_BROADCAST_ID   0x200U  // Channel-setup broadcast
#define TP20_DSG_MODULE     0x02U   // DSG logical module address
#define TP20_DSG_SETUP_RX   0x202U  // Setup response from DSG (0x200 + module)
#define TP20_TX_ID          0x760U  // ESP32 → DSG data channel (negotiated)
#define TP20_RX_ID          0x300U  // DSG → ESP32 data channel (negotiated)

// KWP2000 services used over TP2.0
#define KWP_START_DIAG_SESSION  0x10U
#define KWP_RDBLI               0x21U  // ReadDataByLocalIdentifier
#define KWP_RDBLI_POS           0x61U  // Positive response
#define KWP_DIAG_MODE_DEV       0x89U  // Development/coding session
#define KWP_MEAS_GROUP          0x01U  // Measuring block group 01

// Speed formula: raw 16-bit (big-endian) after formula-ID byte 0x25
// speed_km_h = raw / 3.0  (confirmed: raw=58 @ ~19 km/h)
#define TP20_SPEED_FORMULA_ID   0x25U
#define TP20_SPEED_SCALE_DIV    3.0f

// ---------------------------------------------------------------------------
// UDS (ISO-TP / ISO 14229) speed polling
// Modelled after OpenHaldex C6 UDS stack (queue-based, task-driven)
// Default IDs and DID — change per vehicle/ECU via config
// ---------------------------------------------------------------------------
#define UDS_TX_ID           0x7DFU  // OBD2/UDS functional broadcast
#define UDS_RX_ID           0x7E8U  // Typical ECU 1 response ID
#define UDS_SPEED_DID       0x0101U // DID for vehicle speed — adjust per ECU

// ISO-TP PCI nibbles
#define ISOTP_SF  0x00U  // Single Frame
#define ISOTP_FF  0x10U  // First Frame
#define ISOTP_CF  0x20U  // Consecutive Frame
#define ISOTP_FC  0x30U  // Flow Control

// UDS Service IDs
#define UDS_SID_SESSION_CTRL  0x10U
#define UDS_SID_TESTER_PRESENT 0x3EU
#define UDS_SID_READ_DID      0x22U
#define UDS_SID_READ_DID_POS  0x62U  // Positive response

// FreeRTOS queues shared between CAN RX task and protocol tasks
extern QueueHandle_t tp20RxQueue;  // 0x202 setup + 0x300 data frames → TP2.0 task
extern QueueHandle_t udsRxQueue;   // UDS_RX_ID frames → UDS task

// FreeRTOS task entry points
void taskTP20(void *arg);
void taskUDS(void *arg);

#endif // UDS_H
