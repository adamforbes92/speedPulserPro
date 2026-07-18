#ifndef SAVVYCAN_H
#define SAVVYCAN_H

#include "SpeedPulserPro_config.h"
#include <driver/twai.h>

// ============================================================================
// SavvyCAN / CAN Analyzer Interface
// ============================================================================
// Implements a CAN analyzer that forwards all received frames to external tools:
//   - WiFi mode: GVRET binary protocol (SavvyCAN) or Lawicel/SLCAN (CANHacker)
//                over a TCP server on port 23.
//   - Serial mode: GVRET binary protocol over USB at 1 Mbaud (SavvyCAN serial).
//
// WiFi and Serial modes are mutually exclusive.
//
// #### Connect SavvyCAN via WiFi (GVRET)
// 1. Connect to the SpeedPulserPro Wi-Fi AP.
// 2. In SavvyCAN: Add New Device Connection -> Network Connection (GVRET).
// 3. IP: 192.168.4.1, port 23 (implicit in SavvyCAN GVRET UI).
//
// #### Connect SavvyCAN via Serial (GVRET)
// 1. Connect USB to the ESP32.
// 2. In SavvyCAN: Add New Device Connection -> GVRET Compatible Device.
// 3. Select the correct COM port; baud rate is set to 1 Mbaud automatically.
//
// #### Analyzer Protocol defines
#define ANALYZER_PROTOCOL_GVRET   0   // SavvyCAN GVRET binary (default)
#define ANALYZER_PROTOCOL_LAWICEL 1   // CANHacker SLCAN/Lawicel text

void setupAnalyzer();
void setAnalyzerMode(bool enable);
void setAnalyzerSerialMode(bool enable);
void analyzerQueueFrame(const twai_message_t &frame, uint8_t bus);

#endif // SAVVYCAN_H
