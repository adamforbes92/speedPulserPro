// =============================================================================
// SpeedPulserPro_uds.cpp
//
// Two independent speed-acquisition protocols, each driven by its own
// FreeRTOS task and communicating with taskCANRx via a dedicated queue.
//
// TP2.0 (KWP2000 over VW TP2.0)
//   - Connects to DSG gearbox (module 0x02) on VW MK5 / PQ35 platforms
//   - Protocol confirmed from VCDS SavvyCAN capture (mk5dsgvehiclespeed.csv)
//   - Channel-setup: 0x200 broadcast; data: TX=0x760, RX=0x300
//   - Service: KWP2000 0x21 ReadDataByLocalIdentifier, group 0x01
//   - Speed: raw16 (formula 0x25) / 3 = km/h
//
// UDS (ISO 14229 / ISO-TP)
//   - Models the OpenHaldex C6 UDS task pattern exactly
//   - Extended session + TesterPresent heartbeat + RDBI polling
//   - Configurable TX/RX CAN IDs and DID via SpeedPulserPro_uds.h defines
// =============================================================================

#include "SpeedPulserPro_uds.h"
#include "SpeedPulserPro_globals.h"

// Shared queues (created once; populated by taskCANRx in SpeedPulserPro_can.cpp)
QueueHandle_t tp20RxQueue = nullptr;
QueueHandle_t udsRxQueue  = nullptr;

// =============================================================================
// Helpers
// =============================================================================

static bool canSend(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    twai_message_t msg{};
    msg.identifier       = id;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = dlc;
    memcpy(msg.data, data, dlc);
    return (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK);
}

// =============================================================================
// TP2.0 TASK
// =============================================================================

// Send a KWP2000 payload as a TP2.0 single-data-frame on 0x760.
// Frame format: [0x10|seq, 0x00, len, payload...] padded to 8 bytes.
static bool tp20SendKWP(uint8_t seq, const uint8_t *payload, uint8_t len)
{
    if (len > 5) return false; // single-frame limit for our 8-byte CAN frame
    uint8_t buf[8] = {0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu};
    buf[0] = 0x10u | (seq & 0x0Fu);
    buf[1] = 0x00u;
    buf[2] = len;
    memcpy(&buf[3], payload, len);
    return canSend(TP20_TX_ID, buf, 8);
}

// Drain the TP2.0 RX queue, collecting CAN frames into a KWP payload.
// Returns total KWP bytes assembled, or 0 on timeout/error.
// kwpBuf must be at least 64 bytes.
static uint8_t tp20CollectResponse(uint8_t *kwpBuf, uint8_t kwpBufLen, uint32_t timeoutMs)
{
    uint32_t deadline    = millis() + timeoutMs;
    uint8_t  collected   = 0;
    uint8_t  expectedLen = 0;
    bool     started     = false;

    twai_message_t frame;

    for (;;) {
        uint32_t now = millis();
        if (now >= deadline) break;
        uint32_t remaining = deadline - now;

        if (xQueueReceive(tp20RxQueue, &frame, pdMS_TO_TICKS(remaining < 20u ? remaining : 20u)) != pdTRUE)
            continue;

        if (frame.identifier != TP20_RX_ID) continue;

        uint8_t type = (frame.data[0] >> 4) & 0x0Fu;

        if (type == 0x0Bu) {
            // ACK frame (0xBx) - discard
            continue;
        }

        if (type == 0x02u) {
            if (frame.data[1] == 0x00u && !started) {
                // First data frame: [2x][00][len][payload...]
                expectedLen = frame.data[2];
                uint8_t chunk = (uint8_t)(frame.data_length_code - 3u);
                if (collected + chunk > kwpBufLen) break;
                memcpy(&kwpBuf[collected], &frame.data[3], chunk);
                collected += chunk;
                started = true;
            } else if (started) {
                // Consecutive data frame: [2x][payload...]
                uint8_t chunk = (uint8_t)(frame.data_length_code - 1u);
                if (collected + chunk > kwpBufLen) chunk = kwpBufLen - collected;
                memcpy(&kwpBuf[collected], &frame.data[1], chunk);
                collected += chunk;
            }
            if (started && collected >= expectedLen) break;
            continue;
        }

        if (type == 0x01u) {
            // Last data frame (0x1x): [1x][payload...]
            uint8_t chunk = (uint8_t)(frame.data_length_code - 1u);
            if (!started) {
                if (chunk > kwpBufLen) chunk = kwpBufLen;
                memcpy(kwpBuf, &frame.data[1], chunk);
                collected = chunk;
            } else {
                if (collected + chunk > kwpBufLen) chunk = kwpBufLen - collected;
                memcpy(&kwpBuf[collected], &frame.data[1], chunk);
                collected += chunk;
            }
            break; // last frame always terminates
        }
    }

    return collected;
}

// Perform TP2.0 channel setup. Returns true if DSG acks the channel.
static bool tp20Connect()
{
    xQueueReset(tp20RxQueue);

    // Broadcast channel-setup request to DSG module 0x02.
    // Byte layout confirmed from VCDS SavvyCAN log (mk5dsgvehiclespeed.csv):
    //   [module=0x02][0xC0=request][0x00][0x10]
    //   [RX_id_lo=0x00][RX_id_hi=0x03]  <- ECU will respond on 0x0300 (LE)
    //   [0x01][0x00]
    uint8_t setup[8] = {
        TP20_DSG_MODULE, 0xC0u, 0x00u, 0x10u,
        (uint8_t)(TP20_RX_ID & 0xFFu),
        (uint8_t)(TP20_RX_ID >> 8),
        0x01u, 0x00u
    };
    if (!canSend(TP20_BROADCAST_ID, setup, 8)) return false;

    // Wait for positive response on 0x202 (0x200 + module_addr)
    twai_message_t rsp;
    uint32_t deadline = millis() + 1000u;
    bool gotSetupAck = false;
    while (millis() < deadline) {
        if (xQueueReceive(tp20RxQueue, &rsp, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (rsp.identifier == TP20_DSG_SETUP_RX &&
                rsp.data_length_code >= 2 &&
                rsp.data[1] == 0xD0u) {
                gotSetupAck = true;
                break;
            }
        }
    }
    if (!gotSetupAck) return false;

    // Channel parameter negotiation (0xA0 / 0xA1 exchange)
    // Values observed in VCDS log: 0F 8A FF 32 FF 00 00
    uint8_t chanParam[8] = {0xA0u, 0x0Fu, 0x8Au, 0xFFu, 0x32u, 0xFFu, 0x00u, 0x00u};
    if (!canSend(TP20_TX_ID, chanParam, 8)) return false;

    deadline = millis() + 500u;
    while (millis() < deadline) {
        if (xQueueReceive(tp20RxQueue, &rsp, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (rsp.identifier == TP20_RX_ID &&
                rsp.data_length_code >= 1 &&
                (rsp.data[0] & 0xF0u) == 0xA0u) {
                break; // 0xA1 = channel param ack
            }
        }
    }

    // Open KWP2000 diagnostic session (mode 0x89 = development/extended)
    uint8_t seq = 0;
    const uint8_t sessReq[] = {KWP_START_DIAG_SESSION, KWP_DIAG_MODE_DEV};
    if (!tp20SendKWP(seq++, sessReq, sizeof(sessReq))) return false;

    uint8_t kwpBuf[32];
    uint8_t kwpLen = tp20CollectResponse(kwpBuf, sizeof(kwpBuf), 1000);
    if (kwpLen < 2) return false;
    // Positive response to StartDiagnosticSession: 0x50 0x89
    if (kwpBuf[0] != 0x50u || kwpBuf[1] != KWP_DIAG_MODE_DEV) return false;

    return true;
}

void taskTP20(void *arg)
{
    // Queue receives: 0x202 (channel-setup ack) and 0x300 (data) frames
    // from taskCANRx in SpeedPulserPro_can.cpp
    tp20RxQueue = xQueueCreate(16, sizeof(twai_message_t));

    uint8_t seq = 0;

    while (1) {
        if (!useTP20 || !hasCAN) {
            tp20Speed = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
            seq = 0;
            continue;
        }

        if (!tp20Connect()) {
            tp20Speed = 0;
            vTaskDelay(pdMS_TO_TICKS(2000));
            seq = 0;
            continue;
        }

        // Connected - poll measuring block 01 continuously
        uint32_t lastPollMs = 0;

        while (useTP20 && hasCAN) {
            uint32_t now = millis();
            if ((now - lastPollMs) < 100u) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            lastPollMs = now;

            // KWP2000 ReadDataByLocalIdentifier, measuring group 0x01
            const uint8_t rdReq[] = {KWP_RDBLI, KWP_MEAS_GROUP};
            if (!tp20SendKWP(seq, rdReq, sizeof(rdReq))) break;
            seq = (seq + 1u) & 0x0Fu;

            uint8_t kwpBuf[32];
            uint8_t kwpLen = tp20CollectResponse(kwpBuf, sizeof(kwpBuf), 300);

            // Validate positive response: [0x61][0x01][0x25][hi][lo]...
            if (kwpLen < 5) continue;
            if (kwpBuf[0] != KWP_RDBLI_POS)    continue;
            if (kwpBuf[1] != KWP_MEAS_GROUP)   continue;
            if (kwpBuf[2] != TP20_SPEED_FORMULA_ID) continue;

            uint16_t raw = ((uint16_t)kwpBuf[3] << 8) | kwpBuf[4];
            tp20Speed = (uint16_t)((float)raw / TP20_SPEED_SCALE_DIV + 0.5f);

            DEBUG_UDS("TP2.0 raw=%u  speed=%u km/h", raw, tp20Speed);
        }

        // Lost connection or disabled
        tp20Speed = 0;
        seq = 0;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =============================================================================
// UDS TASK  (OpenHaldex C6 pattern, adapted for vehicle speed reading)
// =============================================================================

static bool udsSendFrame(uint32_t canId, const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen > 7) return false;
    twai_message_t msg{};
    msg.identifier       = canId;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = 8;
    msg.data[0]          = (uint8_t)(ISOTP_SF | payloadLen); // SF PCI
    memcpy(&msg.data[1], payload, payloadLen);
    for (uint8_t i = payloadLen + 1u; i < 8u; i++) msg.data[i] = 0xAAu; // ISO-TP padding
    return (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK);
}

void taskUDS(void *arg)
{
    udsRxQueue = xQueueCreate(8, sizeof(twai_message_t));

    while (1) {
        if (!useUDS || !hasCAN) {
            udsSpeed = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        xQueueReset(udsRxQueue);

        // ── Open extended diagnostic session (DiagnosticSessionControl 0x03) ──
        const uint8_t sessReq[] = {UDS_SID_SESSION_CTRL, 0x03u};
        udsSendFrame(UDS_TX_ID, sessReq, sizeof(sessReq));

        twai_message_t sessResp;
        if (xQueueReceive(udsRxQueue, &sessResp, pdMS_TO_TICKS(2000)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        // Verify positive session response: [SF][0x50][0x03]
        if (sessResp.data_length_code < 3 ||
            sessResp.data[1] != 0x50u ||
            sessResp.data[2] != 0x03u) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // ── Session open: poll speed DID, send TesterPresent every 375 ms ──
        uint32_t lastTP = millis();

        while (useUDS && hasCAN) {
            if ((millis() - lastTP) >= 375u) {
                // TesterPresent with suppress-positive-response bit
                const uint8_t tpReq[] = {UDS_SID_TESTER_PRESENT, 0x80u};
                udsSendFrame(UDS_TX_ID, tpReq, sizeof(tpReq));
                lastTP = millis();
            }

            // ReadDataByIdentifier (0x22) for the configured speed DID
            const uint8_t rdbiReq[] = {
                UDS_SID_READ_DID,
                (uint8_t)(UDS_SPEED_DID >> 8),
                (uint8_t)(UDS_SPEED_DID & 0xFFu)
            };
            udsSendFrame(UDS_TX_ID, rdbiReq, sizeof(rdbiReq));

            // Drain queue within 500 ms, looking for positive 0x62 response
            uint32_t windowEnd = millis() + 500u;
            for (;;) {
                uint32_t now = millis();
                if (now >= windowEnd) break;
                uint32_t remain = windowEnd - now;

                twai_message_t rsp;
                if (xQueueReceive(udsRxQueue, &rsp, pdMS_TO_TICKS(remain < 50u ? remain : 50u)) != pdTRUE)
                    continue;

                // Accept only positive single-frame RDBI responses
                if ((rsp.data[0] & 0xF0u) != ISOTP_SF) continue;
                uint8_t pciLen = rsp.data[0] & 0x0Fu;
                if (pciLen < 4u) continue;
                if (rsp.data[1] != UDS_SID_READ_DID_POS) continue;

                uint16_t respDID = ((uint16_t)rsp.data[2] << 8) | rsp.data[3];
                if (respDID != UDS_SPEED_DID) continue;

                // data[4..] = speed payload.
                // Default: 16-bit big-endian, unit = 0.01 km/h (adjust per ECU).
                if (pciLen >= 5u) {
                    uint32_t rawSpeed = ((uint16_t)rsp.data[4] << 8) | rsp.data[5];
                    udsSpeed = (uint16_t)(rawSpeed / 100u);
                } else {
                    // Single-byte km/h (OBD2-style)
                    udsSpeed = rsp.data[4];
                }

                DEBUG_UDS("DID 0x%04X  speed=%u km/h", respDID, udsSpeed);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(150));
        }

        // Session ended or conditions changed - clear speed
        udsSpeed = 0;
    }
}
