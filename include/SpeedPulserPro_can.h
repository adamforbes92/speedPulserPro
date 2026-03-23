#ifndef CAN_H
#define CAN_H

#include "SpeedPulserPro_config.h"
#include <driver/twai.h>

void canInit();
void taskCANRx(void *parameter);
void processTWAIMessage(const twai_message_t& frame);
void sendBroadcastSpeedFrame();

#endif // CAN_H
