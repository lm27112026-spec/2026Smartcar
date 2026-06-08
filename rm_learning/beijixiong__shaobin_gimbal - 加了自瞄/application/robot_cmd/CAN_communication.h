#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include "CAN_cmd_damiao.h"
#include "CAN_cmd_dji.h"
#include "CAN_receive.h"

extern void CanSendRcDataToBoard(uint8_t can, uint16_t target_id, uint16_t index);
extern void CanSendGimbalDataToBoard(uint8_t can, uint16_t target_id, uint16_t index);

#endif
