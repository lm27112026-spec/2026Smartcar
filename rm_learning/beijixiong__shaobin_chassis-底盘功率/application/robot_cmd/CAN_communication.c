#include "CAN_communication.h"

#include "bsp_can.h"
#include "can_typedef.h"
#include "string.h"
#include "user_lib.h"
#include "referee.h"

static CanCtrlData_s CAN_CTRL_DATA = {
	.tx_header.IDE = CAN_ID_STD,
	.tx_header.RTR = CAN_RTR_DATA,
	.tx_header.DLC = 8,
};

static CanBoardCommunicate_t SEND_CBC;

/*-------------------- Private functions --------------------*/
// 板间通信

/**
 * @brief          发送数据
 * @param[in]      hcan CAN句柄
 * @param[in]      std_id 数据包ID
 * @param[in]      data 包含8个字节的数据的指针
 * @retval         none
 */
static void SendData(uint8_t can, uint16_t std_id, uint8_t * data)
{
	if(can == 1)
		CAN_CTRL_DATA.hcan = &hcan1;
	else if(can == 2)
		CAN_CTRL_DATA.hcan = &hcan2;
	else 
		return;
	
	CAN_CTRL_DATA.tx_header.StdId = std_id;
	
	memcpy(CAN_CTRL_DATA.tx_data, data, 8);
	
	CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/*-------------------- Public functions --------------------*/

void CanSendRcDataToBoard(uint8_t can, uint16_t target_id,uint16_t index)
{
	uint16_t std_id = CAN_STD_ID_PACK_BASE | CAN_STD_ID_Rc << TYPE_ID_OFFSET |
					  (target_id << TARGET_ID_OFFSET) | index;
	
	const RC_ctrl_t * rc_ctrl = get_remote_control_point();
	
	if(index == 0)
	{
		// 发送遥控器数据
		bool offline = GetRcOffline();
		uint16_t ch[5];
        ch[0] = rc_ctrl->rc.ch[0] + RC_CH_VALUE_OFFSET;
        ch[1] = rc_ctrl->rc.ch[1] + RC_CH_VALUE_OFFSET;
        ch[2] = rc_ctrl->rc.ch[2] + RC_CH_VALUE_OFFSET;
        ch[3] = rc_ctrl->rc.ch[3] + RC_CH_VALUE_OFFSET;
        ch[4] = rc_ctrl->rc.ch[4] + RC_CH_VALUE_OFFSET;
		
		SEND_CBC.rc_data.rc.packed.ch0 = ch[0];
        SEND_CBC.rc_data.rc.packed.ch1 = ch[1];
        SEND_CBC.rc_data.rc.packed.ch2 = ch[2];
        SEND_CBC.rc_data.rc.packed.ch3 = ch[3];
        SEND_CBC.rc_data.rc.packed.ch4 = ch[4];
        SEND_CBC.rc_data.rc.packed.s0 = rc_ctrl->rc.s[0];
        SEND_CBC.rc_data.rc.packed.s1 = rc_ctrl->rc.s[1];
        SEND_CBC.rc_data.rc.packed.offline = offline;
		
		SendData(can, std_id, SEND_CBC.rc_data.rc.raw.data);
	}
	else if(index == 1)
	{
		// 键鼠数据（经过压缩）
		SEND_CBC.rc_data.km.packed.mouse_x = rc_ctrl->mouse.x >> 1;
        SEND_CBC.rc_data.km.packed.mouse_y = rc_ctrl->mouse.y >> 1;
        SEND_CBC.rc_data.km.packed.mouse_z = rc_ctrl->mouse.z;
        SEND_CBC.rc_data.km.packed.mouse_press_l = rc_ctrl->mouse.press_l;
        SEND_CBC.rc_data.km.packed.mouse_press_r = rc_ctrl->mouse.press_r;
        SEND_CBC.rc_data.km.packed.key = rc_ctrl->key.v;
		
		SendData(can, std_id, SEND_CBC.rc_data.km.raw.data);
	}
}

void CanSendGimbalDataToBoard(uint8_t can, uint16_t target_id, uint16_t index)
{
//	uint16_t std_id = CAN_STD_ID_PACK_BASE | CAN_STD_ID_Gimbal << TYPE_ID_OFFSET |
//					  (target_id << TARGET_ID_OFFSET) | index;
	
//	float delta_yaw_mid = GetGimbalDeltaYawMid();

}

void CanSendRefereeDataToBoard(uint8_t can, uint16_t target_id, uint16_t index)
{
    // 根据你的通信协议打包标准帧ID
    uint16_t std_id = CAN_STD_ID_PACK_BASE | CAN_STD_ID_Referee << TYPE_ID_OFFSET |
                      (target_id << TARGET_ID_OFFSET) | index;

    uint16_t heat_limit = 0;
    uint16_t heat = 0;

    if (index == 0) 
    {
        // 调取裁判系统解析好的热量接口（这里以你的heat0为例）
        get_shoot_heat0_limit_and_heat0(&heat_limit, &heat);
        
        // 装填数据
        SEND_CBC.referee_data.raw.packed.heat_limit = heat_limit;
        SEND_CBC.referee_data.raw.packed.heat = heat;

        // 发送CAN包
        SendData(can, std_id, SEND_CBC.referee_data.raw.data);
    }
}




