/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_typedef.c/h
  * @brief      can的板间通信部分的相关定义（CAN_communication.c内部文件，不对外开放）
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-04-08      Penguin         1.done
  *
  @verbatim
  ==============================================================================
  11 位 std id 划分
    base id:   110 0000 0000
    type id:   001 1100 0000
    target id: 000 0011 1000
    index id:  000 0000 0111
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#ifndef CAN_TYPEDEF_H
#define CAN_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"

// clang-format off
#define BASE_ID_OFFSET 9
#define TYPE_ID_OFFSET 6
#define TARGET_ID_OFFSET 3

// CAN通信协议相关定义
// 固定类型的数据包
#define CAN_STD_ID_PACK_BASE     ((uint16_t) 0x0400)

#define CAN_STD_ID_Test     ((uint16_t) 1)
#define CAN_STD_ID_Rc       ((uint16_t) 2)
#define CAN_STD_ID_Gimbal   ((uint16_t) 3)
#define CAN_STD_ID_Referee 	((uint16_t) 4)

#define CAN_Test_Duration   ((uint32_t)20) // ms
#define CAN_Rc_Duration     ((uint32_t)16) // ms
#define CAN_Gimbal_Duration ((uint32_t)10) // ms

// 任意类型的数据包
#define CAN_STD_ID_ANY_BASE     ((uint16_t)0x0601)

// clang-format on

/*-------------------- Send & Receive --------------------*/

// typedef union {
//     struct
//     {
//     } __attribute__((packed)) data;
// } Data_Test_u;

typedef struct
{
    struct
    {
        union {
            struct
            {
                uint16_t ch0 : 11;
                uint16_t ch1 : 11;
                uint16_t ch2 : 11;
                uint16_t ch3 : 11;
                uint16_t ch4 : 11;
                uint8_t s0 : 2;
                uint8_t s1 : 2;
                bool offline : 1;
                uint8_t reserved : 4;
            } __packed__ packed;
            struct
            {
                uint8_t data[8];
            } __packed__ raw;
        } rc;

        union {
            struct
            {
                int16_t mouse_x : 15;
                uint16_t mouse_press_l : 1;
                int16_t mouse_y : 15;
                uint16_t mouse_press_r : 1;
                int16_t mouse_z;
                int16_t key;
            } __packed__ packed;
            struct
            {
                uint8_t data[8];
            } __packed__ raw;
        } km;
        RC_ctrl_t rc_unpacked;  // 遥控器数据
    } rc_data;

    struct
    {
        union {
            struct
            {
                float yaw;
                bool offline;
                bool init_judge;
                uint16_t reserved;
            } __packed__ packed_data;
            struct
            {
                uint8_t data[8];
            } __packed__ raw;
        } gimbal;

    } gimbal_data;
	// 3. 裁判系统数据（新增：热量闭环）
    struct {
        union {
            uint8_t data[8];
            struct {
                uint16_t heat_limit; // 热量上限
                uint16_t heat;       // 当前热量
            } packed;
        } raw;
    } referee_data;
	
	
} CanBoardCommunicate_t;

#endif
/*------------------------------ End of File ------------------------------*/
