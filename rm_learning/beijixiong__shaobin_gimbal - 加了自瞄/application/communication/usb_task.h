/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       usb_task.h
  * @brief      通过USB串口与上位机通信（视觉专用）
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done
  *  V1.3.0     2025-12-20      AI Assistant    1. 精简为视觉专用通信
  *
  @verbatim
  ==============================================================================
  与上位机的视觉通信协议（云台控制）
  发送：云台状态数据 (GimbalToVision)
  接收：视觉控制指令 (VisionToGimbal)
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#ifndef USB_TASK_H
#define USB_TASK_H

#include "robot_param.h"
#include "struct_typedef.h"  /* 添加这个头文件以获取 AX_YAW, AX_PITCH 等定义 */

/* 上位机通信数据结构体定义 */
#pragma pack(push, 1)  /* 取消字节对齐，确保与串口字节流一致 */

/* 云台向上位机发送的状态数据帧 (GimbalToVision) */
typedef struct {
    uint8_t head[2];          /* 帧头，固定为 'S' 'P' */
    uint8_t mode;             /* 云台模式: 0=空闲/1=自瞄/2=小符/3=大符 */
    float q[4];               /* 姿态四元数 (w-x-y-z，归一化) */
    float yaw;                /* 偏航角 (rad) */
    float yaw_vel;           /* 偏航角速度 (rad/s) */
    float pitch;              /* 俯仰角 (rad) */
    float pitch_vel;         /* 俯仰角速度 (rad/s) */
    float bullet_speed;      /* 子弹速度 (m/s) */
    uint16_t bullet_count;   /* 剩余子弹数 */
    uint16_t crc16;          /* CRC16校验 */
} GimbalToVision_t;

/* 上位机向云台下发的控制数据帧 (VisionToGimbal) */
typedef struct {
    uint8_t head[2];          /* 帧头，固定为 'S' 'P' */
    uint8_t mode;             /* 控制模式: 0=不控制/1=控制不开火/2=控制开火 */
    float yaw;                /* 偏航角目标值 (rad) */
    float yaw_vel;           /* 偏航角速度目标 (rad/s) */
    float yaw_acc;           /* 偏航角加速度目标 (rad/s^2) */
    float pitch;              /* 俯仰角目标值 (rad) */
    float pitch_vel;         /* 俯仰角速度目标 (rad/s) */
    float pitch_acc;         /* 俯仰角加速度目标 (rad/s^2) */
    uint16_t crc16;          /* CRC16校验 */
} VisionToGimbal_t;

///* ----------------- 新增9字节极简协议定义 ----------------- */

/* 1. IMU数据帧 (MCU -> Vision)
 * 帧头：0xAA
 * 数据：x, y, z, w (int16_t, 实际值 * 10000)
 */
typedef struct {
    uint8_t head;       /* Fixed: 0xAA */
    int16_t w;          /* q[1] * 10000 */
    int16_t x;          /* q[2] * 10000 */
    int16_t y;          /* q[3] * 10000 */
    int16_t z;          /* q[0] * 10000 */
} VisionFrame_Imu_t;

/* 2. 状态数据帧 (MCU -> Vision)
 * 帧头：0xBB
 * 数据：bullet_speed(*100), mode, shoot_mode, ft_angle(*10000)
 */
typedef struct {
    uint8_t head;           /* Fixed: 0xBB */
    int16_t bullet_speed;   /* speed * 100 */
    int16_t mode;           /* robot state */
    int16_t shoot_mode;     /* shoot state */
    int16_t ft_angle;       /* pitch/friction angle * 10000 */
} VisionFrame_Status_t;

/* 3. 控制指令帧 (Vision -> MCU)
 * 帧头：0xCC
 * 数据：control, shoot, yaw, pitch, distance
 */
typedef struct {
    uint8_t head;              /* Fixed: 0xCC */
    uint8_t control;           /* control command */
    uint8_t shoot;             /* shoot command */
    int16_t yaw;               /* target yaw (scale TBD, assume *10000 or raw) */
    int16_t pitch;             /* target pitch (scale TBD) */
    int16_t horizon_distance;  /* distance */
} VisionFrame_Control_t;

#pragma pack(pop)  /* 恢复字节对齐 */

/* 视觉控制指令数据 */
typedef struct {
    float yaw;               /* 偏航角目标 (rad) */
    float pitch;             /* 俯仰角目标 (rad) */
    uint8_t control_mode;    /* 控制模式: 0=不控制/1=控制不开火/2=控制开火 */
    uint8_t vision_mode;     /* 视觉模式: 0=空闲/1=自瞄/2=小符/3=大符 */
	
	/* --- 添加以下两个新成员 --- */
    uint8_t shoot_cmd;       /* 射击指令 */
    int16_t distance;        /* 距离 */
} VisionCmdData_t;

/* 函数声明 */
extern void usb_task(void const * argument);

/* API函数 */
extern VisionCmdData_t GetVisionCmdData(void);
extern _Bool IsVisionControlEnabled(void);  /* 在 usb_task.c 中定义 */
extern float GetVisionCmdYaw(void);
extern float GetVisionCmdPitch(void);
extern uint8_t GetVisionControlMode(void);
extern uint8_t GetVisionMode(void);

/* 兼容性函数 - 保持与原有代码的兼容性 */
extern float GetScCmdGimbalAngle(uint8_t axis);
extern _Bool GetScCmdFire(void);
extern _Bool GetScCmdFricOn(void);
extern float GetScCmdChassisSpeed(uint8_t axis);
extern float GetScCmdChassisVelocity(uint8_t axis);
extern float GetScCmdChassisHeight(void);
extern float GetScCmdChassisAngle(uint8_t axis);
extern float GetVirtualRcCh(uint8_t channel);
extern char GetVirtualRcSw(uint8_t channel);

#endif /* USB_TASK_H */
