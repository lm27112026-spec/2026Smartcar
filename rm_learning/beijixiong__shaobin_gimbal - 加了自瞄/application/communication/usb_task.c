/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c
  * @brief      通过USB串口与上位机通信（视觉专用）
  * @note       已适配上位机串口通信协议（V1.3.0 2025-12-20）
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done
  *  V1.3.0     2025-12-20      AI Assistant    1. 精简为视觉专用通信
  @verbatim
  =================================================================================
  功能说明：
  1. 发送云台状态数据 (GimbalToVision) 给上位机
  2. 接收视觉控制指令 (VisionToGimbal) 并解析
  3. 只保留与视觉通信相关的核心功能
  
  通信协议：
  发送：GimbalToVision_t 结构体（固定帧头 'S' 'P' + CRC16校验）
  接收：VisionToGimbal_t 结构体（固定帧头 'S' 'P' + CRC16校验）
  
  发送周期：5ms (200Hz)
  超时检测：100ms无数据视为离线
  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <stdbool.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "macro_typedef.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "gimbal.h"
#include "IMU.h"
#include "usb_typdef.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

#define USB_TASK_CONTROL_TIME 1  /* ms */
#define USB_OFFLINE_THRESHOLD 100  /* ms */
#define USB_CONNECT_CNT 10

/* 发送周期定义 */
#define SEND_DURATION_GimbalToVision 5   /* ms，云台数据发送周期 */
#define SEND_DURATION_VisionSimple   5   /* ms，新协议发送周期 */

/* 缩放因子定义 */
#define SCALE_QUAT      10000.0f
#define SCALE_ANGLE     10000.0f
#define SCALE_SPEED     100.0f

/* USB接收缓冲区 */
#define USB_RX_DATA_SIZE 256  /* byte */
#define USB_RECEIVE_LEN 150   /* byte */

/* 检查并发送数据的宏 */
#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - LAST_SEND_TIME.##send_name) >= SEND_DURATION_##send_name) { \
            LAST_SEND_TIME.##send_name = HAL_GetTick();                                  \
            UsbSend##send_name##Data();                                                  \
        }                                                                                \
    } while (0)

/*******************************************************************************/
/* 变量声明                                                                   */
/*******************************************************************************/


	
/* USB接收缓冲区 */
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

/* IMU数据指针 */
static const Imu_t * IMU;

/* 连接状态相关变量 */
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

/* 上位机通信数据结构 */
static GimbalToVision_t SEND_GIMBAL_TO_VISION;
static VisionToGimbal_t RECEIVE_VISION_TO_GIMBAL;
	
/* 新增协议数据结构 */
static VisionFrame_Imu_t    SEND_FRAME_IMU;        /* 0xAA */
static VisionFrame_Status_t SEND_FRAME_STATUS;     /* 0xBB */
static VisionFrame_Control_t RECEIVE_FRAME_CONTROL; /* 0xCC */

/* 视觉控制指令数据 */
static VisionCmdData_t VISION_CMD_DATA;

/* 发送时间记录 */
typedef struct {
    uint32_t GimbalToVision;
	uint32_t VisionSimple;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

/*******************************************************************************/
/* 函数声明                                                                   */
/*******************************************************************************/

static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);
static uint16_t CalculateGimbalCRC16(uint8_t* data, uint32_t length);

static void UsbSendGimbalToVisionData(void);
static void ProcessVisionToGimbalData(void);

/* 新协议函数 */
static void UsbSendVisionSimpleData(void);
static void ProcessVisionControlFrame(void);

/******************************************************************/
/* 任务函数                                                       */
/******************************************************************/

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    vTaskDelay(10);  /* 等待USB设备初始化完成 */
    
    UsbInit();

    while (1) {
        UsbSendData();          /* 发送数据 */
        UsbReceiveData();       /* 接收数据 */
//        ProcessVisionToGimbalData();  /* 处理视觉控制指令 */

        /* 连接状态检测 */
        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* 初始化函数                                                                 */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    /* 订阅IMU数据 */
    IMU = Subscribe(IMU_NAME);
    
    /* 数据清零 */
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&SEND_GIMBAL_TO_VISION, 0, sizeof(GimbalToVision_t));
    memset(&RECEIVE_VISION_TO_GIMBAL, 0, sizeof(VisionToGimbal_t));
	
	memset(&SEND_FRAME_IMU, 0, sizeof(VisionFrame_Imu_t));
    memset(&SEND_FRAME_STATUS, 0, sizeof(VisionFrame_Status_t));
    memset(&RECEIVE_FRAME_CONTROL, 0, sizeof(VisionFrame_Control_t));
	
    memset(&VISION_CMD_DATA, 0, sizeof(VisionCmdData_t));
    
    /* 初始化GimbalToVision帧头 */
    SEND_GIMBAL_TO_VISION.head[0] = 'S';
    SEND_GIMBAL_TO_VISION.head[1] = 'P';
	SEND_FRAME_IMU.head = 0xAA;
    SEND_FRAME_STATUS.head = 0xBB;
	
    SEND_GIMBAL_TO_VISION.mode = 0;  /* 默认空闲模式 */
    
    /* 初始化子弹相关数据 */
    SEND_GIMBAL_TO_VISION.bullet_speed = 0;
    SEND_GIMBAL_TO_VISION.bullet_count = 0;
    
    /* 初始化四元数为单位四元数 */
    SEND_GIMBAL_TO_VISION.q[0] = 1.0f;  /* w */
    SEND_GIMBAL_TO_VISION.q[1] = 0.0f;  /* x */
    SEND_GIMBAL_TO_VISION.q[2] = 0.0f;  /* y */
    SEND_GIMBAL_TO_VISION.q[3] = 0.0f;  /* z */
}

/*******************************************************************************/
/* 发送数据函数                                                               */
/*******************************************************************************/

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
//    /* 发送云台数据到上位机 */
//    CheckDurationAndSend(GimbalToVision);
	/* 发送新协议 0xAA/0xBB 帧 */
    CheckDurationAndSend(VisionSimple);
}

//新增    汪辰旭 
/**
 * @brief 发送新协议的 0xAA 和 0xBB 帧
 */
static void UsbSendVisionSimpleData(void)
{
    if (IMU == NULL) return;

    /* --- 1. 准备 0xAA IMU 帧 --- */
    SEND_FRAME_IMU.head = 0xAA;
    /* IMU四元数转换: float -> int16 (x10000) */
	/* 注意：IMU->q[0]=w, q[1]=x, q[2]=y, q[3]=z */
	SEND_FRAME_IMU.w = (int16_t)(IMU->q[0] * SCALE_QUAT); /* w 对应 q[0] */
	SEND_FRAME_IMU.x = (int16_t)(IMU->q[1] * SCALE_QUAT); /* x 对应 q[1] */
	SEND_FRAME_IMU.y = (int16_t)(IMU->q[2] * SCALE_QUAT); /* y 对应 q[2] */
	SEND_FRAME_IMU.z = (int16_t)(IMU->q[3] * SCALE_QUAT); /* z 对应 q[3] */
    
    /* 发送 0xAA 帧 (9字节) */
    USB_Transmit((uint8_t *)&SEND_FRAME_IMU, sizeof(VisionFrame_Imu_t));
    
    /* 添加微小延时防止粘包（可选，取决于上位机解析能力） */
    // vTaskDelay(1);

    /* --- 2. 准备 0xBB 状态帧 --- */
    SEND_FRAME_STATUS.head = 0xBB;
    /* 这里使用模拟数据或全局变量，请替换为实际获取函数 */
    // float bullet_speed_f = GetBulletSpeed(); 
    // uint8_t robot_mode = GetRobotMode();
    float bullet_speed_f = 0.0f; /* 示例 */
    
    SEND_FRAME_STATUS.bullet_speed = (int16_t)(bullet_speed_f * SCALE_SPEED);
    SEND_FRAME_STATUS.mode = (int16_t)0;         /* 示例: 空闲模式 */
    SEND_FRAME_STATUS.shoot_mode = (int16_t)0;   /* 示例: 停止射击 */
    /* ft_angle 通常指 Pitch 轴角度或摩擦轮相关角度 */
    SEND_FRAME_STATUS.ft_angle = (int16_t)(IMU->angle[AX_PITCH] * SCALE_ANGLE);
    
    /* 发送 0xBB 帧 (9字节) */
    USB_Transmit((uint8_t *)&SEND_FRAME_STATUS, sizeof(VisionFrame_Status_t));
}


/**
 * @brief 发送云台数据到上位机（GimbalToVision）
 */
static void UsbSendGimbalToVisionData(void)
{
    if (IMU == NULL) {
        return;
    }
    
    /* 更新帧头 */
    SEND_GIMBAL_TO_VISION.head[0] = 'S';
    SEND_GIMBAL_TO_VISION.head[1] = 'P';
    
    /* 更新云台模式（需要根据实际云台状态获取） */
    SEND_GIMBAL_TO_VISION.mode = 0;  /* 默认空闲模式 */
    
    /* 更新IMU数据 */
    SEND_GIMBAL_TO_VISION.yaw = IMU->angle[AX_Z];
    SEND_GIMBAL_TO_VISION.pitch = IMU->angle[AX_Y];
    SEND_GIMBAL_TO_VISION.yaw_vel = IMU->gyro[AX_Z];
    SEND_GIMBAL_TO_VISION.pitch_vel = IMU->gyro[AX_Y];
    
    /* 更新四元数（这里需要从IMU获取，暂时使用默认值） */
    /* TODO: 从IMU获取四元数数据 */
    
    /* 计算CRC16校验（覆盖除crc16字段外的所有数据） */
    uint32_t data_size = sizeof(GimbalToVision_t) - sizeof(uint16_t);
    SEND_GIMBAL_TO_VISION.crc16 = CalculateGimbalCRC16((uint8_t*)&SEND_GIMBAL_TO_VISION, data_size);
    
    /* 通过USB发送数据 */
    USB_Transmit((uint8_t *)&SEND_GIMBAL_TO_VISION, sizeof(GimbalToVision_t));
}

/**
 * @brief 计算云台通信的CRC16校验
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC16校验值
 */
static uint16_t CalculateGimbalCRC16(uint8_t* data, uint32_t length)
{
    /* 使用现有的CRC16计算函数 */
    /* 注意：需要确保与上位机使用的CRC16算法一致 */
    uint16_t crc = 0xFFFF;  /* CRC初始值 */
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;  /* CRC-16-CCITT多项式 */
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/*******************************************************************************/
/* 接收数据函数                                                               */
/*******************************************************************************/

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t * rx_data_start_address = USB_RX_BUF;
    static uint8_t * rx_data_end_address;
    uint8_t * sof_address = USB_RX_BUF;

    /* 计算数据包的结束位置 */
    rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
    
    /* 读取数据 */
    USB_Receive(rx_data_start_address, &len);

    while (sof_address <= rx_data_end_address - sizeof(VisionToGimbal_t) + 1) {
		 /* --- 检测 0xCC 新协议帧 (9字节) --- */
        if (sof_address[0] == 0xCC) {
            if (sof_address + sizeof(VisionFrame_Control_t) <= rx_data_end_address) {
                /* 复制数据 (无CRC，直接信任) */
                memcpy(&RECEIVE_FRAME_CONTROL, sof_address, sizeof(VisionFrame_Control_t));
                
                /* 处理数据 */
                ProcessVisionControlFrame();
                
                /* 更新连接状态 */
                RECEIVE_TIME = HAL_GetTick();
                CONTINUE_RECEIVE_CNT++;
                
                /* 移动指针 */
                sof_address += sizeof(VisionFrame_Control_t);
                continue;
            }
        }
        /* 寻找'S' 'P'帧头位置 */
       else if (sof_address[0] == 'S' && sof_address[1] == 'P') {
            /* 检查是否接收到完整数据帧 */
            if (sof_address + sizeof(VisionToGimbal_t) <= rx_data_end_address) {
                /* 验证CRC16校验 */
                VisionToGimbal_t *vision_data = (VisionToGimbal_t *)sof_address;
                uint32_t data_size = sizeof(VisionToGimbal_t) - sizeof(uint16_t);
                uint16_t calculated_crc = CalculateGimbalCRC16((uint8_t*)vision_data, data_size);
                
                if (calculated_crc == vision_data->crc16) {
                    /* CRC校验通过，复制数据 */
                    memcpy(&RECEIVE_VISION_TO_GIMBAL, sof_address, sizeof(VisionToGimbal_t));
                    RECEIVE_TIME = HAL_GetTick();
                    CONTINUE_RECEIVE_CNT++;
                    
                    /* 移动到下一个数据包 */
                    sof_address += sizeof(VisionToGimbal_t);
                    continue;  /* 继续处理下一个数据包 */
                }
            }
        }
        sof_address++;
    }
    
    /* 更新下一次接收数据的起始位置 */
    if (sof_address > rx_data_start_address + USB_RECEIVE_LEN) {
        /* 缓冲区中没有剩余数据 */
        rx_data_start_address = USB_RX_BUF;
    } else {
        uint16_t remaining_data_len = USB_RECEIVE_LEN - (sof_address - rx_data_start_address);
        /* 缓冲区中有剩余数据 */
        rx_data_start_address = USB_RX_BUF + remaining_data_len;
        /* 将剩余数据移到缓冲区的起始位置 */
        memcpy(USB_RX_BUF, sof_address, remaining_data_len);
    }
}

/**
 * @brief 处理 0xCC 控制帧数据
 */
static void ProcessVisionControlFrame(void)
{
    /* 解析控制位 */
    VISION_CMD_DATA.control_mode = RECEIVE_FRAME_CONTROL.control;
    VISION_CMD_DATA.shoot_cmd    = RECEIVE_FRAME_CONTROL.shoot;
    
    /* 
     * 解析角度 
     * 假设协议中 float -> int16 也是乘以 10000 
     * 如果上位机发送的是原始角度值(度)乘以100，这里需要修改除数
     */
    VISION_CMD_DATA.yaw   = (float)RECEIVE_FRAME_CONTROL.yaw / SCALE_ANGLE;
    VISION_CMD_DATA.pitch = (float)RECEIVE_FRAME_CONTROL.pitch / SCALE_ANGLE;
    
    VISION_CMD_DATA.distance = RECEIVE_FRAME_CONTROL.horizon_distance;
    
    /* 视觉模式可以通过 control_mode 隐式定义，或者协议中未包含 */
    // VISION_CMD_DATA.vision_mode = ...
}

/**
 * @brief 处理接收到的视觉控制指令
 */
static void ProcessVisionToGimbalData(void)
{
    /* 更新视觉控制指令数据 */
    VISION_CMD_DATA.yaw = RECEIVE_VISION_TO_GIMBAL.yaw;
    VISION_CMD_DATA.pitch = RECEIVE_VISION_TO_GIMBAL.pitch;
    VISION_CMD_DATA.control_mode = RECEIVE_VISION_TO_GIMBAL.mode;
    VISION_CMD_DATA.vision_mode = 0;  /* 需要根据实际解析 */
}

/*******************************************************************************/
/* API函数                                                                     */
/*******************************************************************************/

/**
 * @brief 获取视觉控制指令数据
 * @return VisionCmdData_t 视觉控制指令数据
 */
VisionCmdData_t GetVisionCmdData(void)
{
    return VISION_CMD_DATA;
}

/**
 * @brief 检查视觉控制是否使能
 * @return true: 使能, false: 禁用
 */
bool IsVisionControlEnabled(void)
{
    return (VISION_CMD_DATA.control_mode > 0) && (!USB_OFFLINE);
}

/**
 * @brief 获取视觉控制的偏航角目标
 * @return float 偏航角目标值 (rad)
 */
float GetVisionCmdYaw(void)
{
    return VISION_CMD_DATA.yaw;
}

/**
 * @brief 获取视觉控制的俯仰角目标
 * @return float 俯仰角目标值 (rad)
 */
float GetVisionCmdPitch(void)
{
    return VISION_CMD_DATA.pitch;
}

/**
 * @brief 获取视觉控制模式
 * @return uint8_t 控制模式: 0=不控制/1=控制不开火/2=控制开火
 */
uint8_t GetVisionControlMode(void)
{
    return VISION_CMD_DATA.control_mode;
}

/**
 * @brief 获取视觉模式
 * @return uint8_t 视觉模式: 0=空闲/1=自瞄/2=小符/3=大符
 */
uint8_t GetVisionMode(void)
{
    return VISION_CMD_DATA.vision_mode;
}

/*******************************************************************************/
/* 兼容性函数 - 保持与原有代码的兼容性                                       */
/*******************************************************************************/

/**
 * @brief 获取上位机控制指令：云台姿态，基于欧拉角 r×p×y
 * @param axis 轴id，可配合定义好的轴id宏 AX_PITCH,AX_YAW 使用
 * @return (rad) 云台姿态
 */
float GetScCmdGimbalAngle(uint8_t axis)
{
    if (axis == AX_YAW) {
        return VISION_CMD_DATA.yaw;
    } else if (axis == AX_PITCH) {
        return VISION_CMD_DATA.pitch;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：开火
 * @param void
 * @return bool 是否开火
 */
bool GetScCmdFire(void)
{
    /* 兼容两种协议的开火判断 */
    return (VISION_CMD_DATA.control_mode == 2) || (VISION_CMD_DATA.shoot_cmd > 0);
}

/**
 * @brief 获取上位机控制指令：启动摩擦轮
 * @param void
 * @return bool 是否启动摩擦轮
 */
bool GetScCmdFricOn(void)
{
    /* 控制模式不为0表示需要启动摩擦轮 */
    return (VISION_CMD_DATA.control_mode != 0);
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动线速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (m/s) 底盘坐标系下axis方向运动线速度
 */
float GetScCmdChassisSpeed(uint8_t axis)
{
    /* 视觉通信不涉及底盘控制，返回0 */
    (void)axis; /* 消除未使用参数警告 */
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动角速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (rad/s) 底盘坐标系下axis方向运动角速度
 */
float GetScCmdChassisVelocity(uint8_t axis)
{
    /* 视觉通信不涉及底盘控制，返回0 */
    (void)axis; /* 消除未使用参数警告 */
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘离地高度，平衡底盘中可用作腿长参数
 * @param void
 * @return (m) 底盘离地高度
 */
float GetScCmdChassisHeight(void)
{
    /* 视觉通信不涉及底盘控制，返回0 */
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘角度
 * @param axis 轴id
 * @return float 底盘角度
 */
float GetScCmdChassisAngle(uint8_t axis)
{
    /* 视觉通信不涉及底盘控制，返回0 */
    (void)axis; /* 消除未使用参数警告 */
    return 0.0f;
}

/**
 * @brief 获取虚拟遥控器通道值
 * @param channel 通道号
 * @return float 通道值
 */
float GetVirtualRcCh(uint8_t channel)
{
    /* 视觉通信不使用虚拟遥控器，返回0 */
    (void)channel; /* 消除未使用参数警告 */
    return 0.0f;
}

/**
 * @brief 获取虚拟遥控器开关状态
 * @param channel 通道号
 * @return char 开关状态
 */
char GetVirtualRcSw(uint8_t channel)
{
    /* 视觉通信不使用虚拟遥控器，返回0 */
    (void)channel; /* 消除未使用参数警告 */
    return 0;
}

/*------------------------------ End of File ------------------------------*/
