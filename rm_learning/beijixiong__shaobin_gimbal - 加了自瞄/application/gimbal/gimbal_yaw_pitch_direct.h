/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024-04-01      Penguin         1. done
  *  V1.2.0     2025-02-26      Harry_Wong      1.删除了不需要的变量
  *                                             2.剩余更新详情请见.c文件
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
#ifndef GIMBAL_YAW_PITCH_H
#define GIMBAL_YAW_PITCH_H
#include "IMU.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"
#include "CAN_cmd_dji.h"
#include "detect_task.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "math.h"
#include "macro_typedef.h"
#include "gimbal.h"
#include "CAN_cmd_damiao.h"
#include "VOFA.h"
#include "usb_task.h"

/* 自瞄模式PID参数宏定义（需要根据实际情况调整） */
//yaw 角度环（电机为6020时使用）
#ifndef KP_AIM_YAW_ANGLE
#define KP_AIM_YAW_ANGLE       15.0f
#endif

#ifndef KI_AIM_YAW_ANGLE
#define KI_AIM_YAW_ANGLE       0.0f
#endif

#ifndef KD_AIM_YAW_ANGLE
#define KD_AIM_YAW_ANGLE       1.0f
#endif

#ifndef MAX_OUT_AIM_YAW_ANGLE
#define MAX_OUT_AIM_YAW_ANGLE  10.0f
#endif

#ifndef MAX_IOUT_AIM_YAW_ANGLE
#define MAX_IOUT_AIM_YAW_ANGLE 2.0f
#endif
//yaw速度环（电机为6020时使用）
#ifndef KP_AIM_YAW_VELOCITY
#define KP_AIM_YAW_VELOCITY    5000.0f
#endif

#ifndef KI_AIM_YAW_VELOCITY
#define KI_AIM_YAW_VELOCITY    0.0f
#endif

#ifndef KD_AIM_YAW_VELOCITY
#define KD_AIM_YAW_VELOCITY    100.0f
#endif

#ifndef MAX_OUT_AIM_YAW_VELOCITY
#define MAX_OUT_AIM_YAW_VELOCITY 16000.0f
#endif

#ifndef MAX_IOUT_AIM_YAW_VELOCITY
#define MAX_IOUT_AIM_YAW_VELOCITY 2000.0f
#endif
//yaw轴pid（电机为dm4310）
		
#define	KP_MIT_AIM		33.0f

#define KD_MIT_AIM		1.6f


//pitch角度环
#ifndef KP_AIM_PITCH_ANGLE
#define KP_AIM_PITCH_ANGLE     15.0f
#endif

#ifndef KI_AIM_PITCH_ANGLE
#define KI_AIM_PITCH_ANGLE     0.0f
#endif

#ifndef KD_AIM_PITCH_ANGLE
#define KD_AIM_PITCH_ANGLE     0.8f
#endif

#ifndef MAX_OUT_AIM_PITCH_ANGLE
#define MAX_OUT_AIM_PITCH_ANGLE 10.0f
#endif

#ifndef MAX_IOUT_AIM_PITCH_ANGLE
#define MAX_IOUT_AIM_PITCH_ANGLE 2.0f
#endif
//pitch速度环
#ifndef KP_AIM_PITCH_VELOCITY
#define KP_AIM_PITCH_VELOCITY  3500.0f
#endif

#ifndef KI_AIM_PITCH_VELOCITY
#define KI_AIM_PITCH_VELOCITY  0.0f
#endif

#ifndef KD_AIM_PITCH_VELOCITY
#define KD_AIM_PITCH_VELOCITY  100.0f
#endif

#ifndef MAX_OUT_AIM_PITCH_VELOCITY
#define MAX_OUT_AIM_PITCH_VELOCITY 16000.0f
#endif

#ifndef MAX_IOUT_AIM_PITCH_VELOCITY
#define MAX_IOUT_AIM_PITCH_VELOCITY 2000.0f
#endif

/**
 * @brief 云台模式
 */
typedef enum {
    GIMBAL_ZERO_FORCE,  // 云台无力，所有控制量置0
    GIMBAL_IMU,         // 云台陀螺仪控制(角度控制)
    GIMBAL_INIT,        //云台矫正模式
    GIMBAL_DBUS_ERR,    //遥控器断联相关处理任务
    GIMBAL_GAP,         //跳出矫正进入IMU/AUTO_AIM模式之前的存储数据模式
    GIMBAL_AUTO_AIM,    //自瞄模式
} GimbalMode_e;

/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float pitch;
    float yaw;
} Values_t;

typedef struct
{
    pid_type_def yaw_angle;
    pid_type_def yaw_velocity;  //角速度
 
    pid_type_def pitch_angle;
    pid_type_def pitch_velocity;
    
} PID_t;

//新增dm——yaw     汪辰旭 1.17
typedef struct feedback
{
  fp32 yaw_angel_fdb;// 拨弹盘输出轴位置
  fp32 yaw_speed_fdb;// 拨弹盘输出轴速度
} Fdb;

typedef struct
{
    const RC_ctrl_t * rc;  // 遥控器指针
    GimbalMode_e mode,last_mode,mode_before_rc_err;  // 模式

    /*-------------------- Motors --------------------*/
    Motor_s yaw,pitch;
	
	Fdb FDB;				//新增 		汪辰旭 1.17
    /*-------------------- Values --------------------*/
    Values_t reference;    // 期望值
    Values_t feedback_pos,feedback_vel;     // 状态值(目前专供给IMU数据)
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值
	Values_t vision_reference;         // 视觉下发的目标值缓存
	
	/*-------------------- Vision Status --------------------*/
    bool vision_control_enabled;       // 视觉是否接管
    uint8_t shoot_cmd;                 // 射击指令 (0:不射, 1:射击)
    int16_t target_distance;           // 目标距离

    Values_t init_base;    //初始上电的imu   蒋远志2025.10.14添加
    bool init_base_record; //是否需要记录imu初始位置  蒋远志2025.10.14添加

    PID_t pid;  // PID控制器
//	PID_t aim_pid;                     // 自瞄专用PID (可选)

    float angle_zero_for_imu; //pitch电机处于中值时imupitch的角度

    uint32_t init_start_time,init_timer;

    bool init_continue; //是否继续进行校准模式
} Gimbal_s;

extern Gimbal_s gimbal_direct;
extern PID_t gimbal_direct_pid;
extern PID_t gimbal_aim_pid;

extern void GimbalInit(void);

extern void GimbalHandleException(void);

extern void GimbalObserver(void);

extern void GimbalReference(void);

extern void GimbalConsole(void);

extern void GimbalSendCmd(void);

#endif  // GIMBAL_YAW_PITCH_H
#endif  // GIMBAL_YAW_PITCH




