#include "robot_param.h"
#if(CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#ifndef CHASSIS_OMNI_H
#define CHASSIS_OMNI_H
//#include "chassis.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "motor.h"
#include "CAN_cmd_dji.h"



typedef enum {
    CHASSIS_LOCK,      //底盘锁定，所有轮子速度设定为0
    CHASSIS_SINGLE,    //只有底盘的模式
    CHASSIS_FOLLOW,    //云台跟随模式
    CHASSIS_SPIN,	   //小陀螺
} ChassisMode_e;

/**
 * @brief  底盘轮子PID
 */
 typedef struct
{
    pid_type_def wheel_velocity[4];//麦轮速度解算PID

    pid_type_def follow; //云台跟随PID
} PID_t;

/**
 * @brief  底盘期望
 */
typedef struct
{
    float vx;
    float vy;
    float wz;
} Reference_t;


/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
//    const Imu_t * imu;     // imu数据
    ChassisMode_e mode;    // 底盘模式

    /*-------------------- Motors --------------------*/
    Motor_s wheel[4];  //底盘电机

    /*-------------------- Values --------------------*/
    Reference_t reference; 
    Reference_t reference_rc;

    fp32 feedback[4];
    fp32 set[4];

    fp32 yaw_delta;

    uint16_t x_time;
    uint16_t y_time;
    bool spin_flag, shift_flag, shift_last_flag;
	
	 // 底盘功率控制相关变量
    fp32 dynamic_max_speed;        // 动态最大平移速度
    fp32 dynamic_max_velocity;     // 动态最大旋转速度
    fp32 speed_scale_factor;       // 速度缩放系数 (0.0-1.0+)
    uint8_t power_control_state;   // 功率控制状态 0:禁用 1:启用
	
} Chassis_s;

extern void ChassisInit(void);
extern void ChassisSetMode(void);
extern void ChassisObserver(void);
extern void ChassisReference(void);
extern void ChassisConsole(void);
extern void ChassisSendCmd(void);

#endif
#endif
