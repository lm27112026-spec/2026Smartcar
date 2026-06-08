#include "robot_param.h"

#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)
#ifndef SHOOT_FRIC_H
#define SHOOT_FRIC_H
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_communication.h"
#include "math.h"
#include "user_lib.h"
#include "arm_math.h"
#include "detect_task.h"

// 遥控器相关宏定义
#define SHOOT_MODE_CHANNEL 1  // 射击发射开关通道数据
// 任务相关宏定义
#define SHOOT_TASK_INIT_TIME 201  // 任务初始化 空闲一段时间
#define SHOOT_CONTROL_TIME 1      // 任务控制间隔 1ms

//#define SHOOT_HEAT_REMAIN_VALUE 80 

typedef enum 
{
    LOAD_STOP,      // 停止拨盘
    LAOD_BULLET,    // 单发模式
    LOAD_BURSTFIRE,  // 连发模式,对速度闭环
    LOAD_BLOCK       // 堵转，模式
} LoadMode_e;

typedef enum 
{
    FRIC_NOT_READY,      // 未准备发射
    FRIC_READY,          // 准备发射
} FricState_e;

typedef struct feedback
{
  fp32 trigger_angel_fdb;// 拨弹盘输出轴位置
  fp32 trigger_speed_fdb;// 拨弹盘输出轴速度
  fp32 fric_speed_fdb_L;   // 摩擦轮输出轴速度
  fp32 fric_speed_fdb_R;
} Fdb;

typedef struct reference
{
  fp32 trigger_angel_ref;// 拨弹盘位置期望
  fp32 trigger_speed_ref;// 拨弹盘速度期望
  fp32 fric_speed_ref_L;   // 摩擦轮速度期望
  fp32 fric_speed_ref_R;
} Ref;

typedef struct
{
  const RC_ctrl_t * rc;  // 射击使用的遥控器指针
  LoadMode_e mode;       // 射击模式
  FricState_e state;     // 摩擦轮状态

  Motor_s fric_motor[2];  // 摩擦轮电机
  Motor_s trigger_motor;  // 拨弹盘电机

    //pid
  pid_type_def trigger_angel_pid;
  pid_type_def trigger_speed_pid;
  pid_type_def fric_pid[2];

    //block_reverse
  uint16_t reverse_time;
  uint16_t block_time;
  fp32 last_trigger_vel;
  fp32 last_fric_vel;
    
    //feedback
  Fdb FDB;
  
    //reference
  Ref REF;

    //flag
  uint16_t fric_flag; //    摩擦轮状态
  uint16_t move_flag; //    拨弹盘角度状态，用于判断单发射击执行情况
  uint16_t shoot_flag;//    鼠标左键状态，用于判断弹发射击启动

   //ecd
  int16_t last_ecd; //     上一个ecd
  int16_t ecd_count;//     ecd计数

  // heat
  uint16_t heat_limit;
  uint16_t heat;
  uint16_t mr_time;
} Shoot_s;



extern void ShootInit(void);

extern void ShootSetMode(void);

extern void ShootObserver(void);

extern void ShootReference(void);

extern void ShootConsole(void);

extern void ShootSendCmd(void);

#endif  // SHOOT_FRIC_H
#endif  // SHOOT_TYPE == SHOOT_FRIC

