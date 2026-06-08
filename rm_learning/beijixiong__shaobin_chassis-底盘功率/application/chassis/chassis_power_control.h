/**
  ****************************(C) COPYRIGHT 2025****************************
  * @file       chassis_power_control.h
  * @brief      底盘功率控制（纯电池/裁判系统版，已去除超级电容）
  **************************************************************************
  */
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

#include "main.h"
#include "struct_typedef.h"
#include "referee.h"
#include "remote_control.h"

// 功率控制结构体（精简版）
typedef struct {
    float max_referee_power;     // 裁判系统限制最大功率 (W)
    float buffer_energy;         // 当前裁判系统缓冲能量 (J)
} PowerLimit_t;

extern PowerLimit_t power_limit;

// ============================================================================
// 功率与速度控制参数宏定义
// ============================================================================

// --------------------------------------
// 比赛模式功率参数
// --------------------------------------
#define NORMAL_POWER_LIMIT         180.0f   
#define INFANTRY_BATTLE_POWER      120.0f   
#define THREE_VS_THREE_POWER_HIGH  90.0f   
#define THREE_VS_THREE_POWER_LOW   75.0f   

// --------------------------------------
// 缓冲能量阈值 (标准最大值为60J)
// --------------------------------------
#define BUFFER_ENERGY_CRITICAL_LOW  15.0f   // 极低缓冲能量：强制大幅减速
#define BUFFER_ENERGY_LOW           25.0f   // 偏低缓冲能量：开始限制速度
#define BUFFER_ENERGY_NORMAL        40.0f   // 正常缓冲能量
#define BUFFER_ENERGY_HIGH          50.0f   // 充足缓冲能量：允许短暂超速

// --------------------------------------
// 速度比例系数限制
// --------------------------------------
#define SPEED_SCALE_MIN           0.5f      // 最低速度比例
#define SPEED_SCALE_MAX           1.0f      // 纯电池模式下最高允许的速度比例


// 函数声明
void ChassisPowerCtrlInit(void);
void ChassisPowerCtrlUpdate(void);
float GetPowerControlSpeedScale(void);
bool GetChassisSpinState(void);

#endif

