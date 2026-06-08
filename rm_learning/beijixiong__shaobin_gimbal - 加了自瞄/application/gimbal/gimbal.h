/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal.c/h
  * @brief      云台控制任务所需要的变量和函数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef GIMBAL_H
#define GIMBAL_H

#include "robot_param.h"
#include "struct_typedef.h"
#include "stdbool.h"

// API

extern inline uint8_t GetGimbalStatus(void);
extern inline uint32_t GetGimbalDuration(void);
extern inline float GetGimbalSpeed(uint8_t axis);
extern inline float GetGimbalVelocity(uint8_t axis);
extern inline float GetGimbalDeltaYawMid(void);
extern inline bool GetGimbalInitJudgeReturn(void);
extern inline float CmdGimbalJointState(uint8_t axis);

#if GIMBAL_TYPE != GIMBAL_NONE

#endif  // GIMBAL_TYPE
#endif  // GIMBAL_H

