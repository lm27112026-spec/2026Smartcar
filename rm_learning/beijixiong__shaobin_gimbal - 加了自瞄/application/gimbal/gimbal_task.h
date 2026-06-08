/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task
  *             完成云台控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024     Penguin          1. done
  *  V1.0.1     Apr-16-2024    Penguin          1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
  
#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H  
  
#include "struct_typedef.h"  
#include "CAN_communication.h"
  
/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */  
  
extern void gimbal_task(void const * pvParameters);

extern void set_cali_gimbal_hook(
    const fp32 yaw_middle, const fp32 pitch_horizontal, const fp32 max_pitch, const fp32 min_pitch);

extern bool_t cmd_cali_gimbal_hook(
    fp32 * yaw_middle, fp32 * pitch_horizontal, fp32 * max_pitch, fp32 * min_pitch);

#endif


