/**
  ****************************(C) COPYRIGHT 2025****************************
  * @file       chassis_power_control.c
  * @brief      底盘功率控制（纯电池/裁判系统版）
  * @note       通过监测裁判系统的缓冲能量来动态输出速度限制系数，防止超功率掉血
  **************************************************************************
  */
#include "chassis_power_control.h"
#include "chassis_omni.h"
#include "VOFA.h"

PowerLimit_t power_limit; // 功率控制全局实例

// 外部引用的裁判系统数据
extern robot_status_t robot_status;       // 机器人状态（包含上限功率）
extern power_heat_data_t power_heat_data; // 功率热量数据（包含缓冲能量）

/**
 * @brief 底盘功率控制初始化
 */
void ChassisPowerCtrlInit(void)
{
    power_limit.max_referee_power = INFANTRY_BATTLE_POWER;
    power_limit.buffer_energy = 60.0f; // 默认满缓冲
}

/**
 * @brief 获取底盘速度限制系数（核心防掉血逻辑）
 * @return 速度比例系数（0.5 - 1.0）
 */
float GetPowerControlSpeedScale(void)
{
    float scale = 1.0f;
    float current_buffer = power_limit.buffer_energy;
    float ref_power = power_limit.max_referee_power;

    // =========================================================
    // 第一步：根据裁判系统的工作功率上限，确定当前模式的“基础效能”
    // =========================================================
    float base_scale = 1.0f;
    
    if (ref_power >= INFANTRY_BATTLE_POWER - 5.0f) 
    {
        // 120W 模式
        base_scale = 1.0f;  // 原本是1.2，为了防超功率，严格限制在1.0
    } 
    else if (ref_power >= THREE_VS_THREE_POWER_HIGH - 5.0f) 
    {
        // 90W 模式 (3V3 高功率)
        base_scale = 1.0f;  
    } 
    else if (ref_power >= THREE_VS_THREE_POWER_LOW - 5.0f) 
    {
        // 75W 模式 (3V3 低血状态)
        base_scale = 0.85f; 
    } 
    else 
    {
        // 未知/异常低功率（主板重启或断线）
        base_scale = 0.7f;
    }

    // =========================================================
    // 第二步：根据剩余缓冲能量，进行平滑或急剧的降速压制
    // =========================================================
    if (current_buffer > BUFFER_ENERGY_NORMAL) 
    {
        // 缓冲充足 (>40J)，保持基础输出，不再超频放大！
        scale = base_scale;  
    } 
    else if (current_buffer > BUFFER_ENERGY_LOW) 
    {
        // 缓冲降低 (25~40J)，线性衰减 (降到 70%~100%)
        float attenuation = 0.7f + 0.3f * ((current_buffer - BUFFER_ENERGY_LOW) / (BUFFER_ENERGY_NORMAL - BUFFER_ENERGY_LOW));
        scale = base_scale * attenuation;
    } 
    else if (current_buffer > BUFFER_ENERGY_CRITICAL_LOW) 
    {
        // 缓冲见底警告 (15~25J)，强力衰减 (降到 40%~70%)
        float attenuation = 0.4f + 0.3f * ((current_buffer - BUFFER_ENERGY_CRITICAL_LOW) / (BUFFER_ENERGY_LOW - BUFFER_ENERGY_CRITICAL_LOW));
        scale = base_scale * attenuation;
    } 
    else 
    {
        // 缓冲极危 (<15J)，强制进入最低速度模式
        scale = SPEED_SCALE_MIN; 
    }

    // =========================================================
    // 第三步：最终安全钳制（防止算术错误导致失控）
    // =========================================================
    if (scale < SPEED_SCALE_MIN) scale = SPEED_SCALE_MIN;
    if (scale > SPEED_SCALE_MAX) scale = SPEED_SCALE_MAX;
    
    return scale;
}

/**
 * @brief 底盘功率控制更新任务（放入原先电容任务或者底盘任务中调用）
 */
void ChassisPowerCtrlUpdate(void)
{
    // 1. 更新裁判系统数据
    if (robot_status.chassis_power_limit > 0) 
	{
        power_limit.max_referee_power = robot_status.chassis_power_limit;
		power_limit.buffer_energy = power_heat_data.buffer_energy;
    }
    else 
    {
        // 裁判系统离线（实验室裸车调试），默认给120W和满血，保证正常行走
        power_limit.max_referee_power = INFANTRY_BATTLE_POWER;
        power_limit.buffer_energy = 60.0f;
    }

    // VOFA 调参观察：输出目标功率上限、缓冲能量、当前计算出的速度缩放系数
//    JustFloat(power_limit.max_referee_power, 
//              power_limit.buffer_energy,
//              GetPowerControlSpeedScale(),
//              0);
}

