/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       SupCap.c/h
  * @brief      超级电容相关部分定义
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-29-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef SUPCAP_H
#define SUPCAP_H

#include "struct_typedef.h"
#include "stdbool.h"

typedef enum {
    SUP_CAP_1 = 1,
    SUP_CAP_2,
} SupCapType_e;

typedef struct
{
    int16_t voltage_in;
    int16_t voltage_cap;
    int16_t current_in;
    int16_t power_target;

    uint32_t last_fdb_time;  //上次反馈时间
} SupCapMeasure_s;

/** 
 * @brief        通用超级电容结构体
 * @note         包括电容的信息、状态量和控制量
 */
typedef struct
{
    uint8_t id;         // 超级电容ID
    SupCapType_e type;  // 超级电容类型
    uint8_t can;        // 超级电容所用CAN口
    bool offline;       // 超级电容是否离线 0-在线 1-离线

    struct
    {
        float voltage_in;    // (V)输入电压
        float voltage_cap;   // (V)电容电压
        float current_in;    // (A)输入电流
        float power_target;  // (W)目标功率
    } fdb;

    struct
    {
        float power;  // (W)目标功率
    } set;
} SupCap_s;

extern void SupCapInit(SupCap_s * p_sup_cap, uint8_t can);

#endif
/************************ END OF FILE ************************/
