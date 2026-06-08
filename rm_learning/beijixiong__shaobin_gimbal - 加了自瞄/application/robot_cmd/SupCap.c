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

#include "SupCap.h"

#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"

/**
 * @brief       超级电容初始化
 * @param[in]   p_sup_cap 超级电容结构体
 * @param[in]   can 超级电容使用的can口
 */
void SupCapInit(SupCap_s * p_sup_cap, uint8_t can)
{
    p_sup_cap->id = 1;
    p_sup_cap->type = SUP_CAP_1;
    p_sup_cap->can = can;
    p_sup_cap->offline = true;

    memset(&p_sup_cap->fdb, 0, sizeof(p_sup_cap->fdb));
    memset(&p_sup_cap->set, 0, sizeof(p_sup_cap->set));
}
/************************ END OF FILE ************************/
