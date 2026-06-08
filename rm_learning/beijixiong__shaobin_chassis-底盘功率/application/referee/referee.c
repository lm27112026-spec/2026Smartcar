/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       referee.c/h
  * @brief      Updated the referee system Communication Protocol 1.6.1
  * @note       裁判系统通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024/3/6         YZX             1.更新裁判系统通信协议至1.6.1（不包含半自动步兵和UI）
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#include "referee.h"

#include "CRC8_CRC16.h"
//#include "bsp_buzzer.h"
#include "detect_task.h"
#include "protocol.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "string.h"

#define REFEREE_TIMEOUT 300  //裁判系统超时时间
uint32_t referee_online_time = 0;
uint32_t referee_receive_count = 0;

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_status_t game_status;					//比赛状态数据（0x0001）
game_result_t game_result;                  //比赛结果数据（0x0002）
game_robot_HP_t game_robot_HP;              //机器人存活数据（0x0003）

event_data_t field_event;										//场地时事件数据（0x0101）
ext_supply_projectile_action_t supply_projectile_action_t;      //补给站动作标识（0x0102）
ext_supply_projectile_booking_t supply_projectile_booking_t;    //补给站预约子弹（0x0103）
referee_warning_t referee_warning;								//裁判系统警告信息（0x0105）

robot_status_t robot_status;							//比赛机器人状态（0x0201）
power_heat_data_t power_heat_data;			            //实时功率热量数据（0x0202）
robot_pos_t game_robot_pos_t;				            //机器人位置（0x0203）
buff_t buff_musk_t;                                     //机器人增益（0x0204）
air_support_data_t robot_energy_t;                      //空中机器人能量状态（0x0205）
hurt_data_t robot_hurt_t;                               //伤害状态（0x0206）
shoot_data_t shoot_data;                                //实时射击信息（0x0207）
ext_bullet_remaining_t bullet_remaining_t;              //子弹剩余发射数（0x0208）
robot_interaction_data_t student_interactive_data_t;	//交互数据接收信息（0x0301）
CustomControllerData_t CUSTOM_CONTROLLER_DATA;  		//自定义控制器数据

ext_robot_command_t robot_command_t;					//键鼠控制指令

/*为各个结构体分配对应大小的空间*/
void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_status, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));

    memset(&field_event, 0, sizeof(event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));

    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(buff_t));
    memset(&robot_energy_t, 0, sizeof(air_support_data_t));
    memset(&robot_hurt_t, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));

    memset(&student_interactive_data_t, 0, sizeof(robot_interaction_data_t));
    memset(&CUSTOM_CONTROLLER_DATA, 0, sizeof(CustomControllerData_t));

    memset(&robot_command_t, 0, sizeof(ext_robot_command_t));
}

/*裁判系统数据解算*/
void referee_data_solve(uint8_t * frame)
{
    if (HAL_GetTick() - referee_online_time > REFEREE_TIMEOUT) {
        referee_receive_count = 0;
    }
    referee_receive_count++;

    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id) {
        case GAME_STATE_CMD_ID: {
            memcpy(&game_status, frame + index, sizeof(game_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GAME_RESULT_CMD_ID: {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GAME_ROBOT_HP_CMD_ID: {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
            referee_online_time = HAL_GetTick();
        } break;

        case FIELD_EVENTS_CMD_ID: {
            memcpy(&field_event, frame + index, sizeof(event_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID: {
            memcpy(
                &supply_projectile_action_t, frame + index, sizeof(ext_supply_projectile_action_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID: {
            memcpy(
                &supply_projectile_booking_t, frame + index,
                sizeof(ext_supply_projectile_booking_t));
            referee_online_time = HAL_GetTick();
        } break;
        case REFEREE_WARNING_CMD_ID: {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
            referee_online_time = HAL_GetTick();
        } break;

        case ROBOT_STATE_CMD_ID: {
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case POWER_HEAT_DATA_CMD_ID: {
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case ROBOT_POS_CMD_ID: {
            memcpy(&game_robot_pos_t, frame + index, sizeof(robot_pos_t));
            referee_online_time = HAL_GetTick();
        } break;
        case BUFF_MUSK_CMD_ID: {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_t));
            referee_online_time = HAL_GetTick();
        } break;
        case AERIAL_ROBOT_ENERGY_CMD_ID: {
            memcpy(&robot_energy_t, frame + index, sizeof(air_support_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case ROBOT_HURT_CMD_ID: {
            memcpy(&robot_hurt_t, frame + index, sizeof(hurt_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SHOOT_DATA_CMD_ID: {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case BULLET_REMAINING_CMD_ID: {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
            referee_online_time = HAL_GetTick();
        } break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID: {
            memcpy(&student_interactive_data_t, frame + index, sizeof(robot_interaction_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case CUSTOM_CONTROLLER_CMD_ID: {
            memcpy(&CUSTOM_CONTROLLER_DATA, frame + index, sizeof(CustomControllerData_t));
            referee_online_time = HAL_GetTick();
        } break;
        case ROBOT_COMMAND_CMD_ID: {
            memcpy(&robot_command_t, frame + index, sizeof(ext_robot_command_t));
            referee_online_time = HAL_GetTick();
        } break;
        default: {
            referee_receive_count--;
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 * power, fp32 * buffer)
{
    *power = 0;//power_heat_data.chassis_power;
    *buffer = power_heat_data.buffer_energy;
}

uint8_t get_robot_id(void) { return robot_status.robot_id; }

void get_shoot_heat0_limit_and_heat0(uint16_t * heat0_limit, uint16_t * heat0)
{
    *heat0_limit = robot_status.shooter_barrel_heat_limit;
    *heat0 = power_heat_data.shooter_17mm_1_barrel_heat;
}

void get_shoot_heat1_limit_and_heat1(uint16_t * heat1_limit, uint16_t * heat1)
{
    *heat1_limit = robot_status.shooter_barrel_heat_limit;
    *heat1 = power_heat_data.shooter_17mm_2_barrel_heat;  // 第 2 个 17mm 发射机构的枪口热量
}

void get_shoot_heat42_limit_and_heat42(uint16_t * heat_limit, uint16_t * heat)
{
    *heat_limit = robot_status.shooter_barrel_heat_limit;
    *heat = power_heat_data.shooter_42mm_barrel_heat;
}

/**
 * @brief 反馈机器人颜色
 * @param  none
 * @return 0为红色,1为蓝色,2为未知
 */
uint8_t get_team_color(void)  // 谨防“哨兵在打我”
{
    switch (robot_status.robot_id) {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
            return 0;
        case 101:
        case 102:
        case 103:
        case 104:
        case 105:
        case 106:
        case 107:
        case 108:
        case 109:
        case 110:
        case 111:
            return 1;
        default:
            return 2;
    }
}

uint16_t get_shoot_heat(void)  // 双枪管哨兵
{
    if (power_heat_data.shooter_17mm_1_barrel_heat > power_heat_data.shooter_17mm_2_barrel_heat) {
       return power_heat_data.shooter_17mm_1_barrel_heat;
    } else {
        return power_heat_data.shooter_17mm_2_barrel_heat;
    }
}

CustomControllerData_t * GetCustomControllerDataPoint(void) { return &CUSTOM_CONTROLLER_DATA; }

/*========== API ==========*/

inline bool GetRefereeOffline(void)
{
    uint32_t current_time = HAL_GetTick();
    return !((referee_receive_count > 5) && (current_time - referee_online_time < REFEREE_TIMEOUT));
}

/**
 * @brief 获取自定义控制器数据
 * @param  index 数据索引
 * @return float 数据
 */
inline float GetCustomControllerPos(uint8_t index)
{
    float data = 0;
    memcpy(&data, &CUSTOM_CONTROLLER_DATA.data[index * 4], 4);
    return data;
    // return *((float *)(&CUSTOM_CONTROLLER_DATA.data[index * 4]));
}

/*------------------------------ End of File ------------------------------*/
