#include "chassis_task.h"

#include "attribute_typedef.h"
//#include "chassis_omni.h"
#include "cmsis_os.h"
#include "VOFA.h"
#include "CAN_communication.h"

#ifndef CHASSIS_TASK_INIT_TIME
#define CHASSIS_TASK_INIT_TIME 357
#endif  // CHASSIS_TASK_INIT_TIME

#ifndef CHASSIS_CONTROL_TIME_MS
#define CHASSIS_CONTROL_TIME_MS 2
#endif  // CHASSIS_CONTROL_TIME_MS

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif


__weak void ChassisPublish(void);
__weak void ChassisInit(void);
__weak void ChassisHandleException(void);
__weak void ChassisSetMode(void);
__weak void ChassisObserver(void);
__weak void ChassisReference(void);
__weak void ChassisConsole(void);
__weak void ChassisSendCmd(void);

/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */

void chassis_task(void const * pvParameters)
{
	ChassisPublish();
	//空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	// 初始化底盘
    ChassisInit();
	
	while (1) {
		//发送裁判系统数据
		CanSendRefereeDataToBoard(1,BoardC2,0);
        // 更新状态量
        ChassisObserver();
        // 处理异常
        ChassisHandleException();
        // 设置底盘模式
        ChassisSetMode();
        // 更新目标量
        ChassisReference();
        // 计算控制量
        ChassisConsole();
        // 发送控制量
        ChassisSendCmd();
        // 系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

__weak void ChassisPublish(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisInit(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisHandleException(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisSetMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisObserver(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisReference(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisConsole(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisSendCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

/*------------------------------ Calibrate Function ------------------------------*/

/**
  * @brief          设置底盘校准值，将底盘的校准数据设置为传入的校准值
  * @param[in]      motor_middle:电机中值
  * @retval         返回空
  * @note           底盘任务内部调用的函数
  */
__weak void ChassisSetCaliData(const fp32 motor_middle[4])
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

/**
  * @brief          底盘校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     motor_middle:电机中值
  * @retval         返回空
  * @note           底盘任务内部调用的函数
  */
__weak bool_t ChassisCmdCali(fp32 motor_middle[4])
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
    static uint32_t cnt = 0;
    cnt++;
    if (cnt > 1000) {
        motor_middle[0] = 0.0f;
        motor_middle[1] = 0.0f;
        motor_middle[2] = 0.0f;
        motor_middle[3] = 0.0f;
        cnt = 0;
        return 1;
    } else {
        return 0;
    }
}

/**
  * @brief          底盘校准设置，将校准的底盘中值以及最小最大机械相对角度
  * @param[in]      motor_middle:电机中值
  * @retval         返回空
  * @note           提供给校准任务调用的钩子函数
  */
void set_cali_chassis_hook(const fp32 motor_middle[4]) { ChassisSetCaliData(motor_middle); }

/**
  * @brief          底盘校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     motor_middle:电机中值
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @note           提供给校准任务调用的钩子函数
  */
bool_t cmd_cali_chassis_hook(fp32 motor_middle[4]) { return ChassisCmdCali(motor_middle); }

/*------------------------------ End of File ------------------------------*/

