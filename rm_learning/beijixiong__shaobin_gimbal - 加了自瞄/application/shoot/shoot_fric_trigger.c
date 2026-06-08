/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_fric.c/h
  * @brief      使用摩擦轮的发射机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.1.0     2025-1-15       CJH             1. 实现基本功能
  *  V2.0.0     2025-3-3        CJH             1. 兼容了达妙4310拨弹盘和大疆2006拨弹盘
  *                                             2. 完善了单发功能，上位机火控功能
  *                                             3. 增加了热量限制
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "shoot_fric_tirgger.h"
#include "VOFA.h"
#include "usb_task.h"
#include "CAN_receive.h"

#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)

	Shoot_s SHOOT = {
  .mode = LOAD_STOP,
  .state = FRIC_NOT_READY,
  .fric_flag = 0,
  .move_flag = 0,
  .ecd_count = 0,
  .shoot_flag = 0,
  .heat = 0,
  .heat_limit = 0,
};


uint8_t fric_ui;
fp32 delta;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */

void ShootInit(void)
{
	//获取遥控器指针
  SHOOT.rc = get_remote_control_point(); 

  //摩擦轮相关
  MotorInit(&SHOOT.fric_motor[0],FRIC_MOTOR_R_ID, FRIC_MOTOR_R_CAN, FRIC_MOTOR_TYPE, 1, 1.0f, 0);//初始化R摩擦轮电机结构体
  MotorInit(&SHOOT.fric_motor[1],FRIC_MOTOR_L_ID, FRIC_MOTOR_L_CAN, FRIC_MOTOR_TYPE, 1, 1.0f, 0);//初始化L摩擦轮电机结构体

  const fp32 pid_fric[3] = {FRIC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FRIC_SPEED_PID_KD};//摩擦轮速度环

  PID_init(&SHOOT.fric_pid[0], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
  PID_init(&SHOOT.fric_pid[1], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);//摩擦轮初始化pid

  //拨弹盘相关
  MotorInit(&SHOOT.trigger_motor,TRIGGER_MOTOR_ID, TRIGGER_MOTOR_CAN, TRIGGER_MOTOR_TYPE, 1, 1.0f, 0);//初始化拨弹盘电机结构体

  const fp32 pid_angel_trigger[3] = {TRIGGER_ANGEL_PID_KP, TRIGGER_ANGEL_PID_KI, TRIGGER_ANGEL_PID_KD};//拨弹盘角度环
  const fp32 pid_speed_trigger[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};//拨弹盘速度环

  PID_init(&SHOOT.trigger_angel_pid, PID_POSITION, pid_angel_trigger, TRIGGER_ANGEL_PID_MAX_OUT, TRIGGER_ANGEL_PID_MAX_IOUT);
  PID_init(&SHOOT.trigger_speed_pid, PID_POSITION, pid_speed_trigger, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);  //拨弹盘初始化pid

	
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */

void ShootSetMode(void)
{
	// 【新增】获取视觉指令
    VisionCmdData_t vision_data = GetVisionCmdData();
	
	if (switch_is_up(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))//上档防止误触
	{

//		SHOOT.state = FRIC_READY;
//		if(SHOOT.rc->rc.ch[4] > 300)
//		{
//			SHOOT.mode = LOAD_BURSTFIRE;
//		}
//		else
//		{
//			SHOOT.mode = LOAD_STOP;
        // 在上档时，默认强制开启摩擦轮，时刻准备射击
        SHOOT.state = FRIC_READY;
        
        // 【核心逻辑】：判断是否按住了鼠标右键
        if (SHOOT.rc->mouse.press_r) 
        {
            // 按住右键时，完全由视觉指令接管拨盘开火权
            if (vision_data.shoot_cmd == 1)
            {
                SHOOT.mode = LOAD_BURSTFIRE;   // 视觉锁定，自动连发
            }
            else 
            {
                SHOOT.mode = LOAD_STOP;        // 视觉未锁定，停火等待
            }
		}
	
	}
	else if(switch_is_mid(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))
{
    // 第一步：键鼠(Q/E)控制摩擦轮（连发的前提，摩擦轮就绪才能有效连发）
    if(SHOOT.rc->key.v & KEY_PRESSED_OFFSET_Q) // Q键：启动摩擦轮（连发必备）
    {
        SHOOT.fric_flag = 1; // 置位摩擦轮启动标志
    }
    else if(SHOOT.rc->key.v & KEY_PRESSED_OFFSET_E) // E键：关闭摩擦轮（停止连发保障）
    {
        SHOOT.fric_flag = 0; // 复位摩擦轮启动标志
    }

    // 第二步：更新摩擦轮状态（就绪/未就绪）
    if (SHOOT.fric_flag)
    {
        SHOOT.state = FRIC_READY; // 摩擦轮就绪，可进行连发
    }
    else
    {
        SHOOT.state = FRIC_NOT_READY; // 摩擦轮未就绪，无法连发
        SHOOT.mode = LOAD_STOP; // 强制置为停止模式，避免无效操作
    }

    // 第三步：键鼠触发连发（核心逻辑，仅保留连发/停止两种模式）
    if (SHOOT.state == FRIC_READY) // 只有摩擦轮就绪时，才响应连发指令
    {
        // 方式1：鼠标右键按下 → 连发，松开 → 停止（和你原有代码操作一致）
        if (SHOOT.rc->mouse.press_l)
        {
            SHOOT.mode = LOAD_BURSTFIRE; // 进入连发模式
        }
        else
        {
            SHOOT.mode = LOAD_STOP; // 退出连发，置为停止模式
        }
    }
}
	else if(switch_is_down(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))
	{
		SHOOT.state = FRIC_NOT_READY;
        SHOOT.mode = LOAD_STOP;
	}
    // ---------------- 【新增】视觉控制逻辑覆盖 ----------------
//	 if (!switch_is_down(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])) 
//    {

//        // 如果视觉指令为 0，则不做处理，维持上面的遥控器逻辑（即“自主发弹”）
//    }

	//安全档
    if ((switch_is_down(SHOOT.rc->rc.s[0])))
    {
        SHOOT.mode = LOAD_STOP;
        SHOOT.state = FRIC_NOT_READY;
    }
	
	    //遥控器离线保护
    if ( toe_is_error(DBUS_TOE) )
    {        
        SHOOT.state = FRIC_NOT_READY;
        SHOOT.mode = LOAD_STOP;
    }
	// 1. 通过CAN接口获取下板发来的最新热量数据
    get_shoot_heat_from_can(&SHOOT.heat_limit, &SHOOT.heat);
	
	if (SHOOT.heat_limit != 0 && (SHOOT.heat + SHOOT_HEAT_REMAIN_VALUE) > SHOOT.heat_limit)
    {
        SHOOT.mode = LOAD_STOP;
    }
}	
/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */

void ShootObserver(void)
{
	GetMotorMeasure(&SHOOT.trigger_motor);
	GetMotorMeasure(&SHOOT.fric_motor[0]);
	GetMotorMeasure(&SHOOT.fric_motor[1]);
	
	SHOOT.FDB.fric_speed_fdb_R = SHOOT.fric_motor[0].fdb.vel;
	SHOOT.FDB.fric_speed_fdb_L = SHOOT.fric_motor[1].fdb.vel;
	
	SHOOT.FDB.trigger_speed_fdb = SHOOT.trigger_motor.fdb.vel;
	
	if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd > HALF_ECD_RANGE)
    {
        SHOOT.ecd_count--;
    }
    else if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd < -HALF_ECD_RANGE)
    {
        
        SHOOT.ecd_count++;
    }

    if (SHOOT.ecd_count == FULL_COUNT)
    {
        SHOOT.ecd_count = -(FULL_COUNT - 1);
    }
    else if (SHOOT.ecd_count == -FULL_COUNT)
    {
        SHOOT.ecd_count = FULL_COUNT-1;
    }
    //计算输出轴角度
    SHOOT.FDB.trigger_angel_fdb = (SHOOT.ecd_count * ECD_RANGE + SHOOT.trigger_motor.fdb.ecd )* MOTOR_ECD_TO_ANGLE;

    //记录上一个ecd值
   SHOOT.last_ecd = SHOOT.trigger_motor.fdb.ecd;
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */

void ShootReference(void)
{
	switch (SHOOT.state)
	{
	case FRIC_NOT_READY:
	SHOOT.REF.fric_speed_ref_R=0.0f;
	SHOOT.REF.fric_speed_ref_L=0.0f;
	break;
	
	case FRIC_READY:
	SHOOT.REF.fric_speed_ref_R=FRIC_R_SPEED;
	SHOOT.REF.fric_speed_ref_L=FRIC_L_SPEED;
	break;
	
	default:
	break;
	}
	
	switch (SHOOT.mode)
	{
	case LOAD_STOP:
	SHOOT.REF.trigger_speed_ref=0.0f;
	break;
		
	case LOAD_BURSTFIRE:
	SHOOT.REF.trigger_speed_ref = TRIGGER_SPEED;
	break;
		
	default:
	break;
	}
}
/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ShootConsole(void)
{
	SHOOT.fric_motor[0].set.curr= PID_calc(&SHOOT.fric_pid[0], SHOOT.FDB.fric_speed_fdb_R,SHOOT.REF.fric_speed_ref_R);
	SHOOT.fric_motor[1].set.curr= PID_calc(&SHOOT.fric_pid[1], SHOOT.FDB.fric_speed_fdb_L,SHOOT.REF.fric_speed_ref_L);

	if (SHOOT.mode == LOAD_STOP)
    {
        SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid, SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
    }
    else if (SHOOT.mode == LOAD_BURSTFIRE)
    {
        SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid, SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
    }
	//VOFA调试
//	JustFloat(SHOOT.REF.fric_speed_ref_R,SHOOT.FDB.fric_speed_fdb_R,0,0);         
//	JustFloat(SHOOT.REF.trigger_speed_ref,SHOOT.FDB.trigger_speed_fdb,0,0);	
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ShootSendCmd(void)
{
	 CanCmdDjiMotor(FRIC_MOTOR_R_CAN, STD_ID , SHOOT.fric_motor[0].set.curr,0,SHOOT.trigger_motor.set.curr,SHOOT.fric_motor[1].set.curr );
}

#endif  // SHOOT_TYPE == SHOOT_FRIC
