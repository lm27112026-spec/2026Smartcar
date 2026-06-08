/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *   V1.1.0    2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *   V1.1.1    2024-11-11    Harry_Wong        1.为云台随动添加了yaw轴偏转角度的API
  *   V1.1.2    2024-11-25    Harry_Wong        1.云台模式设置逻辑重构，准备函数给底盘表明是否处于初始化模式
  *   V1.1.3    2024-12-16    Harry_Wong        1.云台的imu获取方式被更改为函数传递，防止Subcribe（）函数停用造成影响 
                                                2.云台在遥控器断联情况下直接发送0电流，防止赛场上出现意外情况影响稳定性
  *   V1.2.0    2025-2-25     Harry_Wong        1.向上位机发送数据进行了优化，采用了电机角度差值控制：（电机实际角度 - 电机中值角度）* 电机旋转方向 （距离中间的偏移角度）
                                                2.添加了Gimbal_direct_ecd_to_imu（）函数 接受上位机返回的目标角度，并且跟发送角度做差得到角度期望偏移值并映射成为云台imu的期望角度值
                                                3. Reference（） AUTO_AIM 模式下的reference计算方法进行了更改：将返回值进行Gimbal_direct_ecd_to_imu（）运算得到结果作为云台的目标角度
                                                4. 删除了 imu_base 变量 ，现在云台已经不需要在第一次校准的时候记录imu初始值了
                                                5.Console() INIT 模式下的PID计算优化：所有云台的旋转计算参考值已经更改为imu相关数据，尽可能的减轻调教负担
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/

#include "gimbal_yaw_pitch_direct.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
Gimbal_s gimbal_direct;
PID_t gimbal_direct_pid;
PID_t gimbal_aim_pid;	// 自瞄专用PID控制器

/*--------------------------------Internal functions---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/

/*----------------angle_solution--------------------*/

/**
 * @brief          解算imu和电机角度的差值（用于更新云台的限位范围）,确认新的电机中值对应的imu值
 * @param[in]      none
 * @retval         none
 */
void Angle_solution(void)
{
	float motor_feedback=gimbal_direct.pitch.fdb.pos;
	float imu_feedback=gimbal_direct.feedback_pos.pitch;
	float motor_mid=GIMBAL_DIRECT_PITCH_MID;
	float imu_mid=0.0;
	float motor_delta=GIMBAL_DIRECT_PITCH_DIRECTION*(motor_feedback-motor_mid);
	float imu_delta=imu_feedback-imu_mid;
	gimbal_direct.angle_zero_for_imu=imu_delta-motor_delta;
}

/*----------------Gimbal_direct_init_judge--------------------*/
/**
 * @brief          判断是否需要继续初始化云台校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */
bool Gimbal_direct_init_judge (void)
{
	if ( ((gimbal_direct.reference.yaw-gimbal_direct.yaw.fdb.pos<0.003f && (-0.003f)<gimbal_direct.reference.yaw-gimbal_direct.yaw.fdb.pos) && (gimbal_direct.reference.pitch-gimbal_direct.pitch.fdb.pos<0.003f && (-0.003f)<gimbal_direct.reference.pitch-gimbal_direct.pitch.fdb.pos) ) || gimbal_direct.init_timer>=GIMBAL_INIT_TIME )
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*----------------Gimbal_direct_ecd_to_imu--------------------*/
/**
 * @brief          ecd角度值转换成imu角度值
 * @param[in]      axis 用于知道读取哪一个轴的角度转换
 * @param[in]      value 用于准换的值
 * @retval         float imu映射角度值
 */
float Gimbal_direct_ecd_to_imu(uint8_t axis,float value)
{
	if (axis == AX_PITCH)
	{
		return value - CmdGimbalJointState(AX_PITCH) + gimbal_direct.feedback_pos.pitch;
	}
	else if(axis == AX_YAW)
	{
		return value - CmdGimbalJointState(AX_YAW) + gimbal_direct.feedback_pos.yaw;
	}
	else 
	{
		return 0.0f;
	}
}

/**
 * @brief          从USB任务获取视觉数据并处理 (核心函数)
 */
void Gimbal_direct_process_vision_data(void)
{
    // 1. 调用 usb_task.c 提供的接口获取解析后的数据结构
    VisionCmdData_t vision_data = GetVisionCmdData();
    
    // 2. 更新控制标志位
    // control_mode: 0=不控制, 1=控制
    gimbal_direct.vision_control_enabled = (vision_data.control_mode > 0);
    
    // 3. 更新射击指令 (0xCC帧中的 shoot 字段)
    // 兼容两种逻辑：control_mode==2 或者 shoot_cmd > 0
    if (vision_data.control_mode == 2 || vision_data.shoot_cmd > 0) {
        gimbal_direct.shoot_cmd = 1;
    } else {
        gimbal_direct.shoot_cmd = 0;
    }
    
    // 4. 更新距离 (0xCC帧中的 distance 字段)
    gimbal_direct.target_distance = vision_data.distance;
    
    // 5. 更新目标角度 (usb_task已经将int16转换为float弧度)
    gimbal_direct.vision_reference.yaw = vision_data.yaw;
    gimbal_direct.vision_reference.pitch = vision_data.pitch;
}

/**
 * @brief          自瞄模式控制逻辑
 */
void Gimbal_direct_auto_aim_control(void)
{
    // 获取最新视觉数据
    Gimbal_direct_process_vision_data();
    
    if (gimbal_direct.vision_control_enabled)
    {
        // --- Pitch 轴控制 (带软限位) ---
        // 加上零点偏移 gimbal_direct.angle_zero_for_imu
        float target_pitch = gimbal_direct.vision_reference.pitch;
        
        gimbal_direct.reference.pitch = fp32_constrain(
            target_pitch,
            gimbal_direct.lower_limit.pitch + gimbal_direct.angle_zero_for_imu,
            gimbal_direct.upper_limit.pitch + gimbal_direct.angle_zero_for_imu
        );

        // --- Yaw 轴控制 (循环限位) ---
        gimbal_direct.reference.yaw = loop_fp32_constrain(
            gimbal_direct.vision_reference.yaw,
            -M_PI, M_PI
        );
    }
    else
    {
        // 视觉虽开启自瞄模式但未发送有效控制位(control=0)，保持当前姿态
        gimbal_direct.reference.pitch = gimbal_direct.feedback_pos.pitch;
        gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
    }
}

/*-------------------------The end of internal functions--------------------------------------*/

/* ---------------- GetGimbalDeltaYawMid -------------------- */

/**
 * @brief          (rad) 获取yaw轴和中值的差值
 * @param[in]      none
 * @retval         float
 */
inline float GetGimbalDeltaYawMid(void)
{
	  return loop_fp32_constrain(gimbal_direct.yaw.fdb.pos-GIMBAL_DIRECT_YAW_MID,-M_PI,M_PI);
}

/* ---------------- GetGimbalInitJudgeReturn -------------------- */

/**
 * @brief          对外宣称自己是否继续校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */
inline bool GetGimbalInitJudgeReturn(void)
{
	return gimbal_direct.init_continue;
}

/* --------------------- CmdGimbalJointState ------------------- */

/**
 * @brief          返回云台的imu基准值
 * @param[in]      uint8_t 轴id
 * @retval         云台的基准值返回 （float)
 */
inline float CmdGimbalJointState(uint8_t axis)//以预设固定中点为基准，算相对偏移（含方向修正，固定基准）
{
	if ( axis == AX_PITCH )
	{
		return loop_fp32_constrain(gimbal_direct.pitch.direction * (gimbal_direct.pitch.fdb.pos - GIMBAL_DIRECT_PITCH_MID),-M_PI,M_PI);
	}
	else if ( axis == AX_YAW )
	{
		return loop_fp32_constrain(gimbal_direct.yaw.direction * (gimbal_direct.yaw.fdb.pos - GIMBAL_DIRECT_YAW_MID),-M_PI,M_PI); 
	}
	else 
	{
		return 0.0;
	}
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void GimbalInit(void)
{
	//step1 获取所有所需变量指针
	gimbal_direct.rc = get_remote_control_point(); 
	//step2 置零所有值
    gimbal_direct.reference.pitch=0;
    gimbal_direct.reference.yaw=0;
	
	gimbal_direct.feedback_pos.pitch=0;
    gimbal_direct.feedback_pos.yaw=0;
	
	gimbal_direct.feedback_vel.pitch=0;
    gimbal_direct.feedback_vel.yaw=0;
	
	gimbal_direct.upper_limit.pitch=GIMBAL_UPPER_LIMIT_PITCH;
	gimbal_direct.lower_limit.pitch=GIMBAL_LOWER_LIMIT_PITCH;
    //step3 PID数据清零，设置PID参数	
	const static fp32 gimbal_yaw_angle[3]={KP_GIMBAL_YAW_ANGLE,KI_GIMBAL_YAW_ANGLE,KD_GIMBAL_YAW_ANGLE};
	
	const static fp32 gimbal_pitch_angle[3] = {KP_GIMBAL_PITCH_ANGLE, KI_GIMBAL_PITCH_ANGLE, KD_GIMBAL_PITCH_ANGLE};
	const static fp32 gimbal_pitch_velocity[3] = {KP_GIMBAL_PITCH_VELOCITY, KI_GIMBAL_PITCH_VELOCITY, KD_GIMBAL_PITCH_VELOCITY};
  
	PID_init(&gimbal_direct_pid.yaw_angle,PID_POSITION,gimbal_yaw_angle,MAX_OUT_GIMBAL_YAW_ANGLE,MAX_IOUT_GIMBAL_YAW_ANGLE);
	
	PID_init(&gimbal_direct_pid.pitch_angle, PID_POSITION, gimbal_pitch_angle, MAX_OUT_GIMBAL_PITCH_ANGLE, MAX_IOUT_GIMBAL_PITCH_ANGLE);
	PID_init(&gimbal_direct_pid.pitch_velocity, PID_POSITION, gimbal_pitch_velocity, MAX_OUT_GIMBAL_PITCH_VELOCITY, MAX_IOUT_GIMBAL_PITCH_VELOCITY);
	
	//自瞄PID数据清零，设置PID参数
	const static fp32 aim_pitch_angle[3] = {KP_AIM_PITCH_ANGLE, KI_AIM_PITCH_ANGLE, KD_AIM_PITCH_ANGLE};
	const static fp32 aim_pitch_velocity[3] = {KP_AIM_PITCH_VELOCITY, KI_AIM_PITCH_VELOCITY, KD_AIM_PITCH_VELOCITY};
	
	PID_init(&gimbal_aim_pid.pitch_angle, PID_POSITION, aim_pitch_angle, MAX_OUT_AIM_PITCH_ANGLE, MAX_IOUT_AIM_PITCH_ANGLE);
	PID_init(&gimbal_aim_pid.pitch_velocity, PID_POSITION, aim_pitch_velocity, MAX_OUT_AIM_PITCH_VELOCITY, MAX_IOUT_AIM_PITCH_VELOCITY);
	
	//step4 初始化电机
    MotorInit(&gimbal_direct.yaw,GIMBAL_DIRECT_YAW_ID,GIMBAL_DIRECT_YAW_CAN,GIMBAL_DIRECT_YAW_MOTOR_TYPE,GIMBAL_DIRECT_YAW_DIRECTION,GIMBAL_DIRECT_YAW_REDUCTION_RATIO,GIMBAL_DIRECT_YAW_MODE);
    MotorInit(&gimbal_direct.pitch,GIMBAL_DIRECT_PITCH_ID,GIMBAL_DIRECT_PITCH_CAN,GIMBAL_DIRECT_PITCH_MOTOR_TYPE,GIMBAL_DIRECT_PITCH_DIRECTION,GIMBAL_DIRECT_PITCH_REDUCTION_RATIO,GIMBAL_DIRECT_PITCH_MODE);
	
	//step5 初始化云台初始化校准相关变量
    gimbal_direct.init_start_time=0;
    gimbal_direct.init_timer=0;
    gimbal_direct.init_continue=false;
	
	//step6 模式设置初始化
    gimbal_direct.mode=GIMBAL_ZERO_FORCE;
    gimbal_direct.last_mode = GIMBAL_ZERO_FORCE;
    gimbal_direct.mode_before_rc_err = GIMBAL_ZERO_FORCE;
	
	//step7 云台初始上电位置校准初始化  蒋远志2025.10.14
    gimbal_direct.init_base.pitch = 0.0f;
    gimbal_direct.init_base.yaw   = 0.0f;
    gimbal_direct.init_base_record = false;
}
/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void GimbalSetMode(void)
{
	if ( toe_is_error(DBUS_TOE) )
	{
		gimbal_direct.mode=GIMBAL_DBUS_ERR;
	}
	else if (gimbal_direct.last_mode == GIMBAL_DBUS_ERR)
	{
		gimbal_direct.mode = gimbal_direct.mode_before_rc_err;
	}
	//下档无力
	else if ((switch_is_down(gimbal_direct.rc->rc.s[0]))) //安全档优先级最高
	{
		gimbal_direct.mode=GIMBAL_ZERO_FORCE;
		gimbal_direct.init_continue=false;
	}
	//初始校准模式
	else if (gimbal_direct.mode==GIMBAL_ZERO_FORCE || gimbal_direct.mode==GIMBAL_INIT)  
	{

		gimbal_direct.mode=GIMBAL_INIT;
 
		gimbal_direct.init_continue=Gimbal_direct_init_judge();
		if (gimbal_direct.init_continue==true)//判断是否需要跳出循环
		{
			gimbal_direct.mode=GIMBAL_GAP;
		}
	}
	//上，中档陀螺仪控制
	else if (switch_is_mid(gimbal_direct.rc->rc.s[0]))
	{
		gimbal_direct.mode=GIMBAL_IMU;
	}
	else if (switch_is_up(gimbal_direct.rc->rc.s[0]))
	{
		// 当按下鼠标右键时，进入自瞄模式 (AUTO_AIM)
		if (gimbal_direct.rc->mouse.press_r) 
		{
			gimbal_direct.mode = GIMBAL_AUTO_AIM;
		}
		// 没按鼠标右键时，保持正常的 IMU 控制模式
		else 
		{
			gimbal_direct.mode = GIMBAL_IMU;
		}
	}
}
/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void) 
{
	 //电机相关数据更新
	GetMotorMeasure(&gimbal_direct.yaw);
	GetMotorMeasure(&gimbal_direct.pitch);

	// ================== 【修复达妙电机跨圈漂移核心逻辑】 ==================
    static float last_raw_yaw = 0.0f;
    static float yaw_wrap_offset = 0.0f;
    float current_raw_yaw = gimbal_direct.yaw.fdb.pos;
	
	if (current_raw_yaw - last_raw_yaw < -15.0f) {
        yaw_wrap_offset += 25.0f; 
    } 
	else if (current_raw_yaw - last_raw_yaw > 15.0f) {
        yaw_wrap_offset -= 25.0f;
    }
    last_raw_yaw = current_raw_yaw;
	
	gimbal_direct.yaw.fdb.pos = current_raw_yaw + yaw_wrap_offset;
	
	//IMU相关数据更新
	gimbal_direct.feedback_pos.pitch=GetImuAngle(AX_PITCH);
	float raw_imu_yaw = GetImuAngle(AX_YAW);
	gimbal_direct.feedback_pos.yaw = atan2f(sinf(raw_imu_yaw), cosf(raw_imu_yaw));
//	gimbal_direct.feedback_pos.yaw=GetImuAngle(AX_YAW);
	
	gimbal_direct.feedback_vel.pitch=GetImuVelocity(AX_PITCH);
	gimbal_direct.feedback_vel.yaw=GetImuVelocity(AX_YAW);
	
	Angle_solution();
	
	if(gimbal_direct.mode == GIMBAL_INIT)
	{
		if (gimbal_direct.last_mode != GIMBAL_INIT)
		{
			gimbal_direct.init_start_time=xTaskGetTickCount();
		}
		gimbal_direct.init_timer=xTaskGetTickCount()-gimbal_direct.init_start_time;
	}
	else
	{
		gimbal_direct.init_timer=0;
	}
	if (gimbal_direct.mode == GIMBAL_DBUS_ERR && gimbal_direct.last_mode != GIMBAL_DBUS_ERR )
	{
		gimbal_direct.mode_before_rc_err=gimbal_direct.last_mode;
	}
	gimbal_direct.last_mode=gimbal_direct.mode; //上一运行模式更新
}
/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void) 
{
	if (gimbal_direct.mode == GIMBAL_INIT)
	{
		gimbal_direct.reference.pitch = loop_fp32_constrain(gimbal_direct.pitch.direction * (GIMBAL_DIRECT_PITCH_MID - gimbal_direct.pitch.fdb.pos) + gimbal_direct.feedback_pos.pitch, -M_PI, M_PI);
		gimbal_direct.reference.yaw = loop_fp32_constrain(gimbal_direct.yaw.direction * (GIMBAL_DIRECT_YAW_MID - gimbal_direct.yaw.fdb.pos) + gimbal_direct.feedback_pos.yaw, -M_PI, M_PI);
 	}
	else if(gimbal_direct.mode == GIMBAL_GAP)
	{
		gimbal_direct.reference.pitch = gimbal_direct.feedback_pos.pitch;
		gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
	}
	else if (gimbal_direct.mode==GIMBAL_IMU)
	{
		if (gimbal_direct.last_mode != GIMBAL_IMU)
		{
			gimbal_direct.reference.pitch = gimbal_direct.feedback_pos.pitch;
			gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
		}

		else 
		{
			//读取鼠标的移动（还未测试过鼠标）
			gimbal_direct.reference.pitch = fp32_constrain(gimbal_direct.reference.pitch - gimbal_direct.rc->mouse.y / MOUSE_SENSITIVITY_PITCH, GIMBAL_LOWER_LIMIT_PITCH + gimbal_direct.angle_zero_for_imu, GIMBAL_UPPER_LIMIT_PITCH + gimbal_direct.angle_zero_for_imu);
			gimbal_direct.reference.yaw = loop_fp32_constrain(gimbal_direct.reference.yaw - gimbal_direct.rc->mouse.x / MOUSE_SENSITIVITY_YAW, -M_PI, M_PI);
			//读取摇杆的数据
			gimbal_direct.reference.pitch = fp32_constrain(gimbal_direct.reference.pitch - fp32_deadline(gimbal_direct.rc->rc.ch[1], REMOTE_CONTROLLER_MIN_DEADLINE, REMOTE_CONTROLLER_MAX_DEADLINE) / REMOTE_CONTROLLER_SENSITIVITY, GIMBAL_LOWER_LIMIT_PITCH + gimbal_direct.angle_zero_for_imu, GIMBAL_UPPER_LIMIT_PITCH + gimbal_direct.angle_zero_for_imu);
			gimbal_direct.reference.yaw = loop_fp32_constrain(gimbal_direct.reference.yaw - fp32_deadline(gimbal_direct.rc->rc.ch[0], REMOTE_CONTROLLER_MIN_DEADLINE, REMOTE_CONTROLLER_MAX_DEADLINE) / REMOTE_CONTROLLER_SENSITIVITY, -M_PI, M_PI);
      	}
	}
	else if (gimbal_direct.mode == GIMBAL_AUTO_AIM)
	{
//		gimbal_direct.reference.pitch = fp32_constrain(Gimbal_direct_ecd_to_imu(AX_PITCH,GetScCmdGimbalAngle(AX_PITCH)), GIMBAL_LOWER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu  , GIMBAL_UPPER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu );
//		gimbal_direct.reference.yaw   = loop_fp32_constrain(Gimbal_direct_ecd_to_imu(AX_YAW,GetScCmdGimbalAngle(AX_YAW)), -M_PI , M_PI );
		// 自瞄模式：调用专用的控制逻辑处理函数
        Gimbal_direct_auto_aim_control();
	}
}
/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void)
{
	if (gimbal_direct.mode == GIMBAL_ZERO_FORCE || gimbal_direct.mode == GIMBAL_DBUS_ERR)
	{
		gimbal_direct.pitch.set.curr=0;
		gimbal_direct.yaw.set.pos = gimbal_direct.yaw.fdb.pos;
	}
	else if(gimbal_direct.mode == GIMBAL_IMU || gimbal_direct.mode == GIMBAL_GAP || gimbal_direct.mode == GIMBAL_INIT)
	{
		gimbal_direct.pitch.set.vel = PID_calc(&gimbal_direct_pid.pitch_angle, gimbal_direct.feedback_pos.pitch, gimbal_direct.reference.pitch);
		gimbal_direct.pitch.set.curr = gimbal_direct.pitch.direction * PID_calc(&gimbal_direct_pid.pitch_velocity, gimbal_direct.feedback_vel.pitch, gimbal_direct.pitch.set.vel);
    
		fp32 delta_yaw=loop_fp32_constrain(gimbal_direct.reference.yaw-gimbal_direct.feedback_pos.yaw,-M_PI,M_PI);
		// 这样输出的就是 rad/s 的目标速度
        fp32 target_velocity = delta_yaw * YAW_SPEED_MIT_KP; // 这里的 20.0 是位置环 KP
		gimbal_direct.yaw.set.vel = target_velocity * gimbal_direct.yaw.direction;
	}
	else if(gimbal_direct.mode == GIMBAL_AUTO_AIM)
	{
		gimbal_direct.pitch.set.vel = PID_calc(&gimbal_aim_pid.pitch_angle, gimbal_direct.feedback_pos.pitch, gimbal_direct.reference.pitch);
		gimbal_direct.pitch.set.curr = gimbal_direct.pitch.direction * PID_calc(&gimbal_aim_pid.pitch_velocity, gimbal_direct.feedback_vel.pitch, gimbal_direct.pitch.set.vel);
    
		fp32 delta_yaw=loop_fp32_constrain(gimbal_direct.reference.yaw-gimbal_direct.feedback_pos.yaw,-M_PI,M_PI);
		// 这样输出的就是 rad/s 的目标速度
        fp32 target_velocity_aim = delta_yaw * KP_MIT_AIM; // 这里的 20.0 是位置环 KP
		gimbal_direct.yaw.set.vel = target_velocity_aim * gimbal_direct.yaw.direction;
		
	}
	//VOFA调试
	JustFloat(gimbal_direct.reference.pitch,gimbal_direct.feedback_pos.pitch,0,0);
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void)
{
	CanCmdDjiMotor(GIMBAL_CAN,GIMBAL_STDID,0,gimbal_direct.pitch.set.curr,0,0);
	DmEnable(&gimbal_direct.yaw);
	if(gimbal_direct.mode == GIMBAL_AUTO_AIM)
	{
		DmMitCtrlVelocity(&gimbal_direct.yaw,KD_MIT_AIM);
	}
	else
	{
		DmMitCtrlVelocity(&gimbal_direct.yaw,YAW_SPEED_MIT_KD);
	}	
}



#endif  // GIMBAL_YAW_PITCH

