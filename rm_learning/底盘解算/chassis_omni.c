#include "robot_param.h"
#if(CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#include "chassis_omni.h"
#include "CAN_receive.h"
//#include "chassis.h"
#include "math.h"
#include "motor.h" 
#include "detect_task.h"
#include "user_lib.h"
#include "CAN_communication.h"
#include "VOFA.h"
#include "chassis_power_control.h"

Chassis_s chassis;
PID_t chassis_pid;

//static inline float wrap_to_pi(float angle)
//{
//    while (angle > M_PI)  angle -= 2.0f * M_PI;
//    while (angle < -M_PI) angle += 2.0f * M_PI;
//    return angle;
//}

#define MAX_SPEED_FULL_POWER       2.5f      
#define MAX_VELOCITY_FULL_POWER    8.0f     
#define MIN_SPEED_LIMITED_POWER    1.2f      
#define MIN_VELOCITY_LIMITED_POWER 5.0f      
#define POWER_SCALE_SMOOTH_FACTOR  0.05f     

#define CHASSIS_SPIN_SPEED         10.0f     
#define SPIN_SPEED_MULTIPLIER      1.3f 


inline bool GetChassisSpinState(void)
{
    return chassis.spin_flag;
}

fp32 GetChassisCurrentMaxSpeed(void)
{
    return chassis.dynamic_max_speed;
}

fp32 GetChassisCurrentMaxVelocity(void)
{
    return chassis.dynamic_max_velocity;
}

fp32 GetChassisSpeedScaleFactor(void)
{
    return chassis.speed_scale_factor;
}

Chassis_s* GetChassisDataPointer(void)
{
    return &chassis;
}

static void CalculateDynamicMaxSpeed(void)
{
    fp32 target_scale = 1.0f;
    
    if (chassis.power_control_state == 1)
    {
        target_scale = GetPowerControlSpeedScale();  
        if (target_scale < 0.5f) target_scale = 0.5f;  
        if (target_scale > 1.2f) target_scale = 1.2f; 
    }
    
    // 平滑滤波器
    static fp32 last_scale = 1.0f;
    target_scale = last_scale * (1.0f - POWER_SCALE_SMOOTH_FACTOR) + 
                   target_scale * POWER_SCALE_SMOOTH_FACTOR;
    last_scale = target_scale;
    
    chassis.speed_scale_factor = target_scale;
    
    chassis.dynamic_max_speed = MIN_SPEED_LIMITED_POWER + 
                               (MAX_SPEED_FULL_POWER - MIN_SPEED_LIMITED_POWER) * target_scale;
    
    if (chassis.mode == CHASSIS_SPIN || chassis.spin_flag)
    {
        chassis.dynamic_max_velocity = 5.0f + (MAX_VELOCITY_FULL_POWER - 5.0f) * target_scale * 1.2f; 
    }
    else
    {
        chassis.dynamic_max_velocity = MIN_VELOCITY_LIMITED_POWER + 
                                      (MAX_VELOCITY_FULL_POWER - MIN_VELOCITY_LIMITED_POWER) * target_scale;
    }
    
    // 安全限幅
    if (chassis.dynamic_max_speed < 0.8f) chassis.dynamic_max_speed = 0.8f;
    if (chassis.dynamic_max_speed > 4.0f) chassis.dynamic_max_speed = 4.0f;
    if (chassis.dynamic_max_velocity < 2.0f) chassis.dynamic_max_velocity = 2.0f;
    if (chassis.dynamic_max_velocity > 20.0f) chassis.dynamic_max_velocity = 20.0f;
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
	//step1 获取所有所需变量指针
	chassis.rc = get_remote_control_point();
	
	//step2 PID数据清零，设置PID参数
	const static fp32 wheel_vel[3] = {KP_OMNI_VEL ,KI_OMNI_VEL ,KD_OMNI_VEL};
	for(int i = 0;i < 4;++i)
	{
		PID_init(&chassis_pid.wheel_velocity[i],PID_POSITION ,wheel_vel ,MAX_OUT_OMNI_VEL,MAX_IOUT_OMNI_VEL);
	}
	
	const static fp32 gimbal_follow[3]={KP_CHASSIS_FOLLOW_GIMBAL,KI_CHASSIS_FOLLOW_GIMBAL,KD_CHASSIS_FOLLOW_GIMBAL};
    PID_init(&chassis_pid.follow,PID_POSITION,gimbal_follow,MAX_OUT_CHASSIS_FOLLOW_GIMBAL,MAX_IOUT_CHASSIS_FOLLOW_GIMBAL);

	
	//step3 初始化电机
	MotorInit(&chassis.wheel[0],WHEEL_1_ID ,WHEEL_1_CAN ,WHEEL_1_MOTOR_TYPE ,WHEEL_1_DIRECTION ,WHEEL_1_RATIO ,WHEEL_1_MODE);
	MotorInit(&chassis.wheel[1],WHEEL_2_ID ,WHEEL_2_CAN ,WHEEL_2_MOTOR_TYPE ,WHEEL_2_DIRECTION ,WHEEL_2_RATIO ,WHEEL_2_MODE);
	MotorInit(&chassis.wheel[2],WHEEL_3_ID ,WHEEL_3_CAN ,WHEEL_3_MOTOR_TYPE ,WHEEL_3_DIRECTION ,WHEEL_3_RATIO ,WHEEL_3_MODE);
	MotorInit(&chassis.wheel[3],WHEEL_4_ID ,WHEEL_4_CAN ,WHEEL_4_MOTOR_TYPE ,WHEEL_4_DIRECTION ,WHEEL_4_RATIO ,WHEEL_4_MODE);
	
	//step4 初始模式设置
	chassis.mode = CHASSIS_LOCK;
	//step5 小陀螺模式初始值设置  汪辰旭2026.1.22
    chassis.shift_flag=false;
    chassis.shift_last_flag = false;
    chassis.spin_flag=false;
	
	chassis.dynamic_max_speed = MAX_SPEED_FULL_POWER;
    chassis.dynamic_max_velocity = MAX_VELOCITY_FULL_POWER;
    chassis.speed_scale_factor = 1.0f;
    chassis.power_control_state = 1;
	
	// 初始化功率控制
    ChassisPowerCtrlInit();
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
	if( switch_is_down(chassis.rc->rc.s[0]))//( toe_is_error(DBUS_TOE)) ||
	{
		chassis.mode = CHASSIS_LOCK ;
	}
	else if(switch_is_mid(chassis.rc->rc.s[0]))
	{
//		chassis.mode = CHASSIS_SINGLE ;
//		chassis.mode = CHASSIS_FOLLOW ;
		chassis.mode = CHASSIS_SPIN;
	}
	else if(switch_is_up(chassis.rc->rc.s[0]))
	{
		chassis.mode = CHASSIS_SPIN;
//		chassis.mode = CHASSIS_SINGLE;
//		chassis.mode = CHASSIS_FOLLOW ;
		chassis.power_control_state = 1;
	}
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void)
{
	for(int i = 0;i < 4;++i)
	{
		GetMotorMeasure(&chassis.wheel[i]);
	}
	for(int i = 0;i < 4; ++i)
	{
		chassis.feedback[i] = chassis.wheel[i].fdb.vel;
	}
	float yaw_total_angle = GetCanGimbalYawMotorPos();
    chassis.yaw_delta = atan2f(sinf(yaw_total_angle), cosf(yaw_total_angle));
	
	chassis.shift_last_flag = chassis.shift_flag;
    chassis.shift_flag  = chassis.rc->key.v & KEY_PRESSED_OFFSET_SHIFT;
    
    if (chassis.shift_flag == true && chassis.shift_last_flag == false)
    {
        chassis.spin_flag = !chassis.spin_flag;
    }
    
    // 动态计算防掉血缩放系数
    CalculateDynamicMaxSpeed();
}
/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{
	if(chassis.mode == CHASSIS_LOCK)
	{
		chassis.reference.vx = 0;
		chassis.reference.vy = 0;
		chassis.reference.wz = 0;
	}
	else if(chassis.mode == CHASSIS_SINGLE)
	{
		chassis.reference.vx = fp32_deadline(chassis.rc->rc.ch[3],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*chassis.dynamic_max_speed;
        chassis.reference.vy = fp32_deadline(-chassis.rc->rc.ch[2],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*chassis.dynamic_max_speed;
        chassis.reference.wz = fp32_deadline(-chassis.rc->rc.ch[0],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*chassis.dynamic_max_velocity;
		
		if(chassis.rc->key.v & KEY_PRESSED_OFFSET_W) chassis.reference.vx += chassis.dynamic_max_speed * 0.6f;
        else if(chassis.rc->key.v & KEY_PRESSED_OFFSET_S) chassis.reference.vx -= chassis.dynamic_max_speed * 0.6f;
        
        if(chassis.rc->key.v & KEY_PRESSED_OFFSET_A) chassis.reference.vy += chassis.dynamic_max_speed * 0.6f;
        else if(chassis.rc->key.v & KEY_PRESSED_OFFSET_D) chassis.reference.vy -= chassis.dynamic_max_speed * 0.6f;
	}
	else if(chassis.mode == CHASSIS_FOLLOW)
	{
		chassis.reference_rc.vx = fp32_deadline(chassis.rc->rc.ch[3],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*chassis.dynamic_max_speed;
        chassis.reference_rc.vy = fp32_deadline(-chassis.rc->rc.ch[2],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*chassis.dynamic_max_speed;
        
        if(chassis.rc->key.v & KEY_PRESSED_OFFSET_W) chassis.reference_rc.vx += chassis.dynamic_max_speed * 0.6f;
        else if(chassis.rc->key.v & KEY_PRESSED_OFFSET_S) chassis.reference_rc.vx -= chassis.dynamic_max_speed * 0.6f;
        
        if(chassis.rc->key.v & KEY_PRESSED_OFFSET_A) chassis.reference_rc.vy += chassis.dynamic_max_speed * 0.6f;
        else if(chassis.rc->key.v & KEY_PRESSED_OFFSET_D) chassis.reference_rc.vy -= chassis.dynamic_max_speed * 0.6f;
        
        chassis.reference.vx = chassis.reference_rc.vx * cosf(chassis.yaw_delta) - chassis.reference_rc.vy * sinf(chassis.yaw_delta);
        chassis.reference.vy = chassis.reference_rc.vx * sinf(chassis.yaw_delta) + chassis.reference_rc.vy * cos(chassis.yaw_delta);
        
        fp32 wz_pid = PID_calc(&chassis_pid.follow, 0, chassis.yaw_delta);
        if (wz_pid > chassis.dynamic_max_velocity) wz_pid = chassis.dynamic_max_velocity;
        if (wz_pid < -chassis.dynamic_max_velocity) wz_pid = -chassis.dynamic_max_velocity;
        chassis.reference.wz = wz_pid;
	}
	else if (chassis.mode == CHASSIS_SPIN)
	{  
		 if(chassis.rc->key.v & KEY_PRESSED_OFFSET_SHIFT)
        {
            chassis.reference_rc.vx = fp32_deadline(chassis.rc->rc.ch[3], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * chassis.dynamic_max_speed;
            chassis.reference_rc.vy = fp32_deadline(-chassis.rc->rc.ch[2], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * chassis.dynamic_max_speed;
        
            if (chassis.rc->key.v & KEY_PRESSED_OFFSET_S) chassis.reference_rc.vx -= chassis.dynamic_max_speed * 0.8f;  
            else if (chassis.rc->key.v & KEY_PRESSED_OFFSET_W) chassis.reference_rc.vx += chassis.dynamic_max_speed * 0.8f;  
        
            if (chassis.rc->key.v & KEY_PRESSED_OFFSET_D) chassis.reference_rc.vy -= chassis.dynamic_max_speed * 0.8f;  
            else if (chassis.rc->key.v & KEY_PRESSED_OFFSET_A) chassis.reference_rc.vy += chassis.dynamic_max_speed * 0.8f;  
    
            chassis.reference.vx = chassis.reference_rc.vx * cosf(chassis.yaw_delta) - chassis.reference_rc.vy * sinf(chassis.yaw_delta);
            chassis.reference.vy = chassis.reference_rc.vx * sinf(chassis.yaw_delta) + chassis.reference_rc.vy * cos(chassis.yaw_delta);
        
			fp32 target_wz = CHASSIS_SPIN_SPEED * chassis.speed_scale_factor * SPIN_SPEED_MULTIPLIER;
            chassis.reference.wz = (1.0f - 0.02f) * chassis.reference.wz + 0.02f * target_wz;  
        }
        else
        {
            chassis.reference_rc.vx = fp32_deadline(chassis.rc->rc.ch[3],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*chassis.dynamic_max_speed;
            chassis.reference_rc.vy = fp32_deadline(-chassis.rc->rc.ch[2],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*chassis.dynamic_max_speed;
            
            if(chassis.rc->key.v & KEY_PRESSED_OFFSET_W) chassis.reference_rc.vx += chassis.dynamic_max_speed * 0.6f; 
            else if(chassis.rc->key.v & KEY_PRESSED_OFFSET_S) chassis.reference_rc.vx -= chassis.dynamic_max_speed * 0.6f; 
            
            if(chassis.rc->key.v & KEY_PRESSED_OFFSET_A) chassis.reference_rc.vy += chassis.dynamic_max_speed * 0.6f; 
            else if(chassis.rc->key.v & KEY_PRESSED_OFFSET_D) chassis.reference_rc.vy -= chassis.dynamic_max_speed * 0.6f; 

            chassis.reference.vx = chassis.reference_rc.vx * cosf(chassis.yaw_delta) - chassis.reference_rc.vy * sinf(chassis.yaw_delta);
            chassis.reference.vy = chassis.reference_rc.vx * sinf(chassis.yaw_delta) + chassis.reference_rc.vy * cos(chassis.yaw_delta);
            
            fp32 wz_pid = PID_calc(&chassis_pid.follow, 0, chassis.yaw_delta);
            if (wz_pid > chassis.dynamic_max_velocity) wz_pid = chassis.dynamic_max_velocity;
            if (wz_pid < -chassis.dynamic_max_velocity) wz_pid = -chassis.dynamic_max_velocity;
            chassis.reference.wz = wz_pid;
        }
	}
}
/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
	chassis.set[0] = ((1.0 / sqrt(2))*(  chassis.reference.vx - chassis.reference.vy ) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[0].reduction_ratio;
    chassis.set[1] = ((1.0 / sqrt(2))*(  chassis.reference.vx + chassis.reference.vy ) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[1].reduction_ratio;
    chassis.set[2] = ((1.0 / sqrt(2))*( -chassis.reference.vx + chassis.reference.vy ) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[2].reduction_ratio;
    chassis.set[3] = ((1.0 / sqrt(2))*( -chassis.reference.vx - chassis.reference.vy ) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[3].reduction_ratio; 

	for(int i = 0;i < 4;++i)
	{
		chassis.wheel[i].set.curr = PID_calc(&chassis_pid.wheel_velocity[i],chassis.feedback[i],chassis.set[i]);
	}
	
//	if(chassis.rc->key.v & KEY_PRESSED_OFFSET_F && !chassis.f_flag)
//	{
//		if(chassis.sc_flag)
//		{
//			chassis.sc_flag = 0;
//		}
//		else
//		{
//			chassis.sc_flag = 1;
//		}
//	}
//	chassis.f_flag = chassis.rc->key.v & KEY_PRESSED_OFFSET_F;
	//VOFA调试
//	JustFloat(chassis.set[1],chassis.feedback[1],0,0);

}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisSendCmd(void)
{
	CanCmdDjiMotor(CHASSIS_CAN,CHASSIS_STDID,chassis.wheel[3].set.curr,chassis.wheel[0].set.curr,chassis.wheel[1].set.curr,chassis.wheel[2].set.curr);
}

#endif

