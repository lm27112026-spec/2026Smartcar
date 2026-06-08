#ifndef __CUSTOM_TYPEDEF_H
#define __CUSTOM_TYPEDEF_H

#include "stdbool.h"

// 数据名称宏
#define IMU_NAME "imu_data"
#define CHASSIS_FDB_SPEED_NAME "chassis_fdb_speed"
#define ROBOT_CMD_DATA_NAME "ROBOT_CMD_DATA"
#define USB_OFFLINE_NAME "usb_offline"
#define VIRTUAL_RC_NAME "virtual_rc_ctrl"
#define CALI_BUZZER_STATE_NAME "CaliBuzzerState"

typedef enum {
    CALI_BUZZER_OFF = 0,
    CALI_BUZZER_BEGIN,
    CALI_BUZZER_MIDDLE_TIME,
    CALI_BUZZER_GIMBAL,
    CALI_BUZZER_IMU,
    CALI_BUZZER_CHASSIS,
} CaliBuzzerState_e;

typedef struct __Imu
{
    float angle[3];  // rad 欧拉角数据
    float gyro[3];   // rad/s 陀螺仪数据
    float accel[3];  // m/s^2 加速度计数据
	float q[4];      // 四元数 w, x, y, z
} Imu_t;

typedef struct  // 底盘速度向量结构体
{
    float vx;  // (m/s) x方向速度
    float vy;  // (m/s) y方向速度
    float wz;  // (rad/s) 旋转速度
} ChassisSpeedVector_t;

typedef struct
{
    ChassisSpeedVector_t speed_vector;
    struct
    {
        float roll;
        float pitch;
        float yaw;
        float leg_length;
    } chassis;

    struct
    {
        float pitch;
        float yaw;
    } gimbal;

    struct
    {
        bool fire;
        bool fric_on;
    } shoot;

} RobotCmdData_t;

#endif  // __CUSTOM_TYPEDEF_H
