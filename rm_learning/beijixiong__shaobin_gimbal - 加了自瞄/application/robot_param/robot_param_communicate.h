/**
  * @file       robot_param_communicate.h
  * @brief      ...
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

// clang-format off
#define CHASSIS_TYPE         CHASSIS_NONE         // 选择底盘类型
#define GIMBAL_TYPE          GIMBAL_NONE          // 选择云台类型
#define SHOOT_TYPE           SHOOT_NONE           // 选择发射机构类型
#define MECHANICAL_ARM_TYPE  MECHANICAL_ARM_NONE  // 选择机械臂类型
#define CONTROL_TYPE         CHASSIS_AND_GIMBAL   // 选择控制类型
// clang-format on

#define __GYRO_BIAS_YAW  0.003096855f // 陀螺仪零飘，单位rad/s


#endif /* INCLUDED_ROBOT_PARAM_H */
