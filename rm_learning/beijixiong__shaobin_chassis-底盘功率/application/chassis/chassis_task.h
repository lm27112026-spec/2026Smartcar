#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"

extern void chassis_task(void const * pvParameters);

extern void set_cali_chassis_hook(const fp32 motor_middle[4]);

extern bool_t cmd_cali_chassis_hook(fp32 motor_middle[4]);


#endif


