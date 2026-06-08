#ifndef __VOFA_H__
#define __VOFA_H__

#include "usart.h"

extern uint8_t RxData[7];
extern float kp,ki,kd,set;


extern void JustFloat(float set,float fdb,float rotate,float kp);


#endif
