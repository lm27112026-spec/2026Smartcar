#include "VOFA.h"
#include "struct_typedef.h"

//注意：
// C板外壳丝印上的UART1 对应 STM32的UART6 
// C板外壳丝印上的UART2 对应 STM32的UART1
// VOFA使用的是UART1 ，对应 STM32的UART6 


//使用方法：
// 在需要发送数据的地方调用JustFloat
// 比如在底盘控制循环中：
// void ChassisPowerCtrlUpdate(void)
// {
//     // ... 原有代码 ...
    
//     // 添加调试输出
//     JustFloat(power_limit.set_power, 
//               power_limit.truth_power_predict,
//               super_cap.actual_vol,
//               GetPowerControlSpeedScale());
// }


typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;

void Float_to_Byte(float f,unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}


/*
 * 	VOFA+数据传输专用函数
 * 	可自行定义所要传输的内容,编写传入变量即可
 *	需要打开丝印串口1（STM32的UART6）
 *
 *  Float_to_Byte(*要传入的内容*, byte);
 *  HAL_UART_Transmit(&huart1, byte, 4, 0xffff);
 *
 *  记得在VOFA+.h中改变函数声明
 *
 * */
void JustFloat(float set,float fdb,float rotate,float kp)
{

	
	uint8_t byte[4]={0};
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

    Float_to_Byte(set, byte);
    HAL_UART_Transmit(&huart1, byte, 4,0xffff);
	
    Float_to_Byte(fdb, byte);
    HAL_UART_Transmit(&huart1, byte, 4,0xffff);
	
	Float_to_Byte(rotate, byte);
    HAL_UART_Transmit(&huart1, byte, 4,0xffff);                   
	
	Float_to_Byte(kp, byte);
    HAL_UART_Transmit(&huart1, byte, 4,0xffff);

	HAL_UART_Transmit(&huart1, tail, 4,0xffff);
}





uint8_t RxData[7];//串口接收缓冲

float kp=0,ki=0,kd=0,set=0;


void VOFA_GetData(uint8_t RxData[],float *kp,float *ki,float *kd,float *set)
{
	if(RxData[0]==0xAA&&RxData[6]==0xFF)
	{
		//float value=(float)((uint32_t)RxData[2]|(uint32_t)RxData[3]<<8|(uint32_t)RxData[4]<<16|(uint32_t)RxData[5]<<24);
		unsigned char mem[]={RxData[2],RxData[3],RxData[4],RxData[5]};
		float *value=(float*)mem;
		switch(RxData[1])			
	    {
			case 0xC1:
				*kp=*value;
				break;
			case 0xC2:
				*ki=*value;
				break;
			case 0xC3:
				*kd=*value;
				break;
			case 0xC4:
				*set=*value;
				break;
			default:
				break;										
		}
		
	}else{
		return;
	}		
	
}



//记得在主函数加上HAL_UART_Receive_DMA(&huart6,RxData,sizeof(RxData));
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{
		VOFA_GetData(RxData,&kp,&ki,&kd,&set);
		HAL_UART_Receive_DMA(&huart1,RxData,sizeof(RxData));
	}	
}
