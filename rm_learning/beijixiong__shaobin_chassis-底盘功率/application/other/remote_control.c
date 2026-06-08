/**
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *  V2.0.0     Feb-17-2025     Penguin         1. support RC AT9S PRO
  *                                             2. support RC HT8A
  *                                             3. support normal sbus RC in struct Sbus_t
  *  V2.0.1     Feb-25-2025     Penguin         1. support RC ET08A
  *
  @verbatim
  ==============================================================================
  使用At9sPro遥控器时请设置5通为SwE，6通为SwG

  注：使用非DT7遥控器时，需要先检查通道值数据是否正常（一般遥控器都带有通道值数据偏移功能，将通道值中值移动到正确数值后再使用）
      AT9S PRO 遥控器中值为 1000
      HT8A 遥控器中值为 992
      ET08A 遥控器中值为 1024
  
  ET08A 遥控器设置指南：
    1. 设置 主菜单->系统设置->摇杆模式 为模式2
    2. 设置 主菜单->通用功能->通道设置 5通道为 [辅助1 SB --] 6通道为 [辅助2 SC --]
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  */

// clang-format off
#include "remote_control.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"

#include "detect_task.h"
#include "robot_param.h"
//#include "communication.h"

// 遥控器掉线时间阈值
#define RC_LOST_TIME 100  // ms
// 非dt7遥控器连续断线上线次数（超过认为断连）
#define SBUS_MAX_LOST_NUN 10

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


//取正函数
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

#if (__RC_TYPE == RC_AT9S_PRO)
static void At9sProSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#elif (__RC_TYPE == RC_HT8A)
static void Ht8aSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#elif (__RC_TYPE == RC_ET08A)
static void Et08aSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#endif

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;
Sbus_t sbus = {.connect_flag = 0xFF};

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

// 上一次接收数据的时间
static uint32_t last_receive_time = 0;
// 记录连续接收数据的次数
static uint32_t receive_count = 0;
// 记录非dt7的sbus遥控器连续断连次数
static uint32_t sbus_lost_count = SBUS_MAX_LOST_NUN + 5;


#if (__RC_TYPE != RC_DT7)
static uint8_t connected_flag;  // 遥控器连接标志位
#endif  // __RC_TYPE != RC_DT7

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
#if (__RC_TYPE == RC_AT9S_PRO)
    connected_flag = AT9S_PRO_RC_CONNECTED_FLAG;
#elif (__RC_TYPE == RC_HT8A)
    connected_flag = HT8A_RC_CONNECTED_FLAG;
#elif (__RC_TYPE == RC_ET08A)
    connected_flag = ET08A_RC_CONNECTED_FLAG;
#endif
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

/**
  * @brief          获取SBUS遥控器数据指针
  * @param[in]      none
  * @retval         SBUS遥控器数据指针
  */
const Sbus_t *get_sbus_point(void)
{
    return &sbus;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

// clang-format on
// 记录接收数据的次数
#define COUNT_RECEIVED                            \
    if (now - last_receive_time > RC_LOST_TIME) { \
        receive_count = 0;                        \
    }                                             \
    receive_count++;
// clang-format off

//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        uint32_t now = HAL_GetTick();

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                
                COUNT_RECEIVED
                
                //记录数据接收时间
                last_receive_time = HAL_GetTick();
                detect_hook(DBUS_TOE);
                sbus_to_usart1(sbus_rx_buf[0]);
            } 
            else if (this_time_rx_len == SBUS_RC_FRAME_LENGTH)
            {
                //处理遥控器数据
#if (__RC_TYPE == RC_AT9S_PRO)
                At9sProSbusToRc(sbus_rx_buf[0], &rc_ctrl);
#elif (__RC_TYPE == RC_HT8A)
                Ht8aSbusToRc(sbus_rx_buf[0], &rc_ctrl);
#elif (__RC_TYPE == RC_ET08A)
                Et08aSbusToRc(sbus_rx_buf[0], &rc_ctrl);
#endif
                
                COUNT_RECEIVED
                
                //记录数据接收时间
                last_receive_time = HAL_GetTick();
                detect_hook(DBUS_TOE);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                
                COUNT_RECEIVED

                //记录数据接收时间
                last_receive_time = HAL_GetTick();
                detect_hook(DBUS_TOE);
                sbus_to_usart1(sbus_rx_buf[1]);
            }
            else if (this_time_rx_len == SBUS_RC_FRAME_LENGTH)
            {
                //处理遥控器数据
#if (__RC_TYPE == RC_AT9S_PRO)
                At9sProSbusToRc(sbus_rx_buf[1], &rc_ctrl);
#elif (__RC_TYPE == RC_HT8A)
                Ht8aSbusToRc(sbus_rx_buf[1], &rc_ctrl);
#elif (__RC_TYPE == RC_ET08A)
                Et08aSbusToRc(sbus_rx_buf[1], &rc_ctrl);
#endif
                
                COUNT_RECEIVED

                //记录数据接收时间
                last_receive_time = HAL_GetTick();
                detect_hook(DBUS_TOE);
            }
        }
    }

}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

// SBUS通道解析
#define SBUS_DECODE()                                                                 \
    sbus.ch[0] =((sbus_buf[2]<<8)   + (sbus_buf[1])) & 0x07ff;                        \
    sbus.ch[1] =((sbus_buf[3]<<5)   + (sbus_buf[2]>>3)) & 0x07ff;                     \
    sbus.ch[2] =((sbus_buf[5]<<10)  + (sbus_buf[4]<<2) + (sbus_buf[3]>>6)) & 0x07ff;  \
    sbus.ch[3] =((sbus_buf[6]<<7)   + (sbus_buf[5]>>1)) & 0x07ff;                     \
    sbus.ch[4] =((sbus_buf[7]<<4)   + (sbus_buf[6]>>4)) & 0x07ff;                     \
    sbus.ch[5] =((sbus_buf[9]<<9)   + (sbus_buf[8]<<1) + (sbus_buf[7]>>7)) & 0x07ff;  \
    sbus.ch[6] =((sbus_buf[10]<<6)  + (sbus_buf[9]>>2)) & 0x07ff;                     \
    sbus.ch[7] =((sbus_buf[11]<<3)  + (sbus_buf[10]>>5)) & 0x07ff;                    \
    sbus.ch[8] =((sbus_buf[13]<<8)  + (sbus_buf[12])) & 0x07ff;                       \
    sbus.ch[9] =((sbus_buf[14]<<5)  + (sbus_buf[13]>>3)) & 0x07ff;                    \
    sbus.ch[10]=((sbus_buf[16]<<10) + (sbus_buf[15]<<2) + (sbus_buf[14]>>6)) & 0x07ff;\
    sbus.ch[11]=((sbus_buf[17]<<7)  + (sbus_buf[16]>>1)) & 0x07ff;                    \
    sbus.ch[12]=((sbus_buf[18]<<4)  + (sbus_buf[17]>>4)) & 0x07ff;                    \
    sbus.ch[13]=((sbus_buf[20]<<9)  + (sbus_buf[19]<<1) + (sbus_buf[18]>>7)) & 0x07ff;\
    sbus.ch[14]=((sbus_buf[21]<<6)  + (sbus_buf[20]>>2)) & 0x07ff;                    \
    sbus.ch[15]=((sbus_buf[22]<<3)  + (sbus_buf[21]>>5)) & 0x07ff;                    \
    sbus.connect_flag = sbus_buf[23];

#define SBUS_LOST_CHECK()                      \
    if (sbus.connect_flag == connected_flag) { \
        sbus_lost_count = 0;                   \
    } else {                                   \
        sbus_lost_count++;                     \
    }

// DT7遥控器特殊通道置零
#define SPECIAL_CHANNEL_SET_SERO()\
    rc_ctrl->mouse.x = 0;         \
    rc_ctrl->mouse.y = 0;         \
    rc_ctrl->mouse.z = 0;         \
    rc_ctrl->mouse.press_l = 0;   \
    rc_ctrl->mouse.press_r = 0;   \
    rc_ctrl->key.v = 0;           \
    rc_ctrl->rc.ch[4] = 0;        \

#if (__RC_TYPE == RC_AT9S_PRO)

/**
  * @brief          AT9S PRO 遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
static void At9sProSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    // SBUS通道解析
    SBUS_DECODE()
    SBUS_LOST_CHECK()

    // 将SBUS通道数据转换为DT7遥控器数据，方便兼容使用
    rc_ctrl->rc.ch[0] =  (sbus.ch[0] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800.0f * 660;
    rc_ctrl->rc.ch[1] = -(sbus.ch[1] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800.0f * 660;
    rc_ctrl->rc.ch[2] =  (sbus.ch[3] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800.0f * 660;
    rc_ctrl->rc.ch[3] =  (sbus.ch[2] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800.0f * 660;

    static char sw_mapping[3] = {RC_SW_UP, RC_SW_MID, RC_SW_DOWN};
    rc_ctrl->rc.s[0] = sw_mapping[(sbus.ch[5] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800 + 1];
    rc_ctrl->rc.s[1] = sw_mapping[(sbus.ch[4] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800 + 1];

    // AT9S PRO 遥控器没有鼠标和键盘数据
    SPECIAL_CHANNEL_SET_SERO()
}

#elif (__RC_TYPE == RC_HT8A)

/**
  * @brief          HT8A 遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
static void Ht8aSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    // SBUS通道解析
    SBUS_DECODE()
    SBUS_LOST_CHECK()

    // 将SBUS通道数据转换为DT7遥控器数据，方便兼容使用
    rc_ctrl->rc.ch[0] =  (sbus.ch[0] - HT8A_RC_CH013_VALUE_OFFSET) / 560.0f * 660;
    rc_ctrl->rc.ch[1] =  (sbus.ch[1] - HT8A_RC_CH013_VALUE_OFFSET) / 560.0f * 660;
    rc_ctrl->rc.ch[2] =  (sbus.ch[3] - HT8A_RC_CH013_VALUE_OFFSET) / 560.0f * 660;
    rc_ctrl->rc.ch[3] =  (sbus.ch[2] - HT8A_RC_CH247_VALUE_OFFSET) / 800.0f * 660;

    static char sw_mapping[3] = {RC_SW_UP, RC_SW_MID, RC_SW_DOWN};
    rc_ctrl->rc.s[0] = sw_mapping[(sbus.ch[7] - HT8A_RC_CH247_VALUE_OFFSET) / 800 + 1];
    rc_ctrl->rc.s[1] = sw_mapping[(sbus.ch[4] - HT8A_RC_CH247_VALUE_OFFSET) / 800 + 1];

    // HT8A 遥控器没有鼠标和键盘数据
    SPECIAL_CHANNEL_SET_SERO()
}

#elif (__RC_TYPE == RC_ET08A)

/**
  * @brief          ET08A 遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
static void Et08aSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl){
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    // SBUS通道解析
    SBUS_DECODE()
    SBUS_LOST_CHECK()

    // 将SBUS通道数据转换为DT7遥控器数据，方便兼容使用
    rc_ctrl->rc.ch[0] =  (sbus.ch[0] - ET08A_RC_CH_VALUE_OFFSET) / 671.0f * 660;
    rc_ctrl->rc.ch[1] = -(sbus.ch[1] - ET08A_RC_CH_VALUE_OFFSET) / 671.0f * 660;
    rc_ctrl->rc.ch[2] =  (sbus.ch[3] - ET08A_RC_CH_VALUE_OFFSET) / 671.0f * 660;
    rc_ctrl->rc.ch[3] =  (sbus.ch[2] - ET08A_RC_CH_VALUE_OFFSET) / 671.0f * 660;

    static char sw_mapping[3] = {RC_SW_UP, RC_SW_MID, RC_SW_DOWN};
    rc_ctrl->rc.s[0] = sw_mapping[(sbus.ch[5] - ET08A_RC_CH_VALUE_OFFSET) / 670 + 1];
    rc_ctrl->rc.s[1] = sw_mapping[(sbus.ch[4] - ET08A_RC_CH_VALUE_OFFSET) / 670 + 1];

    // ET08A 遥控器没有鼠标和键盘数据
    SPECIAL_CHANNEL_SET_SERO()
}

#endif

#undef SBUS_DECODE
#undef SPECIAL_CHANNEL_SET_SERO

/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          通过usart1发送sbus数据,在usart3_IRQHandle调用
  * @param[in]      sbus: sbus数据, 18字节
  * @retval         none
  */
void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}

// clang-format on

/******************************************************************/
/* API                                                            */
/*----------------------------------------------------------------*/
/* function:      GetRcOffline                                    */
/*                GetDt7RcCh                                      */
/*                GetDt7RcSw                                      */
/*                GetDt7MouseSpeed                                */
/*                GetDt7Mouse                                     */
/*                GetDt7Keyboard                                  */
/******************************************************************/

/**
  * @brief          获取遥控器是否离线。
  * @retval         true:离线，false:在线
  */
inline bool GetRcOffline(void)
{
#if __RC_TYPE == RC_DT7
#define USE_SBUS_LOST_COUNT 0
#else
#define USE_SBUS_LOST_COUNT 1
#endif

#if __CONTROL_LINK_RC == CL_RC_DIRECT
    return !((receive_count > 5) && (HAL_GetTick() - last_receive_time < RC_LOST_TIME)) ||
           ((sbus_lost_count > SBUS_MAX_LOST_NUN) && USE_SBUS_LOST_COUNT);
#elif __CONTROL_LINK_RC == CL_RC_UART2
    return GetUartRcOffline();
#else
    return true;
#endif

#undef USE_SBUS_LOST_COUNT
}

/**
  * @brief          获取DT7遥控器通道值。
  * @param[in]      ch 通道id，0-右平, 1-右竖, 2-左平, 3-左竖, 4-左滚轮，配合ch id宏进行使用
  * @retval         DT7遥控器通道值，范围为 [−1,1]
  */
inline float GetDt7RcCh(uint8_t ch) { return rc_ctrl.rc.ch[ch] * RC_TO_ONE; }
/**
  * @brief          获取DT7遥控器拨杆值，可配合switch_is_xxx系列宏函数使用。
  * @param[in]      sw 通道id，0-右, 1-左，配合sw id宏进行使用
  * @retval         DT7遥控器拨杆值，范围为{1,2,3}
  */
inline char GetDt7RcSw(uint8_t sw) { return rc_ctrl.rc.s[sw]; }
/**
  * @brief          获取鼠标axis轴的移动速度
  * @param[in]      axis 轴id, 0-, 1-, 2-，配合轴id宏进行使用
  * @retval         鼠标axis轴移动速度，范围为[,]
  */
inline float GetDt7MouseSpeed(uint8_t axis)
{
    switch (axis) {
        case AX_X:
            return rc_ctrl.mouse.x;
        case AX_Y:
            return rc_ctrl.mouse.y;
        case AX_Z:
            return rc_ctrl.mouse.z;
        default:
            return 0;
    }
}
/**
  * @brief          获取鼠标按键信息
  * @param[in]      key 按键id，配合按键id宏进行使用
  * @retval         鼠标按键是否被按下
  */
inline bool GetDt7Mouse(uint8_t key)
{
    switch (key) {
        case KEY_LEFT:
            return rc_ctrl.mouse.press_l;
        case KEY_RIGHT:
            return rc_ctrl.mouse.press_r;
        default:
            return 0;
    }
}
/**
  * @brief          获取键盘按键信息
  * @param[in]      key 按键id，配合按键id宏进行使用
  * @retval         键盘按键是否被按下
  */
inline bool GetDt7Keyboard(uint8_t key) { return rc_ctrl.key.v & ((uint16_t)1 << key); }
/*------------------------------ End of File ------------------------------*/
