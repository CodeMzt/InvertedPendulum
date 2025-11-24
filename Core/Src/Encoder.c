/********************************************************************************

* @File Encoder.c

* @Author: Ma Ziteng

* @Version: 1.0

* @Date: 2025-10

* @Description: 编码器测速模块

********************************************************************************/
#include "Encoder.h"
#include "tim.h"

/**
* @brief 编码器测速
* @param 无
* @return temp 			读取的计数值
*/
int16_t ReadSpeed(void){
    int16_t temp = (short) __HAL_TIM_GetCounter(&htim3); //将16位改成8位，使之有符号 int16_t(正数部分)->int8_t
    __HAL_TIM_SetCounter(&htim3,0);
    return -temp;
}
/**
* @brief 编码器测速初始化
* @param
* @return
*/
void Encoder_Init(void){
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
}