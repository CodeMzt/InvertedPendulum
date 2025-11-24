/********************************************************************************

* @File motor.c

* @Author: Ma Ziteng

* @Version: 1.0

* @Date: 2025-11

* @Description: 电机驱动

********************************************************************************/

#include "motor.h"
#include "tim.h"

#include "stm32f1xx_hal_tim.h"


void Load(int16_t Speed) {
    if (Speed >= 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Speed>=100?100:Speed);
    }else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Speed<=-100?100:-Speed);
    }
}
void Motor_Init(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
