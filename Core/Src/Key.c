/********************************************************************************

* @File Key.c

* @Author: Ma Ziteng

* @Version: 1.0

* @Date: 2025-11

* @Description: 非阻塞式按键实现

********************************************************************************/

#include "../Inc/Key.h"
#include "gpio.h"
#include <stdint.h>

uint8_t temp,key;

uint8_t Key_GetState(void) {
    if (HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin) == GPIO_PIN_RESET)
        return 1;
    if (HAL_GPIO_ReadPin(K2_GPIO_Port, K2_Pin) == GPIO_PIN_RESET)
        return 2;
    if (HAL_GPIO_ReadPin(K3_GPIO_Port, K3_Pin) == GPIO_PIN_RESET)
        return 3;
    if (HAL_GPIO_ReadPin(K4_GPIO_Port, K4_Pin) == GPIO_PIN_RESET)
        return 4;
    return 0;
}

uint8_t Key_Getkey(void) {
    temp = key;
    key = 0;
    return temp;
}

void Key_Tick(void) {
    static uint8_t count=0;
    static uint8_t previous,now;
    count++;
    //20ms判断一次
    if (count>=20) {
        count=0;
        previous=now;
        now = Key_GetState();
        //检测到松手，previous为键码
        if (!now&&previous) {
            key=previous;
        }
    }

}