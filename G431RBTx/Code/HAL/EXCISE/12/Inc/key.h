#ifndef __KEY_H__
#define __KEY_H__
#include "main.h"

#define  B1 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)
#define  B2 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)
#define  B3 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)
#define  B4 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)

void KeyScan(void);

#endif


