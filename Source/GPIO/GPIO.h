#ifndef GPIO_H_
#define GPIO_H_


#include "stm32f0xx.h"
#include "main.h"
#include "common.h"


void PWM_Init(void);
errcode RCC_Init(void);
void GPIO_Init(void);
void Motor_Init(void);
void LED_test(void);

#endif /* GPIO_H_ */
