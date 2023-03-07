#ifndef GPIO_H_
#define GPIO_H_


#include "stm32f0xx.h"
#include "main.h"

void PWM_Init(void);
void RCC_Init(void);
void GPIO_Init(void);
void Motor_Init(void);
void LEDtst(void);

#endif /* GPIO_H_ */
