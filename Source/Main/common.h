#ifndef COMMON_H_
#define COMMON_H_

// //--------------------------   Lib   -------------------------------------
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//------------------------   Defines   -----------------------------------
#define NUMBER_OF_MOTOR   8
#define TIME_STEP         10

typedef struct {
  uint8_t EN;
  uint8_t Start;
  uint16_t tAcceleration; //The number of overclocking steps. Step 10ms
  uint16_t perStartPWM;   //% of the maximum number of engine revolutions from which acceleration begins
  uint16_t perMaxPWM;     //Set speed. (% of maximum revolutions)
  uint16_t StepSize;      //Estimated step size

  uint16_t StepSizeF;     //Estimated step size

  uint16_t curStep;       //Current acceleration/deceleration step
  uint16_t M_EN;
  uint16_t M_PWM;
} motor;

//--------------------------  Variable   ----------------------------------
volatile motor m[NUMBER_OF_MOTOR];
//--------------------------  Functions   ----------------------------------
void delay(uint32_t ms);

#endif /* COMMON_H_ */
