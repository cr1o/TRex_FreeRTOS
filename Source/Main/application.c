/**
 ******************************************************************************
 * \file   application.c
 * \date   07.03.2023
 * \author Shirobokov Aleksandr
 */

//=============================================================================
//                                   INCLUDES
//=============================================================================
#include "application.h"
#include "GPIO.h"
#include "DFPlayer.h"
#include "cmsis_os2.h"

//=============================================================================
//                                   DEFINES
//=============================================================================
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

//=============================================================================
//                          STATIC FUNC PROTOTYPES
//=============================================================================
void ControlTask(void *argument);

//=============================================================================
//                          GLOBAL FUNC IMPL
//=============================================================================
errcode Application_Init (void) {
  errcode result = APP_OK;
  // /**
  //  * TODO: Do something if HSE not start correctly
  //  */
  // result = RCC_Init();
  // SystemCoreClockUpdate();
  Motor_Init();
  // GPIO_Init();
  // PWM_Init();
  MP3_Init();
  osDelay(500);
  MP3_SendCmd(MP3_VOLUME, 0, 30); // Volume 0-30
  // GPIOB->BSRR |= GPIO_BSRR_BS_10;	//GREEN LED indicator ON
  LED_test();
  osDelay(500);

  controlTaskHandle = osThreadNew(ControlTask, NULL, &controlTask_attributes);

  return result;
}
//=============================================================================
//                          STATIC FUNC IMPL
//=============================================================================
void ControlTask(void *argument)
{

  for(;;) {
    GPIOC->BSRR |= GPIO_BSRR_BS_9;
    osDelay(500);
    GPIOC->BSRR |= GPIO_BSRR_BR_9;
    osDelay(500);
  }

  while(1) {

    for (uint8_t i = 0; i < NUMBER_OF_MOTOR; i++) {
      if (m[i].Start) {
        if (m[i].curStep < (m[i].tAcceleration ) ){
          m[i].curStep++;
        }
      } else{
        if (m[i].curStep > 0) {
          m[i].curStep--;
        }
      }
    }

    //----------------   M1   ------------------------------------
    if (m[0].EN) {
      if ( ((m[0].Start) == 0) & (m[0].curStep == 0) ) {
        TIM1->CCR1 = 0;
        GPIOA->BSRR |= GPIO_BSRR_BR_15;
      } else {
        TIM1->CCR1 = m[0].perStartPWM + ((m[0].curStep * m[0].StepSizeF) /100 );
        GPIOA->BSRR |= GPIO_BSRR_BS_15;
      }
    }
    //----------------   M2   ------------------------------------
    if (m[1].EN) {
      if ( ((m[1].Start) == 0) & (m[1].curStep == 0) ) {
        TIM1->CCR2 = 0;
        GPIOB->BSRR |= GPIO_BSRR_BR_3;
      } else {
        TIM1->CCR2 = m[1].perStartPWM + ((m[1].curStep * m[1].StepSizeF) / 100 );
        GPIOB->BSRR |= GPIO_BSRR_BS_3;
      }
    }
    //----------------   M3   ------------------------------------
    if (m[2].EN) {
      if ( ((m[2].Start) == 0) & (m[2].curStep == 0) ) {
        TIM1->CCR3 = 0;
        GPIOB->BSRR |= GPIO_BSRR_BR_4;
      } else {
        TIM1->CCR3 = m[2].perStartPWM + ((m[2].curStep * m[2].StepSizeF) / 100 );
        GPIOB->BSRR |= GPIO_BSRR_BS_4;
      }
    }
    //----------------   M4   ------------------------------------
    if (m[3].EN) {
      if ( ((m[3].Start) == 0) & (m[3].curStep == 0) ) {
        TIM1->CCR4 = 0;
        GPIOB->BSRR |= GPIO_BSRR_BR_5;
      } else {
        TIM1->CCR4 = m[1].perStartPWM + ((m[3].curStep * m[3].StepSizeF) / 100 );
        GPIOB->BSRR |= GPIO_BSRR_BS_5;
      }
    }

    //----------------   M5   ------------------------------------
    if (m[4].EN) {
      if ( ((m[4].Start) == 0) & (m[4].curStep == 0) ) {
        TIM3->CCR1 = 0;
        GPIOB->BSRR |= GPIO_BSRR_BR_12;
      } else {
        TIM3->CCR1 = m[4].perStartPWM + ((m[4].curStep * m[4].StepSizeF) / 100 );
        GPIOB->BSRR |= GPIO_BSRR_BS_12;
      }
    }
    //----------------   M6   ------------------------------------
    if (m[5].EN) {
      if ( ((m[5].Start) == 0) & (m[5].curStep == 0) ) {
        TIM3->CCR2 = 0;
        GPIOB->BSRR |= GPIO_BSRR_BR_13;
      } else {
        TIM3->CCR2 = m[5].perStartPWM + ((m[5].curStep * m[5].StepSizeF) / 100 );
        GPIOB->BSRR |= GPIO_BSRR_BS_13;
      }
    }
    //----------------   M7   ------------------------------------
    if (m[6].EN) {
      if ( ((m[6].Start) == 0) & (m[6].curStep == 0) ) {
        TIM3->CCR3 = 0;
        GPIOB->BSRR |= GPIO_BSRR_BR_14;
      } else {
        TIM3->CCR3 = m[6].perStartPWM + ((m[6].curStep * m[6].StepSizeF) / 100 );
        GPIOB->BSRR |= GPIO_BSRR_BS_14;
      }
    }
    //----------------   M8   ------------------------------------
    if (m[7].EN) {
      if ( ((m[7].Start) == 0) & (m[7].curStep == 0) ) {
        TIM3->CCR4 = 0;
        GPIOB->BSRR |= GPIO_BSRR_BR_15;
      } else {
        TIM3->CCR4 = m[7].perStartPWM + ((m[7].curStep * m[7].StepSizeF) / 100 );
        GPIOB->BSRR |= GPIO_BSRR_BS_15;
      }
    }
    //--------------   RED INDICATOR   ----------------------------
    //RED LED is ON if Motion Sensor is triggered
    if ((GPIOB->IDR & GPIO_IDR_9) == 0 ) {
      GPIOB->BSRR |= GPIO_BSRR_BS_11;
    } else {
      GPIOB->BSRR |= GPIO_BSRR_BR_11;
    }
    vTaskDelay(TIME_STEP);
  }



}

/**End of file**/
