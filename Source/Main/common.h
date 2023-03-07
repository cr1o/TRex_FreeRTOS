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

typedef enum {
  APP_OK = 0,
  APP_WARNING = 1,
  APP_ERROR = -1,
  APP_ERROR_TIMEOUT = 3,

  APP_ERROR_BAD_ARGUMENT = 4,     ///< Ошибка аргументов функции (нулевые указатели, выход за диапазон)
  APP_ERROR_MEMORY_ALLOCATE = 5,  ///< Ошибка выделения памяти
  APP_ERROR_SEM_BUSY = 6,         ///< Ошибка захвата семафора
  APP_ERROR_STRUCT_SIGN = 7,      ///< Ошибка идентификатора структуры (не совпал MagicNumber)
  APP_ERROR_ALREADY_INIT = 8,     ///< Ошибка инициализации - объект уже инициализирован
  APP_ERROR_UNKNOWN_TYPE = 9,     ///< Ошибка работы с объектом - неизвестный тип объекта
  APP_ERROR_UNINIT = 10,          ///< Ошибка доступа - объект не инициализирован
  APP_ERROR_NO_FUNCTIONAL = 11    ///< Ошибка: нет такого функционала
} errcode;

//--------------------------  Variable   ----------------------------------
volatile motor m[NUMBER_OF_MOTOR];
//--------------------------  Functions   ----------------------------------
void delay(uint32_t ms);

#endif /* COMMON_H_ */
