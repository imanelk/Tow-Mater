#ifndef _TIMER_H_
#define _TIMER_H_

#include "stm32f1xx_hal.h"
#include <math.h>

//Start TIM3 counter
void startMicroSecondCounter();

//Count overflows
void microSecondTimerOverflow();

//return current time in microsecond
uint64_t microSecond();

#endif
