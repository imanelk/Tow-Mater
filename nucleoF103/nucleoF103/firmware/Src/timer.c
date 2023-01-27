#include "timer.h"

uint64_t timerOverflow = 0;
uint64_t microSecondTime = 0;

//Start TIM3 counter
void startMicroSecondCounter(){
	TIM3->CR1 |= TIM_CR1_CEN;
}

//Count overflows
void microSecondTimerOverflow(){
	timerOverflow +=1 ;
}

//return current time in microsecond
uint64_t microSecond(){
	microSecondTime = timerOverflow * TIM3->ARR + TIM3->CNT;
	return microSecondTime;

}
