#include "hook.h"

/**
* Lock the car hook
**/
void lockHook(void){
    HAL_GPIO_WritePin(GPIO_PORT_HOOK, GPIO_PIN_HOOK, GPIO_PIN_SET);
}


/**
* Unlock the car hook
**/
void unlockHook(void){
    HAL_GPIO_WritePin(GPIO_PORT_HOOK, GPIO_PIN_HOOK, GPIO_PIN_RESET);
}
