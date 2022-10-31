#include "hook.h"

void lockHook(void){
    HAL_GPIO_WritePin(GPIO_PORT_HOOK, GPIO_PIN_HOOK, GPIO_PIN_SET);
}


void unlockHook(void){
    HAL_GPIO_WritePin(GPIO_PORT_HOOK, GPIO_PIN_HOOK, GPIO_PIN_RESET);
}
