#ifndef hook_h
#define hook_h

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "main.h"

/**
* Lock the car hook
**/
void lockHook(void);

/**
* Unlock the car hook
**/
void unlockHook(void);


#endif /* hook_h */
