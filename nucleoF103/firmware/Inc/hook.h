#ifndef hook_h
#define hook_h

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"


#define GPIO_PORT_HOOK GPIOC
#define GPIO_PIN_HOOK GPIO_PIN_7

/**
* Lock the car hook
**/
void lockHook(void);

/**
* Unlock the car hook
**/
void unlockHook(void);


#endif /* hook_h */
