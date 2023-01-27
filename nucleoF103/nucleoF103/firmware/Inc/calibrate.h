/*
 * calibrate.h
 *
 *  Created on: 19 nov. 2021
 *      Author: Carole Meyer
 */

#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "steering.h"
#include "can.h"
#include "control.h"
#include "stdlib.h"
#include <FLASH_PAGE_F1.h>


#define pin_blue_user_button GPIO_PIN_13

#define MAX(a,b) ((a)>(b) ? (a) : (b))

/**
*	Performe la calibration du module de direction (action moteur, recuperation valeur capteur)
**/
void calibrate(void);

#endif /* CALIBRATE_H_ */
