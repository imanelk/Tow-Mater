/*
 * control.c
 *
 *  Created on: 12 nov. 2021
 *      Author: Carole Meyer
 */


/* Includes ------------------------------------------------------------------*/

#include "control.h"


/* Private define ------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Programs ------------------------------------------------------------------*/

/**
*	Update motor speeds
**/
void car_control(int left_rear_speed, int right_rear_speed, int steering_speed){

	if (left_rear_speed == DISABLED && right_rear_speed == DISABLED && steering_speed == DISABLED){

		wheels_set_speed(GPIO_PIN_RESET, GPIO_PIN_RESET, STOP, STOP);
		steering_set_speed(GPIO_PIN_RESET, STOP);

	} else {

		//Propulsion
		wheels_set_speed(GPIO_PIN_SET, GPIO_PIN_SET, right_rear_speed, left_rear_speed);

		//Steering
		steering_set_speed(GPIO_PIN_SET, steering_speed);
	}
}

