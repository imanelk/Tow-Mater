#include "wheels.h"


void wheels_set_speed(GPIO_PinState en_right, GPIO_PinState en_left, int speed_right, int speed_left){

	//Normalize value between MAX_SPEED_FORWARD and MAX_SPEED_BACKWARD (because CAN order is between 0 and 100)
	speed_right = ((MAX_SPEED_FORWARD-MAX_SPEED_BACKWARD)/100.0) * speed_right + MAX_SPEED_BACKWARD;
	speed_left = ((MAX_SPEED_FORWARD-MAX_SPEED_BACKWARD)/100.0) * speed_left + MAX_SPEED_BACKWARD;

	/* Threshold rotating speed of propulsion wheels*/
	if (speed_left > MAX_SPEED_FORWARD){
		speed_left = MAX_SPEED_FORWARD;
	}else if (speed_left < MAX_SPEED_BACKWARD)
		speed_left = MAX_SPEED_BACKWARD;


	if (speed_right > MAX_SPEED_FORWARD){
		speed_right = MAX_SPEED_FORWARD;
	}else if (speed_right < MAX_SPEED_BACKWARD){
		speed_right = MAX_SPEED_BACKWARD;
	}

		
	speed_left = 3200 * ( speed_left/ 100.0 );
	speed_right = 3200 * ( speed_right/ 100.0 );

	TIM1->CCR1=speed_left;
	TIM1->CCR2=speed_right;

	/*        Enable moteurs        */
	/* GPIO_PIN_SET : activation    */
	/* GPIO_PIN_RESET : pont ouvert */
		
	HAL_GPIO_WritePin( GPIOC, GPIO_PIN_10, en_left); //PC10  Right Rear
	HAL_GPIO_WritePin( GPIOC, GPIO_PIN_11, en_right); //PC11  Left Rear
}
