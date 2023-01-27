#ifndef _STERRING_H_
#define _STERRING_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#include <FLASH_PAGE_F1.h>
#include "stdlib.h"
#include "math.h"

#define LEFT_MAX_SPEED_STEERING 10 // Left steering max speed
#define RIGHT_MAX_SPEED_STEERING 90 // Right steering max speed

#define NO_STEERING 50

#define pin_bt_right_front_wheel GPIO_PIN_14
#define pin_bt_left_front_wheel GPIO_PIN_15

#define ANGLE_LEFT_VALUE 	0
#define ANGLE_CENTER_VALUE 	100
#define ANGLE_RIGHT_VALUE	200

#define STEERING_CALIBRATION_A_ADDR 0x0800A100	//address coef a
#define STEERING_CALIBRATION_B_ADDR 0x0800B100	//address coef b


#define STEERING_CALIBRATION_A_DEFAULT_ADDR 0x0800D100	//address coef a (default value)
#define STEERING_CALIBRATION_B_DEFAULT_ADDR 0x0800E100	//address coef b (default value)


//CAPT_L_DEFAULT 2675
//CAPT_C_DEFAULT 2379
//CAPT_R_DEFAULT 1790

//currentAngle = (steering_sensor_coef_a*steeringSensor + steering_sensor_coef_b);
//a : ((ANGLE_RIGHT_VALUE-ANGLE_LEFT_VALUE)/((CAPT_R_DEFAULT-CAPT_L_DEFAULT)*(CAPT_R_DEFAULT-CAPT_C_DEFAULT)) - ((ANGLE_CENTER_VALUE-ANGLE_LEFT_VALUE)/((CAPT_C_DEFAULT-CAPT_L_DEFAULT)*(CAPT_R_DEFAULT-CAPT_C_DEFAULT))))

#define STEERING_CALIBRATION_A_DEFAULT	-0.23866348 	//coef a (default value)
#define STEERING_CALIBRATION_B_DEFAULT 	666.109802	//coef b (default value)


/**
*	Set the max and min value of the steering wheels sensor
**/
void steering_Init(void);

/**
* Set the speed of the steering motor. Speed value has to be between 0% and 100%
**/
void steering_set_speed(GPIO_PinState en_steering, int speed);

/**
* Return the steering angle.
**/
int steering_get_angle(void);

/**
* Command the front wheel position
**/
void steering_set_position (GPIO_PinState en_steering, int msg_CAN);

/**
 * Cmd wheels with buttons
 */
void steering_move_with_button(void);

/**
* Return 1 if a steering control button is pressed
*/
int steering_is_a_button_pressed(void);

//Update calibration data from flash memory
void readCalibrationData();


#endif
