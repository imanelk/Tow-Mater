
/* Includes ------------------------------------------------------------------*/

#include "steering.h"


/* Private define ------------------------------------------------------------*/



/* Private variables ---------------------------------------------------------*/

int need_read_calibration = 1;

float steering_sensor_coef_a;
float steering_sensor_coef_b;

extern uint32_t ADCBUF[5];

/* Programs ------------------------------------------------------------------*/

//Set the speed of the steering motor. Speed value has to be between 0% and 100%
void steering_set_speed(GPIO_PinState en_steering, int speed){
    
	//Normalize value between LEFT_MAX_SPEED_STEERING and RIGHT_MAX_SPEED_STEERING (because CAN order is between 0 and 100)
	speed = ((RIGHT_MAX_SPEED_STEERING-LEFT_MAX_SPEED_STEERING)/100.0) * speed + LEFT_MAX_SPEED_STEERING;

    /* Threshold rotating speed of steering wheels*/
    if (speed > RIGHT_MAX_SPEED_STEERING){
        speed = RIGHT_MAX_SPEED_STEERING;
    } else if (speed < LEFT_MAX_SPEED_STEERING){
        speed  = LEFT_MAX_SPEED_STEERING;
    }
    
    speed = 3200 * ( speed/ 100.0 );
    TIM1->CCR3 = speed;
    
    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_12, en_steering);  //PC12  AV
}

//return the current angle between 0 (full left) and 200 (full right)
int steering_get_angle(void){
	if (need_read_calibration==1){
		readCalibrationData();
		need_read_calibration=0;
	}

	int steeringSensor = (int) ADCBUF[1];
    int currentAngle = (steering_sensor_coef_a*steeringSensor + steering_sensor_coef_b);
    if (currentAngle > ANGLE_RIGHT_VALUE){
    	currentAngle = ANGLE_RIGHT_VALUE;

    }else if (currentAngle < ANGLE_LEFT_VALUE){
    	currentAngle = ANGLE_LEFT_VALUE;
    }

    return currentAngle;
}


int steering_is_a_button_pressed(){
	return ((!HAL_GPIO_ReadPin(GPIOB, pin_bt_right_front_wheel)) || (!HAL_GPIO_ReadPin(GPIOB, pin_bt_left_front_wheel)));
}	

//move steering with L/R buttons
void steering_move_with_button(void){
    static int previous_value_right = GPIO_PIN_RESET;
    static int previous_value_left = GPIO_PIN_RESET;
    int current_value_right = !HAL_GPIO_ReadPin(GPIOB, pin_bt_right_front_wheel);
    int current_value_left = !HAL_GPIO_ReadPin(GPIOB, pin_bt_left_front_wheel);
    
    if (
				((current_value_right == GPIO_PIN_SET) && (current_value_left == GPIO_PIN_SET))
        || ((current_value_right == GPIO_PIN_RESET) && (previous_value_right == GPIO_PIN_SET))
        || ((current_value_left == GPIO_PIN_RESET) && (previous_value_left == GPIO_PIN_SET))
				){
        steering_set_speed(GPIO_PIN_RESET, NO_STEERING);
        previous_value_right = GPIO_PIN_RESET;
        previous_value_left = GPIO_PIN_RESET;
    } else if ((current_value_right == GPIO_PIN_SET) && (previous_value_right == GPIO_PIN_RESET)){
        steering_set_speed(GPIO_PIN_SET, RIGHT_MAX_SPEED_STEERING);
        previous_value_right = GPIO_PIN_SET;
    } else if ((current_value_left == GPIO_PIN_SET) && (previous_value_left == GPIO_PIN_RESET)){
           steering_set_speed(GPIO_PIN_SET, LEFT_MAX_SPEED_STEERING);
           previous_value_left = GPIO_PIN_SET;
    }
}

//Read steering coefficients (a and b) in flash memory
void readCalibrationData(){
	steering_sensor_coef_a = Flash_Read_NUM(STEERING_CALIBRATION_A_ADDR);
	steering_sensor_coef_b = Flash_Read_NUM(STEERING_CALIBRATION_B_ADDR);
}
