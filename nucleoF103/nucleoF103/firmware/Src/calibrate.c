/*
 * calibrate.c
 *
 *  Created on: 19 nov. 2021
 *      Author: carol
 */


#include "calibrate.h"


/* Private define ------------------------------------------------------------*/



/* Private variables ---------------------------------------------------------*/



extern int need_read_calibration;

extern uint32_t ADCBUF[5];


/* Programs ------------------------------------------------------------------*/

/*
 * Verifie si le user button bleu est appuyé
 */
int is_blue_button_pressed(){
	return (!HAL_GPIO_ReadPin(GPIOC, pin_blue_user_button));
}

/*
 *	Performe la calibration du module de direction (action moteur, recuperation valeur capteur)
 */
void calibrate(void){
	int right_ok = 0;
	int left_ok = 0;
	int centre_ok = 0;

	float capt_L = 0.0;
	float capt_R = 0.0;
	float capt_C = 0.0;

	float coef_a;
	float coef_b;

	car_control(DISABLED, DISABLED, DISABLED);	//Stop the car

	uint8_t dataCalibration[8] = {CALIBRATION_IN_PROGRESS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN_Send(dataCalibration, CAN_ID_CALIBRATION_MODE);	//Indicate that calibration is in progress (to raspberry)

	if (!right_ok){
		right_ok = 1;
		steering_set_speed(GPIO_PIN_SET, RIGHT_MAX_SPEED_STEERING);
		HAL_Delay(7000);
		capt_R = ADCBUF[1];
	}
	if (!left_ok){
		left_ok = 1;
		steering_set_speed(GPIO_PIN_SET, LEFT_MAX_SPEED_STEERING);
		HAL_Delay(7000);
		capt_L = ADCBUF[1];
	}
	if (!centre_ok){
		dataCalibration[1] = CALIBRATION_USER_NEED;
		CAN_Send(dataCalibration, CAN_ID_CALIBRATION_MODE);	//Indicate that calibration need user action (to raspberry)

		while (1){
			steering_move_with_button();
			if (is_blue_button_pressed()){
				centre_ok = 1;
				capt_C = ADCBUF[1];
				break;
			}
		}
	}


	if (centre_ok){
		int LC = abs(capt_L-capt_C);
		int RC = abs(capt_C-capt_R);

		if (LC<RC){
			coef_a = (ANGLE_CENTER_VALUE-ANGLE_LEFT_VALUE)/(capt_C-capt_L);
		}else{
			coef_a = (ANGLE_CENTER_VALUE-ANGLE_RIGHT_VALUE)/(capt_C-capt_R);
		}

		coef_b = ANGLE_CENTER_VALUE - coef_a*capt_C;

		Flash_Write_NUM(STEERING_CALIBRATION_A_ADDR, coef_a);
		Flash_Write_NUM(STEERING_CALIBRATION_B_ADDR, coef_b);


		need_read_calibration = 1;  //Indicate that steering calibration values has changed

		//Ckeck if Flash_write succeeded
		if (Flash_Read_NUM(STEERING_CALIBRATION_A_ADDR) == coef_a
			&& Flash_Read_NUM(STEERING_CALIBRATION_B_ADDR) == coef_b)
		{
			dataCalibration[0] = CALIBRATION_SUCCESS;

		}else{
			dataCalibration[0] = CALIBRATION_FAIL;
		}


		CAN_Send(dataCalibration, CAN_ID_CALIBRATION_MODE);	//Indicate calibration status (success/fail) (to raspberry)

	}

}


