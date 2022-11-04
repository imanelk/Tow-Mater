#include "../include/car_control/propulsionCmd.h"


/* Calculate rightRearPwmCmd and leftRearPwmCmd (PWM) in MANUAL mode (from joystick orders)
*
* The joystick sends throttle order, which is directly transformed into PWM. The PWMs are equal for both motors in Manual Mode
* 
*/
int * manualPropulsionCmd(float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd){

    if (reverse){
        leftRearPwmCmd = 50 - 50*requestedThrottle;    //requestedThrottle : [0 ; 1] => PWM : [50 -> 0] (reverse)

    } else{
        leftRearPwmCmd = 50 + 50*requestedThrottle;    //requestedThrottle : [0 ; 1] => PWM : [50 -> 100] (forward)
    }

    rightRearPwmCmd = leftRearPwmCmd;

    return 0;

}

/* Calculate rightRearPwmCmd and leftRearPwmCmd (PWM) in AUTO mode (from speed command)
*
* The command sends speed order, which is directly transformed into PWM. 
* 
*/
int * autoPropulsionCmd(float requestedSpeed, float currentLeftSpeed, float currentRightSpeed, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd){
   float errorSpeed = currentLeftSpeed - requestedSpeed;

   //Command's calculation
	if (abs(errorSpeed)<TOLERANCE_SPEED){
		leftRearPwmCmd = STOP;
	}
	else {
        // TO DO : implement the regulation here
	}

    rightRearPwmCmd = leftRearPwmCmd;


    return 0;
}
