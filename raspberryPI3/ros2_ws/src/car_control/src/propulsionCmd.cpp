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


/* Calculate RearPwmCmd  (PWM) in AUTO mode (from speed command)
*
* The command sends speed order, which is directly transformed into PWM. 
* This regulation has to be done on each wheels.
* 
*/
int * autoPropulsionCmd(float requestedSpeed, float currentSpeed,  uint8_t& RearPwmCmd){
    //PID parameters
    const float kp = 200; 
    const float ki = 5; 
    const float kd = 100; 

    //Errors
    float errorSum = 0;
    float errorPrevious = requestedSpeed; 

    // P
    float errorCurrent = requestedSpeed-currentSpeed;

    // I
    errorSum += errorCurrent;  
    
    // D
    float errorSub = errorCurrent - errorPrevious; 
    errorPrevious = errorCurrent;
    
    RearPwmCmd = kp*errorCurrent + ki*errorSum + kd*errorSub; 
  
    return 0;
}

/* 
*
* Compute the RPM speed from a m/s.
* 
*/
float mpsToRpm(float currentMPS){
    float rearSpeed;
    
    rearSpeed = currentMPS*(WHEEL_DIAMETER*1000)/60;

    return rearSpeed;
}