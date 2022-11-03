#ifndef __propulsionCmd_H
#define __propulsionCmd_H

#include <cstdint>
#include <stdint.h>
#include "math.h"

#define STOP 50
#define MAX_PWM_LEFT 0
#define MAX_PWM_RIGHT 100

#define MOTOR_GAIN 10
#define TOLERANCE_SPEED 1

/* Calculate rightRearPwmCmd and leftRearPwmCmd (PWM) in MANUAL mode (from joystick orders)
*
* The joystick sends throttle order, which is directly transformed into PWM. The PWMs are equal for both motors in Manual Mode
* 
*/
int * manualPropulsionCmd(float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd);

/* Calculate rightRearPwmCmd and leftRearPwmCmd (PWM) in AUTO mode (from speed command)
*
* The command sends speed order, which is directly transformed into PWM. 
* 
*/
int * autoPropulsionCmd(float requestedSpeed, float currentLeftSpeed, float currentRightSpeed, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd);

#endif /*__ propulsionCmd_H */