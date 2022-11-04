#ifndef __propulsionCmd_H
#define __propulsionCmd_H

#include <cstdint>
#include <stdint.h>
#include "math.h"

#define STOP 50
#define MAX_PWM_LEFT 0
#define MAX_PWM_RIGHT 100


#define WHEEL_DIAMETER 195 //Wheel diameters in [mm]

/* Calculate rightRearPwmCmd and leftRearPwmCmd (PWM) in MANUAL mode (from joystick orders)
*
* The joystick sends throttle order, which is directly transformed into PWM. The PWMs are equal for both motors in Manual Mode
* 
*/
int * manualPropulsionCmd(float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd);

/* Calculate RearPwmCmd  (PWM) in AUTO mode (from speed command)
*
* The command sends speed order, which is directly transformed into PWM. 
* This regulation has to be done on each wheels.
* 
*/
int * autoPropulsionCmd(float requestedSpeed, float currentSpeed,  uint8_t& RearPwmCmd, float& errorPrevious);

/* 
*
* Compute the RPM speed from a m/s.
* 
*/
float mpsToRpm(float currentMPS);



#endif /*__ propulsionCmd_H */