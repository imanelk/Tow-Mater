#include "../include/odometry/speedCalcul.h"


/* 
*
* Compute the m/s speed from a RPM.
* 
*/
float rpmToMps(float currentRPM){
    float rearSpeed;
    float perimeter = 2*3.14*WHEEL_DIAMETER/1000; // in meters
    
    rearSpeed = currentRPM*perimeter/60;

    return rearSpeed;
}


