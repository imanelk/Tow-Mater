#include "../include/odometry/speedCalcul.h"


/* 
*
* Compute the m/s speed from a RPM.
* 
*/
float rpmToMps(float currentRPM){
    float rearSpeed;
    
    rearSpeed = 60*currentRPM/(WHEEL_DIAMETER*1000);

    return rearSpeed;
}


