#ifndef __motion_planning_node_HPP
#define __motion_planning_node_HPP

#include <stdint.h>
#include <string.h>  

#define PERIOD_UPDATE_MOTION 5ms    //Update period of the movement calculation
#define PERIOD_CHECK_SECURITY 10ms //Update period of the propulsion and steering commands in [ms]

#define INITIAL_VELOCITY 0
#define INITIAL_STEER 0

#define MAX_VELOCITY 0.35   // [m/s]
#define TOWING_VELOCITY (0.8*MAX_VELOCITY)
#define FINAL_REVERSE_VELOCITY -(0.5*MAX_VELOCITY)

#define LOCK_DISTANCE 7    // Lock distance (between US rear center and the damaged car) [cm]
#define LOCK_WAITING_TIME 4 //Waiting time between the lock order and the beginning of the tow [s]

#define NUMBER_OF_CRITICAL_AREAS 6  //Critical areas 

#endif /*__motion_planning_node_HPP */