#ifndef __motion_planning_node_HPP
#define __motion_planning_node_HPP

#include <stdint.h>
#include <string.h>  

#define PERIOD_UPDATE_MOTION 5ms    // Update period of the movement calculation
#define PERIOD_CHECK_SECURITY 10ms // Update period of the propulsion and steering commands in [ms]

#define INITIAL_VELOCITY 0
#define INITIAL_STEER 0

#define MAX_VELOCITY 1.2   // [m/s]

// ---- Final reverse and towing ----

#define FINAL_REVERSE_VELOCITY -0.8 // [m/s]

#define LOCK_DISTANCE 23    // Lock distance (between US rear center and the damaged car) [cm]
#define LOCK_WAITING_TIME 4 //Waiting time after the lock order [s]

#define TOWING_VELOCITY 1.0 // [m/s]
#define TOWING_DURATION 10000ms 
 

#endif /*__motion_planning_node_HPP */