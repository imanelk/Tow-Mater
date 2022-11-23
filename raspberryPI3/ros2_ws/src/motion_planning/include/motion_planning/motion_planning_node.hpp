#ifndef __motion_planning_node_HPP
#define __motion_planning_node_HPP

#include <stdint.h>
#include <string.h>  

#define PERIOD_UPDATE_MOTION 5ms    // Update period of the movement calculation

#define INITIAL_VELOCITY 0.0
#define INITIAL_STEER 0.0

// ---- Final reverse and towing ----

#define REVERSE_VELOCITY -1.0
#define FINAL_REVERSE_VELOCITY -0.7 // [m/s]

#define LOCK_WAITING_TIME 4 //Waiting time after the lock order [s]

#define TOWING_VELOCITY 1.0 // [m/s] 
#define TOWING_DISTANCE 5000 // [cm]

// ---- Trajectories

// No U-turn (NUT)

#define NB_NUT_POINTS 3



// Security
#define LLS_DISTANCE 15 //LowLevelSecurityDistance = 20cm (LLS_DISTANCE)


 

#endif /*__motion_planning_node_HPP */