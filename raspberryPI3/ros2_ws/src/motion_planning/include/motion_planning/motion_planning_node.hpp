#ifndef __motion_planning_node_HPP
#define __motion_planning_node_HPP

#include <stdint.h>
#include <string.h>  

#define PERIOD_UPDATE_MOTION 5ms    // Update period of the movement calculation

#define INITIAL_VELOCITY 0.0
#define INITIAL_STEER 0.0

#define TOLERANCE_STEER 0.1

// ---- Final reverse and towing ----

#define REVERSE_VELOCITY -1.0
#define FINAL_REVERSE_VELOCITY -0.5 // [m/s]

#define LOCK_WAITING_TIME 1 //Waiting time after the lock order [s]

#define TOWING_VELOCITY 1.0 // [m/s] 
#define TOWING_DISTANCE 4000 // [cm]

#define MIN_DISTANCE_AVOIDANCE 30 // minimum distance from the obstacle to start the avoidance process

// ---- Trajectories

// No U-turn (NUT)

#define NB_NUT_POINTS 3

// U-turn (UT)

#define NB_UT_POINTS 4

// Avoidance
#define NB_AVOIDANCE_POINTS 5

// Security
#define NS_DISTANCE 75 //NormalSecurity Distance [cm]
#define LLS_DISTANCE 15 //LowLevelSecurity Distance [cm]
#define AVOIDANCE_DISTANCE 25 //Avoidance Distance [cm]
#define TOW_DISTANCE 100 //Towing Security Distance [cm]


// State machine
#define INITIAL_STATE_MACHINE idle
#define INITIAL_AUTONOMOUS move
#define INITIAL_MOVE analyse
 

#endif /*__motion_planning_node_HPP */