#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "../include/motion_planning/motion_planning_node.hpp"

#include "interfaces/msg/cmd_vel.hpp"
#include "interfaces/msg/cmd_steer.hpp"
#include "interfaces/msg/hook.hpp"
#include "interfaces/msg/obstacles.hpp"
#include "interfaces/msg/fixed_obstacles.hpp"
#include "interfaces/msg/distance.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/avoidance_parameters.hpp"
#include "interfaces/msg/orientation.hpp"

using namespace std;
using placeholders::_1;


class motion_planning : public rclcpp::Node {

public:
    motion_planning()
    : Node("motion_planning_node")
    {
        currentVelocity = -1; 
        currentSteer = -1;

        //No U-Turn trajectory

        nutTraj[0].velocity = 0.8;
        nutTraj[0].angle = 1.0;
        nutTraj[0].distance = 161.0;

        nutTraj[1].velocity = 0.8;
        nutTraj[1].angle = -1.0;
        nutTraj[1].distance = 139.0;

        nutTraj[2].velocity = 0.0;
        nutTraj[2].angle = 0.0;
        nutTraj[2].distance = 0.0;

        //U-Turn trajectory

        utTraj[0].velocity = -0.8;
        utTraj[0].angle = -1.0;
        utTraj[0].distance = 332.0;

        utTraj[1].velocity = 0.8;
        utTraj[1].angle = 1.0;
        utTraj[1].distance = 187.0;

        utTraj[2].velocity = -0.8;
        utTraj[2].angle = -1.0;
        utTraj[2].distance = 161.0;

        utTraj[3].velocity = 0.0;
        utTraj[3].angle = 0.0;
        utTraj[3].distance = 0.0;

        // --- Avoidance Trajectories---

        //Short Left

        shortLeftTraj[0].velocity = 0.8;
        shortLeftTraj[0].angle = -1.0;
        shortLeftTraj[0].distance = 70.0;

        shortLeftTraj[1].velocity = 0.8;
        shortLeftTraj[1].angle = 0.0;
        shortLeftTraj[1].distance = 72.0;

        shortLeftTraj[2].velocity = 0.8;
        shortLeftTraj[2].angle = +1.0;
        shortLeftTraj[2].distance = 205.0;

        shortLeftTraj[3].velocity = 0.8;
        shortLeftTraj[3].angle = -1.0;
        shortLeftTraj[3].distance = 100.0;

        shortLeftTraj[4].velocity = 0.0;
        shortLeftTraj[4].angle = 0.0;
        shortLeftTraj[4].distance = 0.0;

        //Short Right

        shortRightTraj[0].velocity = 0.8;
        shortRightTraj[0].angle = 1.0;
        shortRightTraj[0].distance = 76.0;

        shortRightTraj[1].velocity = 0.8;
        shortRightTraj[1].angle = 0.0;
        shortRightTraj[1].distance = 58.0;

        shortRightTraj[2].velocity = 0.8;
        shortRightTraj[2].angle = -1.0;
        shortRightTraj[2].distance = 163.0;

        shortRightTraj[3].velocity = 0.8;
        shortRightTraj[3].angle = 1.0;
        shortRightTraj[3].distance = 105.0;

        shortRightTraj[4].velocity = 0.0;
        shortRightTraj[4].angle = 0.0;
        shortRightTraj[4].distance = 0.0;


        // Large Left

        largeLeftTraj[0].velocity = -0.8;
        largeLeftTraj[0].angle = 1.0;
        largeLeftTraj[0].distance = 40.0;

        largeLeftTraj[1].velocity = 0.8;
        largeLeftTraj[1].angle = -1.0;
        largeLeftTraj[1].distance = 92.0;

        largeLeftTraj[2].velocity = 0.8;
        largeLeftTraj[2].angle = +1.0;
        largeLeftTraj[2].distance = 273.0;

        largeLeftTraj[3].velocity = 0.8;
        largeLeftTraj[3].angle = -1.0;
        largeLeftTraj[3].distance = 111.0;

        largeLeftTraj[4].velocity = 0.0;
        largeLeftTraj[4].angle = 0.0;
        largeLeftTraj[4].distance = 0.0;

        // Large Right

        largeRightTraj[0].velocity = -0.8;
        largeRightTraj[0].angle = -1.0;
        largeRightTraj[0].distance = 40.0;

        largeRightTraj[1].velocity = 0.8;
        largeRightTraj[1].angle = 1.0;
        largeRightTraj[1].distance = 89.0;

        largeRightTraj[2].velocity = 0.8;
        largeRightTraj[2].angle = -1.0;
        largeRightTraj[2].distance = 216.0;

        largeRightTraj[3].velocity = 0.8;
        largeRightTraj[3].angle = 1.0;
        largeRightTraj[3].distance = 111.0;

        largeRightTraj[4].velocity = 0.0;
        largeRightTraj[4].angle = 0.0;
        largeRightTraj[4].distance = 0.0;

        setAvoidanceTraj(shortLeftTraj);


        publisher_cmd_vel_= this->create_publisher<interfaces::msg::CmdVel>("consign_speed", 10);
        publisher_cmd_steer_= this->create_publisher<interfaces::msg::CmdSteer>("consign_steer", 10);
        publisher_cmd_hook_= this->create_publisher<interfaces::msg::Hook>("hook", 10);

       
        subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
        "joystick_order", 10, std::bind(&motion_planning::joystickOrderCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&motion_planning::motorsFeedbackCallback, this, _1));

        subscription_hook_ = this->create_subscription<interfaces::msg::Hook>(
        "hook", 10, std::bind(&motion_planning::hookCallback, this, _1));

        subscription_obstacles_ = this->create_subscription<interfaces::msg::Obstacles>(
        "obstacle", 10, std::bind(&motion_planning::obstaclesCallback, this, _1));

        subscription_fixed_obstacles_ = this->create_subscription<interfaces::msg::FixedObstacles>(
        "fixed_obstacles", 10, std::bind(&motion_planning::fixedObstaclesCallback, this, _1));

        subscription_avoidance_parameters_ = this->create_subscription<interfaces::msg::AvoidanceParameters>(
        "avoidance_parameters", 10, std::bind(&motion_planning::avoidanceParametersCallback, this, _1));

        subscription_distance_ = this->create_subscription<interfaces::msg::Distance>(
        "distance", 10, std::bind(&motion_planning::distanceCallback, this, _1));

        subscription_ultrasonic_ = this->create_subscription<interfaces::msg::Ultrasonic>(
        "us_data", 10, std::bind(&motion_planning::ultrasonicCallback, this, _1));

        subscription_orientation_ = this->create_subscription<interfaces::msg::Orientation>(
        "orientation", 10, std::bind(&motion_planning::orientationCallback, this, _1));

        timer_motion_planning_ = this->create_wall_timer(PERIOD_UPDATE_MOTION, std::bind(&motion_planning::motionPlanning, this));
 
        sleep(2);   //Waiting for car_control to start
        RCLCPP_INFO(this->get_logger(), "motion_planning_node READY");

        sendVel(INITIAL_VELOCITY);
        sendSteer(INITIAL_STEER,true);

        setInitialStates();
        safeMode = true;
        setSecurityDistance(NS_DISTANCE,NS_DISTANCE);

    }

    
private:

    struct VAD_POINT {
        float velocity;
        float angle;
        float distance;
    };

    /* Update start and mode from joystick order [callback function]  :
    *
    *  This function is called when a message is published on the "/joystick_order" topic
    * 
    */
    void joystickOrderCallback(const interfaces::msg::JoystickOrder & joyOrder) {

        start = joyOrder.start;

        if (joyOrder.mode == 0 || joyOrder.mode == 2)
            manualMode = true;  //Manual Mode
        else if (joyOrder.mode == 1)
            manualMode = false; //Autonomous Mode

        if (joyOrder.reset){
            reset();
            setInitialStates();
        }
            

        if (mode !=4 && joyOrder.mode ==4){
            mode = 4;
            start = false;
            moveState = 0;
            reset();
            RCLCPP_INFO(this->get_logger(), "---------- SELECT mode ----------");

        }
            

        if (joyOrder.mode == 4){
            start = false;

            if (joyOrder.change_state){

                moveState += 1;

                if (moveState == 6)
                    moveState = 1;

                switch (moveState) {
                    case 1:
                        RCLCPP_INFO(this->get_logger(), "Switch to 'Analyse' ? ");
                        break;
                    case 2:
                        RCLCPP_INFO(this->get_logger(), "Switch to 'No U-Turn' ? ");
                        break;
                    case 3:
                        RCLCPP_INFO(this->get_logger(), "Switch to 'U-Turn' ? ");
                        break;
                    case 4:
                        RCLCPP_INFO(this->get_logger(), "Switch to 'Reverse' ? ");
                        break;
                    case 5:
                        RCLCPP_INFO(this->get_logger(), "Switch to 'Tow' ? ");
                        break;
                }

                
            }
        }

        if (mode == 4 && joyOrder.mode !=4){
            mode = joyOrder.mode;
            idle = true;
            move = true;
            changeState(moveState);

            RCLCPP_INFO(this->get_logger(), "---------- SELECT mode [exit] ----------");
            RCLCPP_INFO(this->get_logger(), "Press 'start' to restart");
        }
                

    }

    /* Update feedbackSteer from motors feedback [callback function]  :
    *
    * This function is called when a message is published on the "/motors_feedback" topic
    * 
    */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
        feedbackSteer = motorsFeedback.steering_angle;
    }

    /* Update orientationOK and orientationReceived [callback function]  :
    *
    * This function is called when a message is published on the "/orientation" topic
    * The two cars are in the same orientation => orientationOK = true => no U-turn
    * The two cars are not in the same orientation => orientationOK = false => U-turn
    */
    void orientationCallback(const interfaces::msg::Orientation & orientationMsg){
        orientationOK = orientationMsg.same_orientation;
        orientationReceived = true;
    }


    /* Update usRearLeft and usRearRight from ultrasonic feedback [callback function]  :
    *
    * This function is called when a message is published on the "/ultrasonic" topic
    * usRearLeft and usRearRight are used to correct the final reverse trajectory
    */
    void ultrasonicCallback(const interfaces::msg::Ultrasonic & usonic){
        usRearLeft = usonic.rear_left ;
        usRearRight = usonic.rear_right ;
    }


    /* Update "hookDetected" from hook status [callback function]  :
    *
    * This function is called when a message is published on the "/hook" topic
    * 
    */
    void hookCallback(const interfaces::msg::Hook & hookMsg) {

        if (hookMsg.type == "detect" && hookMsg.status == true){

            if (!hookDetected){
                RCLCPP_INFO(this->get_logger(), "Hook detected");
                hookDetected = true;
                setSecurityDistance(NS_DISTANCE,LLS_DISTANCE);
            }
            hookPos_x = hookMsg.x;


            hookDistance = 40.0;    //TO-DO : Get the real value

        } else if (hookMsg.type == "fdc")
            hookFdc = hookMsg.status;   //End of course sensor
        
    }


    /* Update distance from distance topic [callback function]  :
    *
    * This function is called when a message is published on the "/distance" topic
    * 
    */
    void distanceCallback(const interfaces::msg::Distance & distanceMsg){
 
        distanceTravelled += distanceMsg.total - lastDistance;
        distanceTravelledAvoidance += distanceMsg.total - lastDistance;
        lastDistance = distanceMsg.total;
    }


    /* Update areaStatus from obstacles feedback [callback function]  :
    *
    * This function is called when a message is published on the "/obstacle" topic
    * 
    */
    void obstaclesCallback(const interfaces::msg::Obstacles & obstaclesMsg){

        vector<int> frontObstacles;
        vector<int> rearObstacles;

        if (obstaclesReceived != 1){
            RCLCPP_WARN(this->get_logger(),"Obstacle analysis received [OK]");
            obstaclesReceived = 1;
        }

        //Front obstacles
        for (int i=0;i<3;i++){
            if (obstaclesMsg.area[i] != -1)
                frontObstacles.push_back(obstaclesMsg.area[i]);
        }

        if (frontObstacles.size() == 0)
            frontObstacleDistance = -1;
        else
            frontObstacleDistance = *min_element(frontObstacles.begin(),frontObstacles.end());
        

        //Rear obstacles
        for (int i=3;i<6;i++){
            if (obstaclesMsg.area[i] != -1)
                rearObstacles.push_back(obstaclesMsg.area[i]);
        }

        if (rearObstacles.size() == 0)
            rearObstacleDistance = -1;
        else{
            rearObstacleDistance = *min_element(rearObstacles.begin(),rearObstacles.end());
        }
        
    }

    /* Update frontFixedObstacle from fixed_obstacles feedback [callback function]  :
    *
    * This function is called when a message is published on the "/front_obstacles" topic
    * 
    */
    void fixedObstaclesCallback(const interfaces::msg::FixedObstacles & fixedObstacleMsg){

        if (fixedObstacleMsg.fixed_obstacles[0] || fixedObstacleMsg.fixed_obstacles[1] || fixedObstacleMsg.fixed_obstacles[2] )
            frontFixedObstacle = true;
    }

    /* Update avoidanceChoice from avoidance feedback [callback function]  :
    *
    * This function is called when a message is published on the "/avoidance_parameters" topic
    * 
    */
    void avoidanceParametersCallback(const interfaces::msg::AvoidanceParameters & avoidanceParamMsg){

        if (avoidanceParamMsg.big && avoidanceParamMsg.left)
            avoidanceChoice = "large left";
        else if (avoidanceParamMsg.big && !avoidanceParamMsg.left)
            avoidanceChoice = "large right";
        else if (!avoidanceParamMsg.big && avoidanceParamMsg.left)
            avoidanceChoice = "short left";
        else if (!avoidanceParamMsg.big && !avoidanceParamMsg.left)
            avoidanceChoice = "short right";

    }

    /* Compute the steering angle for the final reverse  :
    *
    * The steering angle depends on the ultrasonic values
    * 
    */
    float computeTargetAngle(int usRLeft, int usRRight){

        float errorUS = usRLeft - usRRight ;

        if (abs(errorUS)< 5){
            return 0.0 ;
        }
        else {
            if (errorUS > 0){
                return 1.0 ;
            }
            else {
                return -1.0 ;
            }
        }
    }

    /**
     * @brief Analyse the obstacles 
     * @param frontObstacleDistance
     * @param rearObstacleDistance
     * @return obstacleDetected 
     */
    bool obstacleAnalysis(){

        if (targetVelocity == 0.0)
            obstacleDetected = false;

        else{

            if (currentVelocity > 0 && (frontObstacleDistance >=0) && (frontObstacleDistance <= frontSecurityDistance)){
                obstacleDetected = true;

            } else if (currentVelocity < 0 && (rearObstacleDistance >=0) && (rearObstacleDistance <= backSecurityDistance)){
                obstacleDetected = true;
            }
            

            if(targetVelocity > 0 && ((frontObstacleDistance == -1) || (frontObstacleDistance > frontSecurityDistance))){
                obstacleDetected = false;

            }else if (targetVelocity < 0 && ((rearObstacleDistance == -1) || (rearObstacleDistance > backSecurityDistance))){
                obstacleDetected = false;
            }


        }

        return obstacleDetected;

    }

    /* Return true if "value ∈ [target - tolerance ; target + tolerance]"
    */
    bool inTolerance(float target, float value, float tolerance){

        if (abs(target - value) <= tolerance)
            return true;
        else 
            return false;
    }

    
    /* Publish the velocity on the /consign_speed topic
    * If the velocity is the same as the current one : the velocity is not published
    */
    void sendVel(float velocity){

        if (currentVelocity != velocity){

            auto cmdVelMsg = interfaces::msg::CmdVel();
            cmdVelMsg.velocity = velocity;

            //Send velocity command to car_control_node
            publisher_cmd_vel_->publish(cmdVelMsg);

            currentVelocity = velocity;
            RCLCPP_INFO(this->get_logger(), "VELOCITY : %.2f m/s",currentVelocity);

        } 
        
    }

    /* Publish the steering angle on the /consign_steer topic
    * If the angle is the same as the current one : the angle is not published
    * If steerStop = true, the steering motor is stopped
    */
    void sendSteer(float angle, bool steerStop){

        if ((angle != currentSteer) || (currentSteerStop != steerStop)){

            if (angle > 1.0)
                angle = 1.0;
            else if (angle < -1.0)
                angle = -1.0;

            //Send steering command to car_control_node
            auto cmdSteerMsg = interfaces::msg::CmdSteer();

            cmdSteerMsg.angle = angle;
            cmdSteerMsg.stop = steerStop;

            publisher_cmd_steer_->publish(cmdSteerMsg);

            currentSteer = angle;
            currentSteerStop = steerStop;

            if (steerStop)
                RCLCPP_INFO(this->get_logger(), "STEERING : STOP ");
            else
                RCLCPP_INFO(this->get_logger(), "STEERING : %.2f ",currentSteer);
        }
        
    }
    

    /* Publish the hook unlocking order on the /hook topic
    *
    */
    void unlockHook(){
        auto hookMsg = interfaces::msg::Hook();

        hookMsg.type = "lock";
        hookMsg.status = false;
        publisher_cmd_hook_->publish(hookMsg);

        RCLCPP_INFO(this->get_logger(), "Hook unlocking");
        sleep(LOCK_WAITING_TIME);

        hookLocked = false;
        
    }


    /* Publish the hook locking order on the /hook topic
    *
    */
    void lockHook(){
        auto hookMsg = interfaces::msg::Hook();

        hookMsg.type = "lock";
        hookMsg.status = true;
        publisher_cmd_hook_->publish(hookMsg);

        RCLCPP_INFO(this->get_logger(), "Hook locking");
        sleep(LOCK_WAITING_TIME);

        hookLocked = true;
        
    }

    /* Change the securityDistance
    *
    */
    void setSecurityDistance(int frontDistance, int backDistance){

        if (frontDistance != frontSecurityDistance){
            frontSecurityDistance = frontDistance;
            RCLCPP_INFO(this->get_logger(), "Front Security Distance : %d cm", frontSecurityDistance);
        }

        if (backDistance != backSecurityDistance){
            backSecurityDistance = backDistance;
            RCLCPP_INFO(this->get_logger(), "Back Security Distance : %d cm", backSecurityDistance);
        }

    }

    /* Set the avoidance trajectory
    *
    */
    void setAvoidanceTraj (struct VAD_POINT * traj){

        for (int i = 0; i < NB_AVOIDANCE_POINTS ;i++){
            avoidanceTraj[i] = traj[i] ;
        }

    }
    
    void setInitialStates(){

        INITIAL_STATE_MACHINE = true;
        INITIAL_AUTONOMOUS = true;
        INITIAL_MOVE = true;

        printState = true;
    }

    /* Change the current state, called when mode = 4 (SELECT mode). See joystickOrderCallback()
    *
    */
    void changeState(int state){
        switch (state) {
            case 0:
                setInitialStates();
                RCLCPP_WARN(this->get_logger(), "Return to initial state");
                break;    
            case 1:
                analyse = true;
                RCLCPP_WARN(this->get_logger(), "'Analyse' selected");
                break;
            case 2:
                noUturn = true;
                RCLCPP_WARN(this->get_logger(), "'No U-Turn' selected");
                break;
            case 3:
                uTurn = true;
                RCLCPP_WARN(this->get_logger(), "'U-Turn' selected");
                break;
            case 4:
                reverse = true;
                RCLCPP_WARN(this->get_logger(), "'Reverse' selected");
                break;
            case 5:
                RCLCPP_WARN(this->get_logger(), "'Tow' selected");
                tow = true;
                break;
        }
    }

    // Reset all the state machine parameters
    void reset(){

        RCLCPP_WARN(this->get_logger(),"!! RESET !!");

        analyse = false;
        noUturn = false;
        uTurn = false;
        reverse = false;
        tow = false;
        move = false;
        hooking = false;
        emergency = false;
        manual = false;
        autonomous = false;
        idle = false;
        
        hookDetected = false;
        hookDistance = 0.0;
        distanceTravelled = 0.0;
        distanceTravelledAvoidance = 0.0;
        currentPoint = 0;
        autoFailed = false;
        orientationOK = false;
        orientationReceived = false;
        alignmentEnd = false;
        hookFdc = false;
        hookLocked = true;
        hookEnd = false;
        obstacleDetected = false;
        frontFixedObstacle = false;
        towingEnd = false;

        safeMode = true;

        start = false;

    }


    /* State machine of the car (see doc)
    *  Decision control (transitions, states)
    */
    void motionPlanning(){

        if (obstaclesReceived <= 0){
            
            sendVel(0.0);
            sendSteer(0.0,true);
            if (obstaclesReceived == 0){
                obstaclesReceived = -1;    //To print the message once
                RCLCPP_INFO(this->get_logger(), "Waiting for obstacle analysis");
            }
            
            return;
        }

        obstacleAnalysis();

        // ---- Transitions ----

        if (idle && start && manualMode){
            idle = false;
            manual = true;

            printState = true;
        }

        if (manual && !start){
            manual = false;
            idle = true;

            printState = true;
        }

        if (idle && start && !manualMode){
            idle = false;
            autonomous = true;

            printState = true;
        }

        if (autonomous && (!start)){
            autonomous = false;
            idle = true;

            printState = true;

        } else if (autonomous && towingEnd){
            autonomous = false;
            idle = true;
            reset();
            setInitialStates();

            printState = true;
            RCLCPP_WARN(this->get_logger(), "TOWING END");

        } else if (autonomous && autoFailed){
            autonomous = false;
            idle = true;
            reset();
            setInitialStates();

            printState = true;
            RCLCPP_ERROR(this->get_logger(), "AUTONOMOUS FAILED");
        }

        if (autonomous && manualMode && start){
            autonomous = false;
            manual = true;

            printState = true;
        }

        if (manual && !manualMode && start){
            manual = false;
            autonomous = true;

            printState = true;
        }
        
        if (move && safeMode && obstacleDetected){
            move = false;
            emergency = true;

            frontFixedObstacle = false;
            printState = true;

        } else if (emergency && !obstacleDetected && avoidanceInProgress){
            emergency = false;
            avoidance = true;

            printState = true;

        } else if (emergency && (!obstacleDetected || !safeMode)){
            emergency = false;
            move = true;

            printState = true;

        } else if (emergency && frontFixedObstacle && frontObstacleDistance > MIN_DISTANCE_AVOIDANCE && tow && !avoidanceInProgress){
            emergency = false;
            avoidance = true;

            if (avoidanceChoice == "large left"){
                RCLCPP_INFO(this->get_logger(), "Large Left avoidance");
                setAvoidanceTraj(largeLeftTraj);

            }else if (avoidanceChoice == "short left"){
                RCLCPP_INFO(this->get_logger(), "Short Left avoidance");
                setAvoidanceTraj(shortLeftTraj);

            }else if (avoidanceChoice == "large right"){
                RCLCPP_INFO(this->get_logger(), "Large Right avoidance");
                setAvoidanceTraj(largeRightTraj);

            }else if (avoidanceChoice == "short right"){
                RCLCPP_INFO(this->get_logger(), "Short Right avoidance");
                setAvoidanceTraj(shortRightTraj);
            }

            distanceTravelledAvoidance = 0.0;
            printState = true;
        }

        if (avoidance && avoidanceEnd){
            move = true;
            avoidance = false;
            avoidanceEnd = false;
            avoidanceInProgress = false;

            printState = true;

        } else if (avoidance && safeMode && obstacleDetected && (frontSecurityDistance == AVOIDANCE_DISTANCE)){
            avoidance = false;
            emergency = true;

            printState = true;
        }

        if (move && hookFdc && !hookLocked){
            move = false;
            hooking = true;

            printState = true;
        }

        if (hooking && hookLocked){
            hooking = false;
            move = true;

            printState = true;
        }

        if (analyse && !orientationOK && orientationReceived){
            analyse = false;
            uTurn = true;

            printState = true;

        } else if (analyse && orientationOK && orientationReceived){
            analyse = false;
            noUturn = true;

            printState = true;
        }

        if (noUturn && alignmentEnd){
            noUturn = false;
            reverse = true;

            printState = true;

        } else if (uTurn && alignmentEnd){
            uTurn = false;
            reverse = true;

            printState = true;
        }

        if (reverse && hookLocked && hookEnd){
            reverse = false;
            tow = true;
            distanceTravelled = 0.0;

            printState = true;
        }


        // ------ States ------
        
        if (idle){

            if (printState)
                RCLCPP_WARN(this->get_logger(), "IDLE");

            targetVelocity = 0.0;
            sendVel(targetVelocity);

            sendSteer(currentSteer,true);       
        }

        else if (manual){

            if (printState)
                RCLCPP_WARN(this->get_logger(), "MANUAL");

        }else if (autonomous){

            if (printState)
                RCLCPP_WARN(this->get_logger(), "AUTONOMOUS");

            if (emergency){ 

                if (printState)
                    RCLCPP_WARN(this->get_logger(), "-> EMERGENCY");

                sendVel(0.0);
                sendSteer(currentSteer,true);


            } else if (avoidance){

                avoidanceInProgress = true;

                if (printState)
                    RCLCPP_WARN(this->get_logger(), "-> AVOIDANCE");
            
                safeMode = true;
                setSecurityDistance(AVOIDANCE_DISTANCE,LLS_DISTANCE);

                if (distanceTravelledAvoidance >= avoidanceTraj[currentPoint].distance){

                    if (currentPoint == (NB_AVOIDANCE_POINTS - 1)){    //Last point

                        targetSteer = avoidanceTraj[currentPoint].angle;
                        sendSteer(targetSteer, false);   //Turn steering

                        if (!inTolerance(targetSteer, feedbackSteer, TOLERANCE_STEER)){
                            targetVelocity = 0.0;   //Stop until targetSteer is reached
                            sendVel(targetVelocity);

                        } else{
                            distanceTravelledAvoidance = 0;
                            currentPoint = 0;
                            avoidanceEnd = true;
                            avoidanceInProgress = false;
                            safeMode = true;
                            return ;
                        }

                        
                    }
                    else{
                        distanceTravelledAvoidance = 0;

                        targetVelocity = 0.0; //Stop
                        sendVel(targetVelocity);

                        currentPoint++; 
                        
                    }
                }

                targetSteer = avoidanceTraj[currentPoint].angle;
                sendSteer(targetSteer, false);   //Turn steering

                if (!inTolerance(targetSteer, feedbackSteer, TOLERANCE_STEER)){
                    targetVelocity = 0.0;   //Stop until targetSteer is reached
                    sendVel(targetVelocity);
                
                } else{
                    targetVelocity = avoidanceTraj[currentPoint].velocity; 
                    sendVel(targetVelocity);    //Send velocity
                }


            } else if (move){

                if (printState)
                    RCLCPP_WARN(this->get_logger(), "-> MOVE");

                if (analyse){

                    if (printState)
                        RCLCPP_WARN(this->get_logger(), "--> ANALYSE");
                    

                }else if (noUturn){

                    if (printState)
                        RCLCPP_WARN(this->get_logger(), "--> NO U-TURN");
                
                    safeMode = true;
                    setSecurityDistance(NS_DISTANCE,NS_DISTANCE);

                    if (distanceTravelled >= nutTraj[currentPoint].distance){

                        if (currentPoint == (NB_NUT_POINTS - 1)){    //Last point

                            targetSteer = nutTraj[currentPoint].angle;
                            sendSteer(targetSteer, false);   //Turn steering

                            if (!inTolerance(targetSteer, feedbackSteer, TOLERANCE_STEER)){
                                targetVelocity = 0.0;   //Stop until targetSteer is reached
                                sendVel(targetVelocity);

                            } else{
                                distanceTravelled = 0;
                                currentPoint = 0;
                                alignmentEnd = true;
                                safeMode = true;
                                return ;
                            }

                            
                        }
                        else{
                            distanceTravelled = 0;

                            targetVelocity = 0.0; //Stop
                            sendVel(targetVelocity);

                            currentPoint++; 
                            
                        }
                    }

                    targetSteer = nutTraj[currentPoint].angle;
                    sendSteer(targetSteer, false);   //Turn steering

                    if (!inTolerance(targetSteer, feedbackSteer, TOLERANCE_STEER)){
                        targetVelocity = 0.0;   //Stop until targetSteer is reached
                        sendVel(targetVelocity);
                    
                    } else{
                        targetVelocity = nutTraj[currentPoint].velocity; 
                        sendVel(targetVelocity);    //Send velocity
                    }


                }else if (uTurn){

                    

                    if (printState)
                        RCLCPP_WARN(this->get_logger(), "--> U-TURN");
                
                    safeMode = true;
                    setSecurityDistance(NS_DISTANCE,NS_DISTANCE);

                    if (distanceTravelled >= utTraj[currentPoint].distance){

                        if (currentPoint == (NB_UT_POINTS - 1)){    //Last point

                            targetSteer = utTraj[currentPoint].angle;
                            sendSteer(targetSteer, false);   //Turn steering

                            if (!inTolerance(targetSteer, feedbackSteer, TOLERANCE_STEER)){
                                targetVelocity = 0.0;   //Stop until targetSteer is reached
                                sendVel(targetVelocity);

                            } else{
                                distanceTravelled = 0;
                                currentPoint = 0;
                                alignmentEnd = true;
                                safeMode = true;
                                return ;
                            }

                            
                        }
                        else{
                            distanceTravelled = 0;

                            targetVelocity = 0.0; //Stop
                            sendVel(targetVelocity);

                            currentPoint++; 
                            
                        }
                    }

                    targetSteer = utTraj[currentPoint].angle;
                    sendSteer(targetSteer, false);   //Turn steering

                    if (!inTolerance(targetSteer, feedbackSteer, TOLERANCE_STEER)){
                        targetVelocity = 0.0;   //Stop until targetSteer is reached
                        sendVel(targetVelocity);
                    
                    } else{
                        targetVelocity = utTraj[currentPoint].velocity; 
                        sendVel(targetVelocity);    //Send velocity
                    }
                    
                }else if (reverse){

                    if (printState)
                        RCLCPP_WARN(this->get_logger(), "--> REVERSE");

                    if (!hookDetected || hookDistance > 50.0){
                        targetVelocity = REVERSE_VELOCITY;
                        sendVel(targetVelocity);
                        sendSteer(0.0,false);

                    }else if (hookDistance <= 50.0){

                        setSecurityDistance(NS_DISTANCE,LLS_DISTANCE);

                        targetSteer = computeTargetAngle(usRearLeft,usRearRight);

                        if (hookLocked){
                            targetVelocity = 0.0;
                            sendVel(targetVelocity);
                            sendSteer(targetSteer,false);
                            sleep(1.0);
                            unlockHook();
                        }

                        targetVelocity = FINAL_REVERSE_VELOCITY;
                        sendVel(targetVelocity);
                        
                        sendSteer(targetSteer,false); //TO DO : Adapt steering according to the QR code position

                    }else {
                        autoFailed = true;
                    }

                }else if (tow){

                    if (printState)
                        RCLCPP_WARN(this->get_logger(), "--> TOW");

                    setSecurityDistance(TOW_DISTANCE,LLS_DISTANCE);

                    if (distanceTravelled < TOWING_DISTANCE){
                        targetVelocity = TOWING_VELOCITY;
                        sendVel(targetVelocity);
                        sendSteer(0.0,false);

                    }else{
                        targetVelocity = 0.0;
                        sendVel(targetVelocity);
                        sendSteer(0.0,false);
                        towingEnd = true;
                    }
                }

            }else if (hooking){

                if (printState)
                    RCLCPP_WARN(this->get_logger(), "-> HOOKING");

                targetVelocity = 0.0;
                sendVel(targetVelocity);
                sendSteer(0.0,false);
                sleep(1.0);
                lockHook();
                setSecurityDistance(NS_DISTANCE,LLS_DISTANCE);
                hookEnd = true;

            }

        }

        printState = false;

    }

    // ---- Private variables ----
    int mode; 

    bool hookDetected = false;

    //States
    bool analyse, noUturn, uTurn, reverse, tow, move, hooking, emergency, idle, manual, autonomous, avoidance = false;
    bool printState = true; // If true, the current state is displayed in the terminal
    int moveState = 0;

    //Transitions
    bool start, manualMode, autoFailed, orientationOK, orientationReceived, alignmentEnd, hookFdc, hookEnd, obstacleDetected, safeMode, towingEnd, avoidanceEnd, avoidanceInProgress = false;
    bool hookLocked = true;
    //Trajectories
    float currentVelocity = 0.0;
    float targetVelocity;
    float targetSteer;
    float currentSteer = 0.0;
    float feedbackSteer = 0.0;
    float hookPos_x = -1.0;

    int currentPoint = 0;

    VAD_POINT nutTraj[NB_NUT_POINTS];
    VAD_POINT utTraj[NB_UT_POINTS];


    VAD_POINT shortLeftTraj[NB_AVOIDANCE_POINTS];
    VAD_POINT largeLeftTraj[NB_AVOIDANCE_POINTS];
    VAD_POINT shortRightTraj[NB_AVOIDANCE_POINTS];
    VAD_POINT largeRightTraj[NB_AVOIDANCE_POINTS];

    VAD_POINT avoidanceTraj[NB_AVOIDANCE_POINTS];

    string avoidanceChoice = "short left";

    // Ultrasonic sensors
    int usRearLeft ;
    int usRearRight ;


    //Security
    int frontSecurityDistance = 0;
    int backSecurityDistance = 0;
    int obstaclesReceived = 0;  //0 and -1 => not received ; 1 => received
    int frontObstacleDistance, rearObstacleDistance = 0;   //minimum obstacle distance (-1 if no obstacle)
    bool frontFixedObstacle = false;
    bool currentSteerStop = false; //true => the steering movement is stopped

    //Distance
    float distanceTravelled = 0.0; //Distance measurement [cm]
    float distanceTravelledAvoidance = 0.0;
    float lastDistance = 0.0; //Last total distance [cm]
    float printDistance = 0.0;
    float hookDistance = 0.0;


    //Publishers
    rclcpp::Publisher<interfaces::msg::CmdVel>::SharedPtr publisher_cmd_vel_;
    rclcpp::Publisher<interfaces::msg::CmdSteer>::SharedPtr publisher_cmd_steer_;
    rclcpp::Publisher<interfaces::msg::Hook>::SharedPtr publisher_cmd_hook_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::Hook>::SharedPtr subscription_hook_;
    rclcpp::Subscription<interfaces::msg::Obstacles>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<interfaces::msg::FixedObstacles>::SharedPtr subscription_fixed_obstacles_;
    rclcpp::Subscription<interfaces::msg::AvoidanceParameters>::SharedPtr subscription_avoidance_parameters_;
    rclcpp::Subscription<interfaces::msg::Distance>::SharedPtr subscription_distance_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_ultrasonic_;
    rclcpp::Subscription<interfaces::msg::Orientation>::SharedPtr subscription_orientation_;

    //Timers
    rclcpp::TimerBase::SharedPtr timer_motion_planning_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motion_planning>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}