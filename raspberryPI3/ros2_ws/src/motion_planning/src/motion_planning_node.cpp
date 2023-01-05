#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "../include/motion_planning/motion_planning_node.hpp"

#include "interfaces/msg/cmd_vel.hpp"
#include "interfaces/msg/cmd_steer.hpp"
#include "interfaces/msg/hook.hpp"
#include "interfaces/msg/obstacles.hpp"
#include "interfaces/msg/distance.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/ultrasonic.hpp"

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
        nutTraj[0].distance = 150.0;

        nutTraj[1].velocity = 0.8;
        nutTraj[1].angle = -1.0;
        nutTraj[1].distance = 175.0;

        nutTraj[2].velocity = 0.0;
        nutTraj[2].angle = 0.0;
        nutTraj[2].distance = 0.0;

        //U-Turn trajectory

        utTraj[0].velocity = -0.8;
        utTraj[0].angle = -1.0;
        utTraj[0].distance = 320.0;

        utTraj[1].velocity = 0.8;
        utTraj[1].angle = 1.0;
        utTraj[1].distance = 191.0;

        utTraj[2].velocity = -0.8;
        utTraj[2].angle = -1.0;
        utTraj[2].distance = 164.0;

        utTraj[3].velocity = 0.0;
        utTraj[3].angle = 0.0;
        utTraj[3].distance = 0.0;

        //Avoidance

        avoidanceTraj[0].velocity = 0.8;
        avoidanceTraj[0].angle = -1.0;
        avoidanceTraj[0].distance = 70.0;

        avoidanceTraj[1].velocity = 0.8;
        avoidanceTraj[1].angle = 0.0;
        avoidanceTraj[1].distance = 72.0;

        avoidanceTraj[2].velocity = 0.8;
        avoidanceTraj[2].angle = +1.0;
        avoidanceTraj[2].distance = 221.0;

        avoidanceTraj[3].velocity = 0.8;
        avoidanceTraj[3].angle = -1.0;
        avoidanceTraj[3].distance = 100.0;

        avoidanceTraj[4].velocity = 0.0;
        avoidanceTraj[4].angle = 0.0;
        avoidanceTraj[4].distance = 0.0;


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

        subscription_distance_ = this->create_subscription<interfaces::msg::Distance>(
        "distance", 10, std::bind(&motion_planning::distanceCallback, this, _1));

        subscription_ultrasonic_ = this->create_subscription<interfaces::msg::Ultrasonic>(
        "us_data", 10, std::bind(&motion_planning::ultrasonicCallback, this, _1));

        timer_motion_planning_ = this->create_wall_timer(PERIOD_UPDATE_MOTION, std::bind(&motion_planning::motionPlanning, this));
 
        sleep(2);   //Waiting for car_control to start
        RCLCPP_INFO(this->get_logger(), "motion_planning_node READY");

        sendVel(INITIAL_VELOCITY);
        sendSteer(INITIAL_STEER,true);

        setInitialStates();
        safeMode = true;
        setSecurityDistance(NS_DISTANCE);

    }

    
private:


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
    }

    /* Update currentAngle from motors feedback [callback function]  :
    *
    * This function is called when a message is published on the "/motors_feedback" topic
    * 
    */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
        feedbackSteer = motorsFeedback.steering_angle;
    }


    /* Update usRearLeft and usRearRight from ultrasonic feedback [callback function]  :
    *
    * This function is called when a message is published on the "/ultrasonic" topic
    * 
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
                setSecurityDistance(LLS_DISTANCE);
            }
            hookPos_x = hookMsg.x;


            hookDistance = 40.0;

        } else if (hookMsg.type == "fdc")
            hookFdc = hookMsg.status;
        
    }


    /* Update distance from distance topic [callback function]  :
    *
    * This function is called when a message is published on the "/distance" topic
    * 
    */
    void distanceCallback(const interfaces::msg::Distance & distanceMsg){
 
        distanceTravelled += distanceMsg.total - lastDistance;
        lastDistance = distanceMsg.total;

        //RCLCPP_INFO(this->get_logger(), "Distance : %f cm",distanceTravelled);

        // printDistance = distanceTravelled;
        // if (printDistance >= 20.0){
        //     printDistance = 0.0;
        //     RCLCPP_INFO(this->get_logger(), "Distance : %f cm",distanceTravelled);
        // }

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

            if (currentVelocity > 0 && (frontObstacleDistance >=0) && (frontObstacleDistance <= securityDistance)){
                obstacleDetected = true;

            } else if (currentVelocity < 0 && (rearObstacleDistance >=0) && (rearObstacleDistance <= securityDistance)){
                obstacleDetected = true;
            }
            

            if(targetVelocity > 0 && ((frontObstacleDistance == -1) || (frontObstacleDistance > securityDistance))){
                obstacleDetected = false;

            }else if (targetVelocity < 0 && ((rearObstacleDistance == -1) || (rearObstacleDistance > securityDistance))){
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
    

    /* Publish the hook locking order on the /hook topic
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


    /* Publish the hook unlocking order on the /hook topic
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

    void setSecurityDistance(int distance){

        if (distance != securityDistance){
            securityDistance = distance;
            RCLCPP_INFO(this->get_logger(), "Security Distance : %d cm", securityDistance);
        }

    }
    
    void setInitialStates(){

        INITIAL_STATE_MACHINE = true;
        INITIAL_AUTONOMOUS = true;
        INITIAL_MOVE = true;

        printState = true;
    }

    // Reset the state machine to the initial state
    void reset(){
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
        autoFailed = false;
        orientationOK = false;
        trajectoryOK = false;
        alignmentEnd = false;
        hookFdc = false;
        hookLocked = false;
        hookEnd = false;
        obstacleDetected = false;
        towingEnd = false;

        safeMode = true;

        setInitialStates();
    }



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

            printState = true;
            RCLCPP_WARN(this->get_logger(), "TOWING END");

        } else if (autonomous && autoFailed){
            autonomous = false;
            idle = true;
            reset();

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

            printState = true;

        } else if (emergency && (!obstacleDetected || !safeMode)){
            emergency = false;
            move = true;

            printState = true;

        } else if (emergency && frontFixedObstacle && tow){
            emergency = false;
            avoidance = true;

            printState = true;
        }

        if (avoidance && avoidanceEnd){
            move = true;
            avoidance = false;

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

        if (analyse && trajectoryOK && !orientationOK){
            analyse = false;
            uTurn = true;

            printState = true;

        } else if (analyse && trajectoryOK && orientationOK){
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


        //States
        
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

                if (printState)
                    RCLCPP_WARN(this->get_logger(), "-> AVOIDANCE");
            
                safeMode = true;
                setSecurityDistance(LLS_DISTANCE);

                if (distanceTravelled >= avoidanceTraj[currentPoint].distance){

                    if (currentPoint == (NB_AVOIDANCE_POINTS - 1)){    //Last point

                        targetSteer = avoidanceTraj[currentPoint].angle;
                        sendSteer(targetSteer, false);   //Turn steering

                        if (!inTolerance(targetSteer, feedbackSteer, TOLERANCE_STEER)){
                            targetVelocity = 0.0;   //Stop until targetSteer is reached
                            sendVel(targetVelocity);

                        } else{
                            distanceTravelled = 0;
                            currentPoint = 0;
                            avoidanceEnd = true;
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
                    // ...

                }else if (noUturn){

                    if (printState)
                        RCLCPP_WARN(this->get_logger(), "--> NO U-TURN");

                    if (currentPoint == 0){
                        sendSteer(nutTraj[currentPoint].angle, false);
                        targetVelocity = nutTraj[currentPoint].velocity;
                        sendVel(targetVelocity);
                        
                    }

                    if (distanceTravelled >= nutTraj[currentPoint].distance){

                        if (currentPoint == (NB_UT_POINTS - 1)){    //Last point
                            distanceTravelled = 0;
                            currentPoint = 0;
                            alignmentEnd = true;
                            return ;
                        }
                        else{
                            distanceTravelled = 0;

                            currentPoint++; 

                            sendSteer(utTraj[currentPoint].angle, false);
                            targetVelocity = utTraj[currentPoint].velocity;
                            sendVel(targetVelocity);
                        }
                    }

                }else if (uTurn){

                    

                    if (printState)
                        RCLCPP_WARN(this->get_logger(), "--> U-TURN");
                
                    safeMode = true;
                    setSecurityDistance(NS_DISTANCE);

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

                        setSecurityDistance(LLS_DISTANCE);

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
                setSecurityDistance(NS_DISTANCE);
                hookEnd = true;

            }

        }

        printState = false;

    }

    // ---- Private variables ----

    bool hookDetected = false;

    //States
    bool analyse, noUturn, uTurn, reverse, tow, move, hooking, emergency, idle, manual, autonomous, avoidance = false;
    bool printState = true; // If true, the current state is displayed in the terminal

    //Transitions
    bool start, manualMode, autoFailed, orientationOK, trajectoryOK, alignmentEnd, hookFdc, hookEnd, obstacleDetected, safeMode, towingEnd, avoidanceEnd = false;
    bool hookLocked = true;
    //Trajectories
    float currentVelocity = 0.0;
    float targetVelocity;
    float targetSteer;
    float currentSteer = 0.0;
    float feedbackSteer = 0.0;
    float hookPos_x = -1.0;

    struct VAD_POINT {
        float velocity;
        float angle;
        float distance;
    };

    int currentPoint = 0;

    VAD_POINT nutTraj[NB_NUT_POINTS];
    VAD_POINT utTraj[NB_UT_POINTS];
    VAD_POINT avoidanceTraj[NB_AVOIDANCE_POINTS];

    // Ultrasonic sensors
    int usRearLeft ;
    int usRearRight ;


    //Security
    int securityDistance = 0;
    int obstaclesReceived = 0;  //0 and -1 => not received ; 1 => received
    int frontObstacleDistance, rearObstacleDistance = 0;   //minimum obstacle distance (-1 if no obstacle)
    bool frontFixedObstacle = false;
    bool currentSteerStop = false; //true => the steering movement is stopped

    //Distance
    float distanceTravelled = 0.0; //Distance measurement [cm]
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
    rclcpp::Subscription<interfaces::msg::Distance>::SharedPtr subscription_distance_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_ultrasonic_;

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