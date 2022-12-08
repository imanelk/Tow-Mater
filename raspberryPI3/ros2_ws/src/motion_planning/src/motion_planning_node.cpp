#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "../include/motion_planning/motion_planning_node.hpp"

#include "interfaces/msg/cmd_vel.hpp"
#include "interfaces/msg/cmd_steer.hpp"
#include "interfaces/msg/hook.hpp"
#include "interfaces/msg/obstacles.hpp"
#include "interfaces/msg/distance.hpp"

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
        utTraj[1].distance = 186.0;

        utTraj[2].velocity = -0.8;
        utTraj[2].angle = -1.0;
        utTraj[2].distance = 154.0;

        utTraj[3].velocity = 0.0;
        utTraj[3].angle = 0.0;
        utTraj[3].distance = 0.0;

        publisher_cmd_vel_= this->create_publisher<interfaces::msg::CmdVel>("consign_speed", 10);
        publisher_cmd_steer_= this->create_publisher<interfaces::msg::CmdSteer>("consign_steer", 10);
        publisher_cmd_hook_= this->create_publisher<interfaces::msg::Hook>("hook", 10);

       

        subscription_hook_ = this->create_subscription<interfaces::msg::Hook>(
        "hook", 10, std::bind(&motion_planning::hookCallback, this, _1));

        subscription_obstacles_ = this->create_subscription<interfaces::msg::Obstacles>(
        "obstacle", 10, std::bind(&motion_planning::obstaclesCallback, this, _1));

        subscription_distance_ = this->create_subscription<interfaces::msg::Distance>(
        "distance", 10, std::bind(&motion_planning::distanceCallback, this, _1));



        timer_motion_planning_ = this->create_wall_timer(PERIOD_UPDATE_MOTION, std::bind(&motion_planning::motionPlanning, this));
 
        sleep(2);   //Waiting for car_control to start
        RCLCPP_INFO(this->get_logger(), "motion_planning_node READY");

        sendVel(INITIAL_VELOCITY);
        sendSteer(INITIAL_STEER,false);

        uTurn = true;
        RCLCPP_WARN(this->get_logger(), "U-TURN");


    }

    
private:

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
            }
            hookPos_x = hookMsg.x;


            hookDistance = 40.0;

        } else if (hookMsg.type == "fdc")
            hookFdc = hookMsg.status;
        
    }

    /* Update areaStatus from obstacles feedback [callback function]  :
    *
    * This function is called when a message is published on the "/obstacles" topic
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

    /* Update distance from distance topic [callback function]  :
    *
    * This function is called when a message is published on the "/distance" topic
    * 
    */
    void distanceCallback(const interfaces::msg::Distance & distanceMsg){
 
        distanceTravelled += distanceMsg.last;
        printDistance += distanceMsg.last;
        if (printDistance >= 20.0){
            printDistance = 0.0;
            RCLCPP_INFO(this->get_logger(), "Distance : %f cm",distanceTravelled);
        }       

    }


    
    /* Publish the velocity on the /consign_speed topic
    * If the velocity is not safe or if the velocity is the same as the current one : the velocity is not published
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


    void motionPlanning(){   //Periodic

        if (obstaclesReceived <= 0){
            
            sendVel(0.0);
            sendSteer(0.0,false);
            if (obstaclesReceived == 0){
                obstaclesReceived = -1;    //To print the message once
                RCLCPP_INFO(this->get_logger(), "Waiting for obstacle analysis");
            }
            
            return;
        }

        //Transitions
        if (!emergency && lowLevelSecurity && (currentVelocity > 0) && (frontObstacleDistance >=0) && (frontObstacleDistance <= LLS_DISTANCE)){
            emergency = true;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY");

        } else if (!emergency && lowLevelSecurity && (currentVelocity < 0) && (rearObstacleDistance >=0) && (rearObstacleDistance <= LLS_DISTANCE)){
            emergency = true;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY");
        }
        
        else if (!emergency && !lowLevelSecurity && (currentVelocity > 0) && (frontObstacleDistance >= 0) && (frontObstacleDistance <= NS_DISTANCE)){
            emergency = true;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY");
        }

        else if (!emergency && !lowLevelSecurity && (currentVelocity < 0) && (rearObstacleDistance >= 0) && (rearObstacleDistance <= NS_DISTANCE)){
            emergency = true;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY");
        }
        
        
        if(emergency && lowLevelSecurity && (targetVelocity > 0) && ((frontObstacleDistance==-1) || (frontObstacleDistance > LLS_DISTANCE))){
            emergency = false;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY [exit]");

        }else if (emergency && lowLevelSecurity && (targetVelocity < 0) && ((rearObstacleDistance==-1) || (rearObstacleDistance > LLS_DISTANCE))){
            emergency = false;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY [exit]");
        }

        else if (emergency && !lowLevelSecurity && (targetVelocity > 0) && ( (frontObstacleDistance == -1) || (frontObstacleDistance > NS_DISTANCE) )){
            emergency = false;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY [exit]");
        }

        else if (emergency && !lowLevelSecurity && (targetVelocity < 0) && ( (rearObstacleDistance == -1) || (rearObstacleDistance > NS_DISTANCE) )){
            emergency = false;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY [exit]");
        }

        else if (emergency && reverse && !lowLevelSecurity && hookDetected){
            emergency = false;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY [exit]");
        }

        if (noUturn && alignmentEnd){
            noUturn = false;
            reverse = true;
            RCLCPP_WARN(this->get_logger(), "REVERSE");
        }

        if (uTurn && alignmentEnd){
            uTurn = false;
            reverse = true;
            RCLCPP_WARN(this->get_logger(), "REVERSE");
        }

        if (reverse && reverseEnd && hookLocked){
            reverse = false;
            tow = true;
            lowLevelSecurity = false;
            distanceTravelled = 0.0;

            RCLCPP_WARN(this->get_logger(), "TOW");
        }

        if (reverse && reverseError){
            RCLCPP_ERROR(this->get_logger(), "REVERSE [ERROR]");
            reverse = false;
            idle = true;
        }

        if (tow && towingEnd){
            tow = false;
            idle = true;
            RCLCPP_WARN(this->get_logger(), "IDLE");
        }

        //States
        if (manual){
            return;

        } else if (idle){

            targetVelocity = 0.0;
            sendVel(targetVelocity);
            if (hookLocked)
                unlockHook();        
        }

        else if (emergency){ 

            sendVel(0.0);
            sendSteer(currentSteer,true);

        
        }else if (avoidance){
            //...
        
        }else if (noUturn){

            if (currentPoint < NB_NUT_POINTS){
                targetVelocity = nutTraj[currentPoint].velocity;
                sendVel(targetVelocity);
                sendSteer(nutTraj[currentPoint].angle, false);
            }

            else{  //currentPoint == lastPoint + 1 (ie end of the maneuver)
                distanceTravelled = 0;
                currentPoint = 0;
                sleep(3);
                alignmentEnd = true;
                return ;
            }

            if (distanceTravelled >= nutTraj[currentPoint].distance){
                distanceTravelled = 0;
                RCLCPP_INFO(this->get_logger(),"Next Point");

                currentPoint++;          

            }

        }else if (uTurn){
            if (currentPoint < NB_UT_POINTS){
                targetVelocity = utTraj[currentPoint].velocity;
                sendVel(targetVelocity);
                sendSteer(utTraj[currentPoint].angle, false);
            }

            else{  //currentPoint == lastPoint + 1 (ie end of the maneuver)
                distanceTravelled = 0;
                currentPoint = 0;
                sleep(3);
                alignmentEnd = true;
                return ;
            }

            if (distanceTravelled >= utTraj[currentPoint].distance){
                distanceTravelled = 0;
                RCLCPP_INFO(this->get_logger(),"Next Point");

                currentPoint++;

                targetVelocity = 0.0;
                sendVel(targetVelocity);
                sleep(3);        

            }
            
        }else if (reverse){

            reverseError = false;

            if (hookFdc){
                targetVelocity = 0.0;
                sendVel(targetVelocity);
                sendSteer(0.0,false);
                sleep(1.0);
                lockHook();
                reverseEnd = true;

            }else if (!hookDetected || hookDistance > 50.0){
                targetVelocity = REVERSE_VELOCITY;
                sendVel(targetVelocity);
                sendSteer(0.0,false);

            }else if (hookDistance <= 50.0){
                targetSteer = -(hookPos_x/200.0) + 260.0/200;

                if (hookLocked){
                    targetVelocity = 0.0;
                    sendVel(targetVelocity);
                    sendSteer(targetSteer,false);
                    sleep(1.0);
                    unlockHook();
                }

                targetVelocity = FINAL_REVERSE_VELOCITY;
                sendVel(targetVelocity);
                lowLevelSecurity = true;
                
                sendSteer(targetSteer,false); //TO DO : Adapt steering according to the QR code position

            }else {
                reverseError = true;
            }

        }else if (tow){
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


    }

    // ---- Private variables ----
    bool hookDetected, hookFdc = false;
    bool hookLocked = true;

    //States
    bool noUturn, reverse, uTurn, tow, emergency, avoidance, idle, manual = false;

    //Trans
    bool towingEnd, reverseEnd, alignmentEnd = false;
    bool reverseError = false;

    //Trajectories
    float currentVelocity = 0.0;
    float targetVelocity;
    float targetSteer;
    float currentSteer = 0.0;
    float hookPos_x = -1.0;

    struct VAD_POINT {
        float velocity;
        float angle;
        float distance;
    };

    int currentPoint = 0;

    VAD_POINT nutTraj[NB_NUT_POINTS];
    VAD_POINT utTraj[NB_UT_POINTS];

    //Security
    bool lowLevelSecurity = false;
    int obstaclesReceived = 0;  //0 and -1 => not received ; 1 => received
    int frontObstacleDistance, rearObstacleDistance = 0;   //minimum obstacle distance (-1 if no obstacle)
    bool currentSteerStop = false; //true => the steering movement is stopped

    //Distance
    float distanceTravelled = 0.0; //Distance measurement [cm]
    float printDistance = 0.0;
    float hookDistance = 0.0;


    //Time counter
    chrono::milliseconds localMsCounter = 0ms;

    //Publishers
    rclcpp::Publisher<interfaces::msg::CmdVel>::SharedPtr publisher_cmd_vel_;
    rclcpp::Publisher<interfaces::msg::CmdSteer>::SharedPtr publisher_cmd_steer_;
    rclcpp::Publisher<interfaces::msg::Hook>::SharedPtr publisher_cmd_hook_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::Hook>::SharedPtr subscription_hook_;
    rclcpp::Subscription<interfaces::msg::Obstacles>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<interfaces::msg::Distance>::SharedPtr subscription_distance_;

    //Timers
    rclcpp::TimerBase::SharedPtr timer_security_;
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