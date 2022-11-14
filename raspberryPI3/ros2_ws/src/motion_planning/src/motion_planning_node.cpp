#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "../include/motion_planning/motion_planning_node.hpp"

#include "interfaces/msg/cmd_vel.hpp"
#include "interfaces/msg/cmd_steer.hpp"
#include "interfaces/msg/hook.hpp"
#include "interfaces/msg/obstacles.hpp"

using namespace std;
using placeholders::_1;


class motion_planning : public rclcpp::Node {

public:
    motion_planning()
    : Node("motion_planning_node")
    {
        safeMode = true;
        currentVelocity = -1; 
        currentSteer = -1, 
        finalReverse = true;

        publisher_cmd_vel_= this->create_publisher<interfaces::msg::CmdVel>("consign_speed", 10);
        publisher_cmd_steer_= this->create_publisher<interfaces::msg::CmdSteer>("cmd/steer", 10);
        publisher_cmd_hook_= this->create_publisher<interfaces::msg::Hook>("hook", 10);

       

        subscription_hook_ = this->create_subscription<interfaces::msg::Hook>(
        "hook", 10, std::bind(&motion_planning::hookCallback, this, _1));

        subscription_obstacles_ = this->create_subscription<interfaces::msg::Obstacles>(
        "obstacle", 10, std::bind(&motion_planning::obstaclesFeedback, this, _1));


        timer_security_ = this->create_wall_timer(PERIOD_CHECK_SECURITY, std::bind(&motion_planning::checkSecurity, this));

        timer_motion_planning_ = this->create_wall_timer(PERIOD_UPDATE_MOTION, std::bind(&motion_planning::motionPlanning, this));

        sendVel(INITIAL_VELOCITY);
        sendSteer(INITIAL_STEER);

        unlockHook();
 
        RCLCPP_INFO(this->get_logger(), "motion_planning_node READY");

    }

    
private:

    /* Update "hookDetected" from hook status [callback function]  :
    *
    * This function is called when a message is published on the "/hook" topic
    * 
    */
    void hookCallback(const interfaces::msg::Hook & hookMsg) {

        if (hookMsg.type == "detect"){
           hookDetected = hookMsg.status;
        }
        
    }

    /* Update areaStatus from obstacles feedback [callback function]  :
    *
    * This function is called when a message is published on the "/obstacles" topic
    * 
    */
    void obstaclesFeedback(const interfaces::msg::Obstacles & obstaclesMsg){

        if (obstaclesReceived != 1){
            RCLCPP_WARN(this->get_logger(),"Obstacle analysis received [OK]");
            obstaclesReceived = 1;
        }

        for (int i=0 ; i<6 ; i++)
            areaStatus[i] = obstaclesMsg.area[i];
    }


    /* Stops the car if the movement is not safe (obstacle detected) */
    bool checkSecurity(){

        if (!safe(currentVelocity)){
            sendVel(0.0);
            return false;
        } else
            return true;

    }


    /* Checks if the movement is safe (depends on speed and obstacles)
    * returns "true" if safe
    */
    bool safe(float velocity){

        if (velocity == 0 || !safeMode)
            return true;        //not moving => safe

        if (obstaclesReceived !=1) //If the obstacle analysis has not yet been received on the /obstacles topic
            return false;


        //If the car is moving, check the corresponding areas
        if ((velocity > 0) && (areaStatus[0] == -1 && areaStatus[1] == -1 && areaStatus[2] == -1))   //check the front areas
            return true;

        else if ((velocity < 0) && (areaStatus[3] == -1 && areaStatus[4] == -1 && areaStatus[5] == -1))   //check the rear areas
            return true;

        else{
            RCLCPP_DEBUG(this->get_logger(), "Movement not safe");
            return false;
        }
    }

    /* Publish the velocity on the /cmd/vel topic
    * If the velocity is not safe or if the velocity is the same as the current one : the velocity is not published
    */
    void sendVel(float velocity){

        if (safe(velocity) && (currentVelocity != velocity)){

            auto cmdVelMsg = interfaces::msg::CmdVel();
            cmdVelMsg.velocity = velocity;

            //Send velocity command to car_control_node
            publisher_cmd_vel_->publish(cmdVelMsg);

            currentVelocity = velocity;
            RCLCPP_DEBUG(this->get_logger(), "Send velocity : %f m/s",velocity);

        } 
        
    }

    /* Publish the steering angle on the /cmd/steer topic
    * If the angle is the same as the current one : the angle is not published
    */
    void sendSteer(float angle){

        if (angle != currentSteer){

            //Send steering command to car_control_node
            auto cmdSteerMsg = interfaces::msg::CmdSteer();

            cmdSteerMsg.angle = angle;

            publisher_cmd_steer_->publish(cmdSteerMsg);

            currentSteer = angle;
        }
        
    }

    /* Publish the hook locking order on the /hook topic
    *
    */
    void unlockHook(){
        auto hookMsg = interfaces::msg::Hook();

        hookMsg.type = "lock";
        hookMsg.status = true;
        publisher_cmd_hook_->publish(hookMsg);

        RCLCPP_INFO(this->get_logger(), "Hook unlocking");
        sleep(LOCK_WAITING_TIME);

        hookLocked = true;
        
    }


    /* Publish the hook unlocking order on the /hook topic
    *
    */
    void lockHook(){
        auto hookMsg = interfaces::msg::Hook();

        hookMsg.type = "lock";
        hookMsg.status = false;
        publisher_cmd_hook_->publish(hookMsg);

        RCLCPP_INFO(this->get_logger(), "Hook locking");
        sleep(LOCK_WAITING_TIME);

        hookLocked = false;
        
    }

    /*  Manages the movements of the car: Decision-making entity  */
    void motionPlanning(){

        if (obstaclesReceived != 1){  

            if (obstaclesReceived != -1){    //To print the message once
                RCLCPP_WARN(this->get_logger(),"Obstacle analysis not received [waiting]");
                obstaclesReceived = -1;
            }
            return;
        }

        if (finalReverse){
    
            sendSteer(0.0);

            if (hookDetected && !hookLocked){   

                if (areaStatus[4] == -1) {
                    RCLCPP_ERROR(this->get_logger(),"Inconsistent events : Hook detected but no obstacle detected");
                    RCLCPP_ERROR(this->get_logger(),"Final reverse - FAILED");
                    sendVel(0.0);
                    finalReverse = false;
              
                }else if (areaStatus[4] > LOCK_DISTANCE){     //Hook detected => Slow movement until locking distance is reached
                    safeMode = false;
                    sendVel(0.5*FINAL_REVERSE_VELOCITY);

                }else{                  //Lock distance reached => Stop and lock
                    sendVel(0.0);
                    sleep(2);
                    lockHook();
                    safeMode = true;
                    finalReverse = false;

                    RCLCPP_WARN(this->get_logger(),"Start of the towing process");
                    towing = true;
                }

            } else{
                sendVel(FINAL_REVERSE_VELOCITY);
            }

        }else if (towing){

            if (hookLocked){
                
                localMsCounter += PERIOD_UPDATE_MOTION;

                if (localMsCounter <= TOWING_DURATION){    //Towing
                    sendVel(TOWING_VELOCITY);

                } else{     //End of the towing step
                    sendVel(0.0);
                    sleep(2);
                    unlockHook();

                    localMsCounter = 0ms;

                    RCLCPP_WARN(this->get_logger(),"End of the towing process");
                    towing = false;
                }
            }

        }else 
            sendVel(0.0);
        
    }
    
    // ---- Private variables ----
    bool hookDetected, hookLocked = false;

    //Steps
    bool finalReverse = false;
    bool towing = false;

    //Security
    int obstaclesReceived = 0;  //0 and -1 => not received ; 1 => received
    int16_t areaStatus[20];
    bool safeMode;    //If false, the automatic safety control is disabled !! (see safe() function)

    //Time counter
    chrono::milliseconds localMsCounter = 0ms;

    float currentVelocity;
    float currentSteer;

    //Publishers
    rclcpp::Publisher<interfaces::msg::CmdVel>::SharedPtr publisher_cmd_vel_;
    rclcpp::Publisher<interfaces::msg::CmdSteer>::SharedPtr publisher_cmd_steer_;
    rclcpp::Publisher<interfaces::msg::Hook>::SharedPtr publisher_cmd_hook_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::Hook>::SharedPtr subscription_hook_;
    rclcpp::Subscription<interfaces::msg::Obstacles>::SharedPtr subscription_obstacles_;

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