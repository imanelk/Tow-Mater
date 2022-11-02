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

        publisher_cmd_vel_= this->create_publisher<interfaces::msg::CmdVel>("cmd/vel", 10);
        publisher_cmd_steer_= this->create_publisher<interfaces::msg::CmdSteer>("cmd/steer", 10);
        publisher_cmd_hook_= this->create_publisher<interfaces::msg::Hook>("hook", 10);

       

        subscription_hook_ = this->create_subscription<interfaces::msg::Hook>(
        "hook", 10, std::bind(&motion_planning::hookCallback, this, _1));

        subscription_obstacles_ = this->create_subscription<interfaces::msg::Obstacles>(
        "obstacles", 10, std::bind(&motion_planning::obstaclesFeedback, this, _1));


        timer_security_ = this->create_wall_timer(PERIOD_CHECK_SECURITY, std::bind(&motion_planning::checkSecurity, this));

        timer_motion_planning_ = this->create_wall_timer(PERIOD_UPDATE_MOTION, std::bind(&motion_planning::motionPlanning, this));

        sendVel(INITIAL_VELOCITY);
        sendSteer(INITIAL_STEER);

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
        for (int i=0 ; i<NUMBER_OF_CRITICAL_AREAS ; i++)
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
    void lockHook(){
        auto hookLockMsg = interfaces::msg::Hook();

        hookLockMsg.type = "lock";
        hookLockMsg.status = true;
        publisher_cmd_hook_->publish(hookLockMsg);

        RCLCPP_INFO(this->get_logger(), "Hook Locking");
        sleep(LOCK_WAITING_TIME);

        hookLocked = true;
        
    }

    /*  Manages the movements of the car: Decision-making entity  */
    void motionPlanning(){

        if (finalReverse){
    
            sendSteer(0.0);

            if (hookDetected && !hookLocked){

                if (areaStatus[4] > LOCK_DISTANCE){
                    safeMode = false;
                    sendVel(0.5*FINAL_REVERSE_VELOCITY);

                }else{ 
                    sendVel(0.0);
                    lockHook();
                    safeMode = true;
                }

            } else if (hookLocked){
                sendVel(TOWING_VELOCITY);

            } else{
                sendVel(FINAL_REVERSE_VELOCITY);
            }
        }
        
    }
    
    // ---- Private variables ----
    bool hookDetected, hookLocked = false;

    //Steps
    bool finalReverse = false;

    //Security
    int8_t areaStatus[20];
    bool safeMode;    //If false, the automatic safety control is disabled !! (see safe() function)

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