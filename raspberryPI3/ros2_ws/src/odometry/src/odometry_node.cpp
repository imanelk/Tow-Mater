#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/motors_feedback.hpp"
#include "nav_msgs/msg/odometry.hpp"


#include "../include/odometry/odometry_node.h"
#include "../include/odometry/speedCalcul.h"

using namespace std;
using placeholders::_1;


class odometry : public rclcpp::Node {

public:
    odometry()
    : Node("odometry_node")
    {
   
        /* Publishers */
        publisher_car_control_= this->create_publisher<nav_msgs::msg::Odometry>("wheel_odometry", 10);

        /* Subscribers */
        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&odometry::motorsFeedbackCallback, this, _1));

        /* Timer for update */
        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&odometry::updateCmd, this));

        
        RCLCPP_INFO(this->get_logger(), "odometry_node READY");
    }

    
private:

      /* Update currentAngle from motors feedback [callback function]  :
    *
    * This function is called when a message is published on the "/motors_feedback" topic
    * 
    */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
        currentLeftRPM = motorsFeedback.left_rear_speed;
        currentRightRPM = motorsFeedback.right_rear_speed;
    }

    /* Update wheel speed : leftRearPwmCmd, rightRearPwmCmd, steeringPwmCmd
    *
    * This function is called periodically by the timer [see PERIOD_UPDATE_CMD in "odometry_node.h"]
    *
    */
    void updateCmd(){

        auto odometry = nav_msgs::msg::Odometry();

        // Convert from RPM to a m/s 
        leftRearSpeed = rpmToMps(currentLeftRPM);
        rightRearSpeed = rpmToMps(currentRightRPM);

        // First version : the means of the two wheels speeds
        linearSpeed = (leftRearSpeed + rightRearSpeed)/2;


        //Send odometries to car control 
        odometry.twist.twist.linear.x = linearSpeed;
        publisher_car_control_->publish(odometry);

    }

    
    // ---- Private variables ----

    //Motors feedback variables
    float currentLeftRPM;
    float currentRightRPM;

    //Speed variables
    uint8_t linearSpeed;
    uint8_t leftRearSpeed;
    uint8_t rightRearSpeed;

    //Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_car_control_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<odometry>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}