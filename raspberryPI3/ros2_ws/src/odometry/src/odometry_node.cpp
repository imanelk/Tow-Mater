#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/motors_feedback.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "interfaces/msg/distance.hpp"

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

        /* Publishers */
        publisher_distance_= this->create_publisher<interfaces::msg::Distance>("distance", 10);


        /* Subscribers */
        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&odometry::motorsFeedbackCallback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "odometry_node READY");
    }

    
private:

    float calculateSpeed(float leftSpeed, float rightSpeed){
        // Convert from RPM to a m/s 
        float leftRearSpeed = rpmToMps(leftSpeed);
        float rightRearSpeed = rpmToMps(rightSpeed);

        // First version : the means of the two wheels speeds
        linearSpeed = (leftRearSpeed + rightRearSpeed)/2;

        return linearSpeed;
    }

    float calculateDistance(int leftRearOdometry, int rightRearOdometry){
        float distanceLeft = (leftRearOdometry * M_PI * (WHEEL_DIAMETER/10.0)) / 36.0;
        float distanceRight = (rightRearOdometry * M_PI * (WHEEL_DIAMETER/10.0)) / 36.0;

        float distance = (distanceLeft + distanceRight)/2.0;
        return distance;
    }

      /* Compute and send the total distance from motors feedback [callback function]  :
    *
    * This function is called when a message is published on the "/motors_feedback" topic
    * 
    */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){

        linearSpeed = calculateSpeed(motorsFeedback.left_rear_speed,motorsFeedback.right_rear_speed);
        lastDistance = calculateDistance(motorsFeedback.left_rear_odometry,motorsFeedback.right_rear_odometry);
        totalDistance += lastDistance; 


        auto odometryMsg = nav_msgs::msg::Odometry();

        //Send odometries to car control 
        odometryMsg.twist.twist.linear.x = linearSpeed;
        publisher_car_control_->publish(odometryMsg);

        //Send distance data to /distance topic
        auto distanceMsg = interfaces::msg::Distance();

        distanceMsg.total = totalDistance;
        distanceMsg.last = lastDistance;
        publisher_distance_->publish(distanceMsg);


    }

    
    // ---- Private variables ----

    //Speed variables
    float linearSpeed = 0.0;  //current linear speed [m/s]

    //Distance variables
    float totalDistance = 0.0;    //total distance traveled [cm]
    float lastDistance = 0.0;     //distance traveled since the last feedback

    //Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_car_control_;
    rclcpp::Publisher<interfaces::msg::Distance>::SharedPtr publisher_distance_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<odometry>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}