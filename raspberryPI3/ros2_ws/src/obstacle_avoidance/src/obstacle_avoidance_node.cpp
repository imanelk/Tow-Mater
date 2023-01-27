#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/avoidance_parameters.hpp"
#include "interfaces/msg/obstacles_id.hpp"
#include "interfaces/msg/obstacle_side.hpp"


#include "../include/obstacle_avoidance/obstacle_avoidance_node.h"
#include <fstream> 


using namespace std;
using placeholders::_1;


class obstacle_avoidance : public rclcpp::Node {

public:
    obstacle_avoidance()
    : Node("obstacle_avoidance_node")
    {
        subscription_lidar_ = this->create_subscription<interfaces::msg::ObstacleSide>(
        "obstacle_side", 10, std::bind(&obstacle_avoidance::lidarDataCallback, this, _1));

        subscription_us_ = this->create_subscription<interfaces::msg::ObstaclesID>(
        "obstacles_id", 10, std::bind(&obstacle_avoidance::usDataCallback, this, _1));

        publisher_avoidance_param_= this->create_publisher<interfaces::msg::AvoidanceParameters>("avoidance_parameters", 10);

        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&obstacle_avoidance::updateParam, this));

        RCLCPP_INFO(this->get_logger(), "obstacle_avoidance_node READY");
    }

    
private:


    /* Update the result of the Lidar analyse
    *
    *  This function is called when a message is published on the "/obstacle_side" topic
    */
    void lidarDataCallback(const interfaces::msg::ObstacleSide & lidar_obstacle) {
        lidar_left = lidar_obstacle.left_lidar;
        lidar_right = lidar_obstacle.right_lidar;
        left_min = lidar_obstacle.left_min;
        right_min = lidar_obstacle.right_min;
    }


    /* Update the result of the Ultrasonic Sensors analyse
    *
    *  This function is called when a message is published on the "/obstacle_id" topic
    */
    void usDataCallback(const interfaces::msg::ObstaclesID & us_obstacle) {
        us_left = us_obstacle.obstacle_left;
        us_right = us_obstacle.obstacle_right;
        us_middle = us_obstacle.obstacle_middle;
        us_big = us_obstacle.big_obstacle;
    }


    /* Update Avoidance parameters : right, left, big, small
    *
    * This function is called periodically by the timer [see PERIOD_UPDATE_CMD in "obstacle_avoidance_node.h"]
    * 
    */
    void updateParam(){

        auto avoidanceParam = interfaces::msg::AvoidanceParameters();
        avoidanceParam.big = false;

        if (us_big){                                        /* Big obstacle */
            avoidanceParam.big = true;
        }

        if (lidar_left && !lidar_right){                    /* Harder to go by the left than the right : */
            if(us_left && !us_right && !us_middle){             // Obstacle on the left ->
                avoidanceParam.left = false;                    // Go by the right
            }
            else if(!us_left && us_right && !us_middle){        // Obstacle on the right ->    
                if (left_min >= AVOIDANCE_DISTANCE) {       
                    avoidanceParam.left = true;                 // + space on the left -> Go by the left 
                    avoidanceParam.big = false;
                }
                else{
                    avoidanceParam.left = false;                // + no space on the left -> Go by the right 
                    avoidanceParam.big = true;
                }
            }
             else if(us_middle){                                // Obstacle on the middle ->             
                avoidanceParam.left = false;                    // Go by the right
                avoidanceParam.big = true;
            }
        }
        else if(!lidar_left && lidar_right){                /* Harder to go by the right than the left : */
            if(us_left && !us_right && !us_middle){             // Obstacle on the left ->
               if (right_min >= AVOIDANCE_DISTANCE) {       
                    avoidanceParam.left = false;                 // + space on the right -> Go by the right 
                    avoidanceParam.big = false;
                }
                else{
                    avoidanceParam.left = true;                  // + no space on the right -> Go by the left 
                    avoidanceParam.big = true;
                }
            }
            else if(!us_left && us_right && !us_middle){       // Obstacle on the right ->             
                avoidanceParam.left = true;                     // Go by the left
            }
            else if(us_middle){                                // Obstacle on the middle ->             
                avoidanceParam.left = true;                    // Go by the left
                avoidanceParam.big = true;
            }

        }
 
        publisher_avoidance_param_->publish(avoidanceParam);
    }

     
    // ---- Private variables ----


    // The obstacle analyse with the Ultrasonic Sensors
    bool us_left;
    bool us_right;
    bool us_middle;
    bool us_big;

    // The obstacle analyse with the Lidar
    bool lidar_left;
    bool lidar_right;
    float left_min;
    float right_min;

    //Publishers
    rclcpp::Publisher<interfaces::msg::AvoidanceParameters>::SharedPtr publisher_avoidance_param_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::ObstacleSide>::SharedPtr subscription_lidar_;
    rclcpp::Subscription<interfaces::msg::ObstaclesID>::SharedPtr subscription_us_;


    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<obstacle_avoidance>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}