#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "interfaces/msg/obstacle_side.hpp"
#include "../include/side_detection_node.h"

using namespace std;
using placeholders::_1;


class side_detection : public rclcpp::Node {

public:
    side_detection()
    : Node("side_detection_node")
    {

        publisher_side_= this->create_publisher<interfaces::msg::ObstacleSide>("obstacle_side", 10);

        subscription_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&side_detection::scanDataCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "side_detection_node READY");
    }

    
private:

    /* Update the side where there is a closer obstacle 
    *
    *  This function is called when a message is published on the "/scan" topic
    */
    void scanDataCallback(const sensor_msgs::msg::LaserScan & scan) {
        auto sideMsg = interfaces::msg::ObstacleSide();  
        int size = (int)scan.ranges.size();
  
        float left_distance = 0, right_distance = 0;
        int left_count = 0, right_count = 0;

         
        RCLCPP_INFO(this->get_logger(), "La taille du tableau est : %d", size);

        for (int i = 0; i < size; i++)
        {
            if (i < size / 2)
            {
                if (scan.ranges[i] > 0 && scan.ranges[i] < scan.range_max)
                {
                    left_distance += scan.ranges[i];
                    left_count++;
                }
            }
            else
            {
                if (scan.ranges[i] > 0 && scan.ranges[i] < scan.range_max)
                {
                    right_distance += scan.ranges[i];
                    right_count++;
                }
            }
        }

        left_distance /= left_count;
        right_distance /= right_count;

        RCLCPP_INFO(this->get_logger(), "Left distance: %f, Right distance: %f", left_distance, right_distance);   
        


        if (left_distance >= right_distance){
            sideMsg.left_lidar = true;
            sideMsg.right_lidar = false;
        }
        else{
            sideMsg.left_lidar = false;
            sideMsg.right_lidar = true;
        }

        publisher_side_->publish(sideMsg);
    }

     
    // ---- Private variables ----

    //Publishers
    rclcpp::Publisher<interfaces::msg::ObstacleSide>::SharedPtr publisher_side_;

    //Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<side_detection>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}