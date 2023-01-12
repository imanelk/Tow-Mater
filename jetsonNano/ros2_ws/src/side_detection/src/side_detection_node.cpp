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
        "scan", 10, std::bind(&obstacle_detector::scanCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "side_detection_node READY");
    }

    
private:

    /* 
    */
    void scanDataCallback(const sensor_msgs::msg::LaserScan & scan) {
        int size;
        int sum_left = 0;
        int sum_right = 0;

        size = sizeof scan.intensities/sizeof scan.intensities[0];
        RCLCPP_INFO(this->get_logger(), "La taille du tablea est %d", size);

        for(int i=0; i<= (int)size/2; i++){
            sum_left = scan.intensities[i] + sum_left;
        }
        for(int i=(int)size/2; i<size; i++){
            sum_right = scan.intensities[i] + sum_right;
        }

      
    }

     
    // ---- Private variables ----


    //Publishers
    rclcpp::Publisher<interfaces::msg::ObstacleSide>::SharedPtr publisher_side_;

    //Subscribers
    rclcpp::Subscription<sensor_msgs::LaserScan>::SharedPtr subscription_scan_;

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