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
        int size;
        int sum_left = 0;
        int sum_right = 0;
        float mean_left;
        float mean_right;


        // Get the number of points get with the LiDAR
        size = 1800; // sizeof scan.intensities/sizeof scan.intensities[0];
        RCLCPP_INFO(this->get_logger(), "La  moitié de la taille du tableau est %d", (int)(size/2));


        // Get the side where the intensity of the points is the highest
        for(int i=0; i< (int)(size/2) ; i++){
            sum_left = scan.intensities[i] + sum_left;
        }
        for(int i=(int)(size/2); i<size; i++){
            RCLCPP_INFO(this->get_logger(), "boucle/ i = %d ", i);
            sum_right = scan.intensities[i] + sum_right;
        }
        RCLCPP_INFO(this->get_logger(), "Les sommes sont : %d (Left) et %d (right)", sum_left, sum_right);

        mean_left = sum_left / (int)(size/2);
        mean_right = sum_right / (int)(size/2);

        RCLCPP_INFO(this->get_logger(), "Les moyennes sont : %f (Left) et %f (right)", mean_left, mean_right);

        if (mean_left >= mean_right){
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