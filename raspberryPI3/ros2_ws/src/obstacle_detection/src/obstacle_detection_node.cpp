#include "rclcpp/rclcpp.hpp"
#include <cstdio>

#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/obstacles.hpp"

#include "../include/obstacle_detection/obstacle_detection_node.h"

using namespace std;
using placeholders::_1;

class obstacle_detection : public rclcpp::Node {
  public:
    obstacle_detection()
    : Node("obstacle_detection_node")
    {
      //Array's initialzation 
      for(int i = 0 ; i < 6 ; i++){
        area[i]= 0;
      }

      publisher_obstacle_= this->create_publisher<interfaces::msg::Obstacles>("obstacle", 10);
  
      
      subscription_us_data_ = this->create_subscription<interfaces::msg::Ultrasonic>(
      "us_data", 10, std::bind(&obstacle_detection::usDataCallback, this, _1));
 
      
      RCLCPP_INFO(this->get_logger(), "obstacle_detection_node READY");
    }
  
  private: 
  // ---- Private variables ----
    //Publishers
    rclcpp::Publisher<interfaces::msg::Obstacles>::SharedPtr publisher_obstacle_;
    
    //Subscribers
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_us_data_;
    
    //area array contains 6 zones represented by each ultrasonic sensors
    int16_t area[6];

    /* 
    *
    * This function is called when a message is published on the "/us_data" topic
    * 
    */
  
    void usDataCallback(const interfaces::msg::Ultrasonic & usMsg) {

      //message à publier
      auto obstacleMsg = interfaces::msg::Obstacles();

      if (usMsg.front_left > OBSTACLE_PRESENT){
        area[0] = -1;
      } else {
        area[0] = usMsg.front_left;
      }
      if (usMsg.front_center > OBSTACLE_PRESENT){
        area[1] = -1;
      } else {
        area[1] = usMsg.front_center;
      }
      if (usMsg.front_right > OBSTACLE_PRESENT){
        area[2] = -1;
      } else {
        area[2] = usMsg.front_right;
      }
      if (usMsg.rear_left > OBSTACLE_PRESENT){
        area[3] = -1;
      } else {
        area[3] = usMsg.rear_left;
      }
      if (usMsg.rear_center > OBSTACLE_PRESENT){
        area[4] = -1;
      } else {
        area[4] = usMsg.rear_center;
      }
      if (usMsg.rear_right > OBSTACLE_PRESENT){
        area[5] = -1;
      } else {
        area[5] = usMsg.rear_right;
      }

      //Modification du message avant
      for(int i = 0 ; i < 6 ; i++){
        obstacleMsg.area[i] = area[i];
      }
      

      // Publication du message
      publisher_obstacle_->publish(obstacleMsg);
    }
};




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<obstacle_detection>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
