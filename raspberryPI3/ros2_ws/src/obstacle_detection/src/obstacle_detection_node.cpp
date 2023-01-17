#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <chrono>

#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/obstacles.hpp"
#include "interfaces/msg/fixed_obstacles.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/obstacles_id.hpp"

#include "../include/obstacle_detection/obstacle_detection_node.h"

using namespace std;
using namespace std::chrono_literals;
using placeholders::_1;

class ObstacleDetection : public rclcpp::Node{
  public:
    ObstacleDetection()
    : Node("obstacle_detection_node")
    {
      //Array's initialization 
      for(int i = 0 ; i < 6 ; i++){
        area[i]= 0;
      }

      //Variables' initialization
      obstacle_fl = false;
      obstacle_fc = false;
      obstacle_fr = false;
      obstacle_rl = false;
      obstacle_rc = false;
      obstacle_rr = false;

      cnt_fl = 0;
      cnt_fc = 0;
      cnt_fr = 0;
      cnt_rl = 0;
      cnt_rc = 0;
      cnt_rr = 0;

      fixedObstacle_fl = false;
      fixedObstacle_fc = false;
      fixedObstacle_fr = false;
      fixedObstacle_rl = false;
      fixedObstacle_rc = false;
      fixedObstacle_rr = false;
      
      //Publishers
      publisher_obstacle_= this->create_publisher<interfaces::msg::Obstacles>("obstacle", 10);
      publisher_fixed_obstacles_= this->create_publisher<interfaces::msg::FixedObstacles>("fixed_obstacles", 10);
      publisher_obstacles_id_= this->create_publisher<interfaces::msg::ObstaclesID>("obstacles_id", 10);

      //Subscribers 
      subscription_us_data_ = this->create_subscription<interfaces::msg::Ultrasonic>("us_data", 10, std::bind(&ObstacleDetection::usDataCallback, this, _1));
      subscription_motor_data_ = this->create_subscription<interfaces::msg::MotorsFeedback>("motors_feedback", 10, std::bind(&ObstacleDetection::motorsDataCallback, this, _1));

      //This timer calls the methode counterToFive() each 1s
      timerFixedObstacle = this->create_wall_timer(PERIOD_UPDATED_COUNTER_OBSTACLE, std::bind(&ObstacleDetection::counterToFive, this));
      
      RCLCPP_INFO(this->get_logger(), "obstacle_detection_node READY");
    }
  
  private: 
  // ---- Private variables ----
    //Publishers
    rclcpp::Publisher<interfaces::msg::Obstacles>::SharedPtr publisher_obstacle_;
    rclcpp::Publisher<interfaces::msg::FixedObstacles>::SharedPtr publisher_fixed_obstacles_;
    rclcpp::Publisher<interfaces::msg::ObstaclesID>::SharedPtr publisher_obstacles_id_;
    
    //Subscribers
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_us_data_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motor_data_;

    //Timers
    rclcpp::TimerBase::SharedPtr timerFixedObstacle;

    //area array contains 6 zones represented by each ultrasonic sensors
    int16_t area[6];

    //Obstacle_ represents if there is an obstacle in any of 6 zones 
    bool obstacle_fl;
    bool obstacle_fc;
    bool obstacle_fr;
    bool obstacle_rl;
    bool obstacle_rc;
    bool obstacle_rr;

    //Motors' variables
    float_t r_motor;
    float_t l_motor;

    //Counter's variable
    int cnt_fl;
    int cnt_fc;
    int cnt_fr;
    int cnt_rl;
    int cnt_rc;
    int cnt_rr;

    //Fixed Obstacle variable
    bool fixedObstacle_fl;
    bool fixedObstacle_fc;
    bool fixedObstacle_fr;
    bool fixedObstacle_rl;
    bool fixedObstacle_rc;
    bool fixedObstacle_rr;

    /* 
    *
    * This function is called when a message is published on the "/us_data" topic
    * 
    */
  
    void usDataCallback(const interfaces::msg::Ultrasonic & usMsg) {

      // Message à publier
      auto obstacleMsg = interfaces::msg::Obstacles();

      if (usMsg.front_left > OBSTACLE_PRESENT){
        area[0] = -1;
        obstacle_fl = false;
      } else {
        area[0] = usMsg.front_left;
        obstacle_fl = true;
      }
      if (usMsg.front_center > OBSTACLE_PRESENT){
        area[1] = -1;
        obstacle_fc = false;
      } else {
        area[1] = usMsg.front_center;
        obstacle_fc = true;;
      }
      if (usMsg.front_right > OBSTACLE_PRESENT){
        area[2] = -1;
        obstacle_fr = false;
      } else {
        area[2] = usMsg.front_right;
        obstacle_fr = true;
      }
      if (usMsg.rear_left > OBSTACLE_PRESENT){
        area[3] = -1;
        obstacle_rl = false;
      } else {
        area[3] = usMsg.rear_left;
        obstacle_rl = true;
      }
      if (usMsg.rear_center > OBSTACLE_PRESENT){
        area[4] = -1;
        obstacle_rc = false;
      } else {
        area[4] = usMsg.rear_center;
        obstacle_rc = true;
      }
      if (usMsg.rear_right > OBSTACLE_PRESENT){
        area[5] = -1;
        obstacle_rr = false;
      } else {
        area[5] = usMsg.rear_right;
        obstacle_rr = true;
      }

      //Messages modification
      for(int i = 0 ; i < 6 ; i++){
        obstacleMsg.area[i] = area[i];
      }

      // Message publication
      publisher_obstacle_->publish(obstacleMsg);
    }


    void motorsDataCallback(const interfaces::msg::MotorsFeedback & motorMsg) {

      // Message à publier
      auto fixedObstacleMsg = interfaces::msg::FixedObstacles();

      r_motor = motorMsg.right_rear_speed ;
      l_motor = motorMsg.left_rear_speed ;

      //Identification of an fixed obstacle
      if (l_motor == 0.0 && r_motor == 0.0){
        if(fixedObstacle_fl && obstacle_fl){
          fixedObstacleMsg.fixed_obstacles[0] = fixedObstacle_fl;
          RCLCPP_DEBUG(this->get_logger(), "There is a fixed obstacle in zone 1");
        }else if((!fixedObstacle_fl)){
          fixedObstacleMsg.fixed_obstacles[0] = fixedObstacle_fl;
        }
        if(fixedObstacle_fc && obstacle_fc){
          fixedObstacleMsg.fixed_obstacles[1] = fixedObstacle_fc;
          RCLCPP_DEBUG(this->get_logger(), "There is a fixed obstacle in zone 2");
        }else if(!(fixedObstacle_fc)){
          fixedObstacleMsg.fixed_obstacles[1] = fixedObstacle_fc;
        }
        if(fixedObstacle_fr && obstacle_fr){
          fixedObstacleMsg.fixed_obstacles[2] = fixedObstacle_fr;
          RCLCPP_DEBUG(this->get_logger(), "There is a fixed obstacle in zone 3");
        }else if(!(fixedObstacle_fr)){
          fixedObstacleMsg.fixed_obstacles[2] = fixedObstacle_fr;
        }

        if(fixedObstacle_rl && obstacle_rl){
          fixedObstacleMsg.fixed_obstacles[3] = fixedObstacle_rl;
          RCLCPP_DEBUG(this->get_logger(), "There is a fixed obstacle in zone 4");
        }else  if (!(fixedObstacle_rl)){
          fixedObstacleMsg.fixed_obstacles[3] = fixedObstacle_rl;
        }
        if(fixedObstacle_rc && obstacle_rc){
          fixedObstacleMsg.fixed_obstacles[4] = fixedObstacle_rc;
          RCLCPP_DEBUG(this->get_logger(), "There is a fixed obstacle in zone 5");
        }else  if (!(fixedObstacle_rc)){
          fixedObstacleMsg.fixed_obstacles[4] = fixedObstacle_rc;
        }
        if(fixedObstacle_rr && obstacle_rr){
          fixedObstacleMsg.fixed_obstacles[5] = fixedObstacle_rr;
          RCLCPP_DEBUG(this->get_logger(), "There is a fixed obstacle in zone 6");
        }else if (!(fixedObstacle_rr)){
          fixedObstacleMsg.fixed_obstacles[5] = fixedObstacle_rr;
        }
      }

      // Message publication
      publisher_fixed_obstacles_->publish(fixedObstacleMsg);
    }
    
    void counterToFive(){
      if (obstacle_fl){
          cnt_fl ++;
          // RCLCPP_INFO(this->get_logger(), "cnt_fl value: %d",cnt_fl);
        }else {
          cnt_fl = 0;
        }
         if (obstacle_fc){
          cnt_fc ++;
          // RCLCPP_INFO(this->get_logger(), "cnt_fc value: %d",cnt_fc);
        }else {
          cnt_fc = 0;
        }
         if (obstacle_fr){
          cnt_fr ++;
          // RCLCPP_INFO(this->get_logger(), "cnt_fr value: %d",cnt_fr);
        }else {
          cnt_fr = 0;
        }
        if (obstacle_rl){
          cnt_rl ++;
          // RCLCPP_INFO(this->get_logger(), "cnt_rl value: %d",cnt_rl);
        }else {
          cnt_rl = 0;
        }
         if (obstacle_rc){
          cnt_rc ++;
          // RCLCPP_INFO(this->get_logger(), "cnt_rc value: %d",cnt_rc);
        }else {
          cnt_rc = 0;
        }
         if (obstacle_rr){
          cnt_rr ++;
          // RCLCPP_INFO(this->get_logger(), "cnt_rr value: %d",cnt_rr);
        }else {
          cnt_rr = 0;
        }


        if (cnt_fl >= 5){
          fixedObstacle_fl = true;
        }else{
          fixedObstacle_fl = false;
        }
        if (cnt_fc >= 5){
          fixedObstacle_fc = true;
        }else{
          fixedObstacle_fc = false;
        }
        if (cnt_fr >= 5){
          fixedObstacle_fr = true;
        }else{
          fixedObstacle_fr = false;
        }
        if (cnt_rl >= 5){
          fixedObstacle_rl = true;
        }else{
          fixedObstacle_rl = false;
        }
        if (cnt_rc >= 5){
          fixedObstacle_rc = true;
        }else{
          fixedObstacle_rc = false;
        }
        if (cnt_rr >= 5){
          fixedObstacle_rr = true;
        }else{
          fixedObstacle_rr = false;
        }
    }
    void obstacleIDCallBack(const interfaces::msg::FixedObstacles & fixedObstaclesMsg){
      // Message à publier
      auto obstacleIDMsg = interfaces::msg::ObstaclesID();

      if (fixedObstaclesMsg.fixed_obstacles[0] && fixedObstaclesMsg.fixed_obstacles[1] && fixedObstaclesMsg.fixed_obstacles[2]){
        obstacleIDMsg.obstacle_middle = true;
        obstacleIDMsg.big_obstacle = true;
        obstacleIDMsg.obstacle_left = false;
        obstacleIDMsg.obstacle_right = false;

      }else if (fixedObstaclesMsg.fixed_obstacles[0] && fixedObstaclesMsg.fixed_obstacles[1]){
        obstacleIDMsg.obstacle_left = true;
        obstacleIDMsg.big_obstacle = false;
        obstacleIDMsg.obstacle_middle = false;
        obstacleIDMsg.obstacle_right = false;
      }else if (fixedObstaclesMsg.fixed_obstacles[1] && fixedObstaclesMsg.fixed_obstacles[2]){
        obstacleIDMsg.obstacle_right = true;
        obstacleIDMsg.big_obstacle = false;
        obstacleIDMsg.obstacle_left = false;
        obstacleIDMsg.obstacle_middle = false;
      }else if (fixedObstaclesMsg.fixed_obstacles[0]){
        obstacleIDMsg.obstacle_left = true;
        obstacleIDMsg.big_obstacle = false;
        obstacleIDMsg.obstacle_middle = false;
        obstacleIDMsg.obstacle_right = false;
      }else if (fixedObstaclesMsg.fixed_obstacles[1]){
        obstacleIDMsg.obstacle_middle = true;
        obstacleIDMsg.big_obstacle = false;
        obstacleIDMsg.obstacle_left = false;
        obstacleIDMsg.obstacle_right = false;
      }else if (fixedObstaclesMsg.fixed_obstacles[2]){
        obstacleIDMsg.obstacle_right = true;
        obstacleIDMsg.big_obstacle = false;
        obstacleIDMsg.obstacle_left = false;
        obstacleIDMsg.obstacle_middle = false;
      }
      // Message publication
      publisher_obstacles_id_->publish(obstacleIDMsg);
    }

};


int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetection>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
