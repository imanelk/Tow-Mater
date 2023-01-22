#include "rclcpp/rclcpp.hpp"


#include "../include/car_orientation_node.h"


#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

using namespace std;
using placeholders::_1;


class car_orientation : public rclcpp::Node {

public:
    car_orientation()
    : Node("car_orientation_node")
    {

        start = false; // Variable used to identify the first time GPS coordinates are received 

        publisher_navigation_= this->create_publisher<interfaces::msg::Navigation>("navigation", 10);

        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10, std::bind(&motion_planning::imuCallback, this, _1));

         subscription_magnetic_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "imu/mag", 10, std::bind(&motion_planning::magCallback, this, _1));


        RCLCPP_INFO(this->get_logger(), "car_orientation_node READY");
    }

    
private:


    void imuCallback(const sensor_msgs::msg::Imu & IMU){
        quaternion_to_euler(IMU.orientation) ;
    }
    
    void quaternion_to_euler(geometry_msgs::msg::Quaternion q)
    {
        phi = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        theta = asin(2 * (q.w * q.y - q.z * q.x));
        psi = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }

    void magCallback(const sensor_msgs::msg::MagneticField & MAG){
        
        magnetic_field_compass(MAG.magnetic_field) ;
    }
    void magnetic_field_compass(geometry_msgs::msg::Vector3 meas)
    {
        direction = atan2(meas.y, meas.x) * rad2deg ;
    }

     
    // ---- Private variables ----

   

    // IMU variables
    float phi ;
    float theta ;
    float psi ;

    float direction ;
    float pi = 3.14159265359 ;
    float rad2deg = 180 / pi ;


    //Publishers
    rclcpp::Publisher<interfaces::msg::Navigation>::SharedPtr publisher_navigation_;

    //Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr subscription_magnetic_;
    
    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_orientation>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}