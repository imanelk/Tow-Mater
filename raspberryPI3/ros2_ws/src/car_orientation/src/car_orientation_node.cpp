#include "rclcpp/rclcpp.hpp"

#include "../include/car_orientation_node.h"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "interfaces/msg/orientation.hpp"

using namespace std;
using placeholders::_1;


class car_orientation : public rclcpp::Node {

public:
    car_orientation()
    : Node("car_orientation_node")
    {

        publisher_orientation_= this->create_publisher<interfaces::msg::Orientation>("orientation", 10);


        subscription_magnetic_dc_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "imu/mag_dc", 10, std::bind(&car_orientation::magDcCallback, this, _1));

        subscription_imu_dc_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10, std::bind(&car_orientation::imuDcCallback, this, _1));

        subscription_magnetic_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "imu/mag", 10, std::bind(&car_orientation::magCallback, this, _1));

        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10, std::bind(&car_orientation::imuCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "car_orientation_node READY");
    }

    
private:


    void imuCallback(const sensor_msgs::msg::Imu & IMU){
        geometry_msgs::msg::Quaternion q;
        q = IMU.orientation;
        phi = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        theta = asin(2 * (q.w * q.y - q.z * q.x));
        psi = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }
    
    void imuDcCallback(const sensor_msgs::msg::Imu & IMU){
        geometry_msgs::msg::Quaternion q;
        q = IMU.orientation;
        phi_dc = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        theta_dc = asin(2 * (q.w * q.y - q.z * q.x));
        psi_dc = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }
    
    void magCallback(const sensor_msgs::msg::MagneticField & MAG){  
        geometry_msgs::msg::Vector3 meas;
        meas = MAG.magnetic_field;      
        direction = atan2(meas.y, meas.x) * rad2deg ;
        RCLCPP_INFO(this->get_logger(), "Damaged car : %f - Towing car : %f", direction_dc, direction);
    }

    void magDcCallback(const sensor_msgs::msg::MagneticField & MAG){  
        geometry_msgs::msg::Vector3 meas = MAG.magnetic_field;      
        direction_dc = atan2(meas.y, meas.x) * rad2deg ;
    }

     
    // ---- Private variables ----

   

    // IMU variables
    float phi ;
    float theta ;
    float psi ;
    float direction ;

    float phi_dc ;
    float theta_dc ;
    float psi_dc ;
    float direction_dc ;

    // Constants
    float pi = 3.14159265359 ;
    float rad2deg = 180 / pi;


    //Publisher
    rclcpp::Publisher<interfaces::msg::Orientation>::SharedPtr publisher_orientation_;


    //Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_dc_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr subscription_magnetic_dc_;
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