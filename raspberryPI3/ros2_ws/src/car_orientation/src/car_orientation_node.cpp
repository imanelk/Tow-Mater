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
        "imu/data_raw_dc", 10, std::bind(&car_orientation::imuDcCallback, this, _1));

        subscription_magnetic_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "imu/mag", 10, std::bind(&car_orientation::magCallback, this, _1));

        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10, std::bind(&car_orientation::imuCallback, this, _1));

        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_orientation::updateDirection, this));


        RCLCPP_INFO(this->get_logger(), "car_orientation_node READY");
    }

    
private:

    /* Update phi, theta and psi from IMU [callback function]  :
    *
    * This function is called when a message is published on the "/imu/data_raw" topic
    * 
    */
    void imuCallback(const sensor_msgs::msg::Imu & IMU){
        geometry_msgs::msg::Quaternion q;
        q = IMU.orientation;
        phi = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        theta = asin(2 * (q.w * q.y - q.z * q.x));
        psi = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }
    
    /* Update phi_dc, theta_dc and psi_dc from IMU [callback function]  :
    *
    * This function is called when a message is published on the "/imu/data_raw_dc" topic
    * 
    */
    void imuDcCallback(const sensor_msgs::msg::Imu & IMU){
        geometry_msgs::msg::Quaternion q;
        q = IMU.orientation;
        phi_dc = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        theta_dc = asin(2 * (q.w * q.y - q.z * q.x));
        psi_dc = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }
    
    /* Update direction from IMU [callback function]  :
    *
    * This function is called when a message is published on the "/imu/mag" topic
    * 
    */
    void magCallback(const sensor_msgs::msg::MagneticField MAG){  

        update_min_max(MAG.magnetic_field) ;
        correct_mag(MAG.magnetic_field) ;    
        direction = atan2(corrected_y, corrected_x) * rad2deg ;
    }

    /* Update direction_dc from IMU [callback function]  :
    *
    * This function is called when a message is published on the "/imu/mag_dc" topic
    * 
    */
    void magDcCallback(const sensor_msgs::msg::MagneticField MAG){  
        // geometry_msgs::msg::Vector3 meas = MAG.magnetic_field; 
        update_min_max_dc(MAG.magnetic_field) ;
        correct_mag_dc(MAG.magnetic_field) ;
        direction_dc = atan2(corrected_y_dc, corrected_x_dc) * rad2deg ;
    }

    void update_min_max_dc(const geometry_msgs::msg::Vector3 meas)
    {
        if (meas.x < xmin_dc){
            xmin_dc = meas.x ;
        }
        if (meas.x > xmax_dc){
            xmax_dc = meas.x ;
        }
        if (meas.y < ymin_dc){
            ymin_dc = meas.y ;
        }
        if (meas.y > ymax_dc){
            ymax_dc = meas.y ;
        }
        if (meas.z < zmin_dc){
            zmin_dc = meas.z ;
        }
        if (meas.z > zmax_dc){
            zmax_dc = meas.z ;
        } 
    }
    void correct_mag_dc(const geometry_msgs::msg::Vector3 meas){
        float offset_x = (xmax_dc + xmin_dc)/2 ;
        float offset_y = (ymax_dc + ymin_dc)/2 ;
        
        float avg_delta_x = (xmax_dc - xmin_dc)/2 ;
        float avg_delta_y = (ymax_dc - ymin_dc)/2 ;
        float avg_delta_z = (zmax_dc - zmin_dc)/2 ;

        float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z)/3 ;

        float scale_x = avg_delta / avg_delta_x ;
        float scale_y = avg_delta / avg_delta_y ;


        corrected_x_dc = (meas.x - offset_x) * scale_x ;
        corrected_y_dc = (meas.y - offset_y) * scale_y ;

    }

    void update_min_max(const geometry_msgs::msg::Vector3 meas)
    {
        if (meas.x < xmin){
            xmin = meas.x ;
        }
        if (meas.x > xmax){
            xmax = meas.x ;
        }
        if (meas.y < ymin){
            ymin = meas.y ;
        }
        if (meas.y > ymax){
            ymax = meas.y ;
        }
        if (meas.z < zmin){
            zmin = meas.z ;
        }
        if (meas.z > zmax){
            zmax = meas.z ;
        } 
    }
    void correct_mag(const geometry_msgs::msg::Vector3 meas){
        float offset_x = (xmax + xmin)/2 ;
        float offset_y = (ymax + ymin)/2 ;
        
        float avg_delta_x = (xmax - xmin)/2 ;
        float avg_delta_y = (ymax - ymin)/2 ;
        float avg_delta_z = (zmax - zmin)/2 ;

        float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z)/3 ;

        float scale_x = avg_delta / avg_delta_x ;
        float scale_y = avg_delta / avg_delta_y ;


        corrected_x = (meas.x - offset_x) * scale_x ;
        corrected_y = (meas.y - offset_y) * scale_y ;

    }

    /* 
    *
    * This function compares the two cars (towing and damaged) directions
    * and publish on the /orientation topic the information
    * 
    */
    void updateDirection(){
        auto orientation = interfaces::msg::Orientation();

        RCLCPP_INFO(this->get_logger(), "Towing car : %f - Damaged car : %f ", direction, direction_dc);

        if (direction - THRESHOLD <= direction_dc && direction + THRESHOLD >= direction_dc){
            orientation.same_orientation = true;
        }
        else {
            orientation.same_orientation = false;
        }

        publisher_orientation_->publish(orientation);
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

    float xmin = 100 ;
    float xmax = 0 ;
    float ymin = 100 ;
    float ymax = 0 ;
    float zmin = 100 ;
    float zmax = 0 ;

    float corrected_x ;
    float corrected_y ;

    float xmin_dc = 100 ;
    float xmax_dc = 0 ;
    float ymin_dc = 100 ;
    float ymax_dc = 0 ;
    float zmin_dc = 100 ;
    float zmax_dc = 0 ;

    float corrected_x_dc ;
    float corrected_y_dc ;


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