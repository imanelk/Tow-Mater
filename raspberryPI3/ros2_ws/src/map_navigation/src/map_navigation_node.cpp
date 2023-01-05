#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/gnss.hpp"
#include "interfaces/msg/navigation.hpp"

#include "../include/map_navigation_node.h"



using namespace std;
using placeholders::_1;


class map_navigation : public rclcpp::Node {

public:
    map_navigation()
    : Node("map_navigation_node")
    {

        publisher_navigation_= this->create_publisher<interfaces::msg::Navigation>("navigation", 10);

    
        subscription_gps_dc_ = this->create_subscription<interfaces::msg::Gnss>(
        "gnss_data_dc", 10, std::bind(&map_navigation::gnssDataCallback, this, _1));

        
        RCLCPP_INFO(this->get_logger(), "map_navigation_node READY");
    }

    
private:

    /* Update the towing status, it detects when the damaged car sends GPS coordinates and starts the towing car intervention
    *
    *  This function is called when a message is published on the "/gnss_data_dc" topic
    * 
    */
    void gnssDataCallback(const interfaces::msg::Gnss & gnssDc) {
        auto navigationMsg = interfaces::msg::Navigation();    

        dclatitude = gnssDc.latitude;
        dclongitude = gnssDc.longitude;
        dcaltitude = gnssDc.altitude;

        navigationMsg.start = true;
        navigationMsg.dclatitude = dclatitude;
        navigationMsg.dclongitude = dclongitude;
        navigationMsg.dcaltitude = dcaltitude;
        
        publisher_navigation_->publish(navigationMsg);

        RCLCPP_INFO(this->get_logger(), "GPS coordinations received, towing car intervention READY");
    }

     
    // ---- Private variables ----

   
    //Gnss coordinates of the damages car
    float dclatitude;
    float dclongitude;
    float dcaltitude;

    //Publishers
    rclcpp::Publisher<interfaces::msg::Navigation>::SharedPtr publisher_navigation_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::Gnss>::SharedPtr subscription_gps_dc_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<map_navigation>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}