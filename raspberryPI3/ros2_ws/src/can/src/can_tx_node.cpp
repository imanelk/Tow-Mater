#include <stdint.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/system_check.hpp"
#include "interfaces/msg/hook.hpp"

#include "../include/can/can.h"

using std::placeholders::_1;



class can_tx : public rclcpp::Node {
public:
  can_tx()
  : Node("can_tx_node")
  {
    if(initCommunication()==0) {
      subscription_motors_order_ = this->create_subscription<interfaces::msg::MotorsOrder>(
      "motors_order", 10, std::bind(&can_tx::sendMotorsOrderCallback, this, _1));

      subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
      "steering_calibration", 10, std::bind(&can_tx::sendCalibrationRequestCallback, this, _1));

      subscription_system_check_ = this->create_subscription<interfaces::msg::SystemCheck>(
      "system_check", 10, std::bind(&can_tx::sendCommunicationRequestCallback, this, _1));

      subscription_hook_ = this->create_subscription<interfaces::msg::Hook>(
      "hook", 10, std::bind(&can_tx::hookLockingOrderCallback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Ready to transmit");
    }
  }

  int closeCommunication(){

    if (s==0 && close(s) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Close communication failed - Socket close error");
        return 1;
    }
    RCLCPP_INFO(this->get_logger(), "CAN communication closed");
    return 0;
  }


private:

  /* Initialize CAN communicaction :
  * 1. create socket
  * 2. bind
  */
  int initCommunication(){
        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "CAN initialization failed - Socket creation error");
            return 1;
        }

        strcpy(ifr.ifr_name, "can0" );
        ioctl(s, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "CAN initialization failed - Bind error");
            return 1;
        }

        frame.can_dlc = 8;

      return 0;
    }

    /* Send motors orders via CAN bus [callback function]  :
    * This function is called when a message is published on the "/motors_order" topic
    */
    int sendMotorsOrderCallback(const interfaces::msg::MotorsOrder & motorsOrder) {

      frame.can_id = ID_MOTORS_CMD;

      frame.data[0] = motorsOrder.left_rear_pwm;
      frame.data[1] = motorsOrder.right_rear_pwm;
      frame.data[2] = motorsOrder.steering_pwm;

      return canSend(frame);
    }

    /* Send steering calibration request via CAN bus [callback function]  :
    * This function is called when a message is published on the "/steering_calibration" topic
    */
    int sendCalibrationRequestCallback(const interfaces::msg::SteeringCalibration & calibrationMsg) {

      if (calibrationMsg.request == true){
        frame.can_id = ID_CALIBRATION_MODE;
        frame.data[0] = CALIBRATION_REQUEST;

        return canSend(frame);
      }

      return 0;
      
    }

    /* Send checking communcation request [callback function]
    * This function is called when a message is published on the "/system_check" topic
    */
    int sendCommunicationRequestCallback(const interfaces::msg::SystemCheck & checkingMsg) {

      if (checkingMsg.request == true){
        
        frame.can_id = ID_COMM_CHECKING;
        frame.data[0] = COMM_CHECKING_REQUEST;
    
        return canSend(frame);

      }
      return 0;
    }

    /* Send hook locking order [callback function]
    * This function is called when a message is published on the "/hook" topic
    */
    int hookLockingOrderCallback(const interfaces::msg::Hook & hookMsg) {

      if (hookMsg.type == "lock" && hookMsg.status == true){
        
        frame.can_id = ID_HOOK;
        frame.data[0] = HOOK_LOCK;
    
        return canSend(frame);

      }else if (hookMsg.type == "lock" && hookMsg.status == false){
        
        frame.can_id = ID_HOOK;
        frame.data[0] = HOOK_UNLOCK;
    
        return canSend(frame);
      }
      return 0;
    }
    

    //Sends frame via CAN bus
    int canSend(struct can_frame frame){

      if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Sending fail, can ID 0x%03X", frame.can_id);
        return 1;
      }

      RCLCPP_DEBUG(this->get_logger(), "Sending success, can ID 0x%03X", frame.can_id);
      return 0;

    }

    //CAN variables
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::MotorsOrder>::SharedPtr subscription_motors_order_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steering_calibration_;
    rclcpp::Subscription<interfaces::msg::SystemCheck>::SharedPtr subscription_system_check_;
    rclcpp::Subscription<interfaces::msg::Hook>::SharedPtr subscription_hook_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<can_tx>();
  rclcpp::spin(node);

  node->closeCommunication();
  rclcpp::shutdown();
  return 0;
}