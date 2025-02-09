#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/cmd_vel.hpp"
#include "interfaces/msg/pid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "interfaces/msg/cmd_steer.hpp"

#include "std_srvs/srv/empty.hpp"

#include "../include/car_control/steeringCmd.h"
#include "../include/car_control/propulsionCmd.h"
#include "../include/car_control/car_control_node.h"

using namespace std;
using placeholders::_1;


class car_control : public rclcpp::Node {

public:
    car_control()
    : Node("car_control_node")
    {
        start = false;
        mode = 0;
        requestedThrottle = 0;
        requestedSteerAngle = 0;
        autonomousSteerAngle = 0;
        stopSteer = false;

        requestedWheelsSpeedRPM = 0.0;
        requestedWheelsSpeedMPS = 0.0;

        Kp = 0.8;
        Ki = 0.0;
        Kd = 0.0;
    

        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);

        

        subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
        "joystick_order", 10, std::bind(&car_control::joystickOrderCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
        "steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));

        subscription_linear_speed_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "linear_speed", 10, std::bind(&car_control::linearSpeedCallback, this, _1));

        subscription_consign_speed_ = this->create_subscription<interfaces::msg::CmdVel>(
        "consign_speed", 10, std::bind(&car_control::consignSpeedCallback, this, _1));

        subscription_consign_steer_ = this->create_subscription<interfaces::msg::CmdSteer>(
        "consign_steer", 10, std::bind(&car_control::consignSteerCallback, this, _1));

        subscription_pid_ = this->create_subscription<interfaces::msg::Pid>(
        "pid", 10, std::bind(&car_control::pidCallback, this, _1));

        

        server_calibration_ = this->create_service<std_srvs::srv::Empty>(
                            "steering_calibration", std::bind(&car_control::steeringCalibration, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_control::updateCmd, this));

        
        RCLCPP_INFO(this->get_logger(), "car_control_node READY");
    }

    
private:

    /* Update start, mode, requestedThrottle, requestedSteerAngle and reverse from joystick order [callback function]  :
    *
    *  This function is called when a message is published on the "/joystick_order" topic
    * 
    */
    void joystickOrderCallback(const interfaces::msg::JoystickOrder & joyOrder) {

        if (joyOrder.start != start){
            start = joyOrder.start;

            if (start)
                RCLCPP_INFO(this->get_logger(), "START");
            else 
                RCLCPP_INFO(this->get_logger(), "STOP");
        }
        

        if (joyOrder.mode != mode && joyOrder.mode != -1){ //if mode change
            mode = joyOrder.mode;

            if (mode==0){
                RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            }else if (mode==1){
                RCLCPP_INFO(this->get_logger(), "Switching to AUTONOMOUS Mode");
            }else if (mode==2){
                RCLCPP_INFO(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
                startSteeringCalibration();
            }
        }
        
        if (mode == 0 && start){  //if manual mode -> update requestedThrottle, requestedSteerAngle and reverse from joystick order
            requestedThrottle = joyOrder.throttle;
            requestedSteerAngle = joyOrder.steer;
            reverse = joyOrder.reverse;
        }
    }

    /* Update currentAngle from motors feedback [callback function]  :
    *
    * This function is called when a message is published on the "/motors_feedback" topic
    * 
    */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
        currentAngle = motorsFeedback.steering_angle;
        currentLeftSpeedRPM = motorsFeedback.left_rear_speed;
        currentRightSpeedRPM = motorsFeedback.right_rear_speed;
    }

    /* Update currentSpeed from odometry [callback function]  :
    *
    * This function is called when a message is published on the "/odometry" topic
    * 
    */
    void linearSpeedCallback(const nav_msgs::msg::Odometry & odometry){
        currentCarSpeedMPS = odometry.twist.twist.linear.x;
    }

    /* Update currentSpeed from odometry [callback function]  :
    *
    * This function is called when a message is published on the "/odometry" topic
    * 
    */
    void consignSpeedCallback(const interfaces::msg::CmdVel & cmdVel){
        requestedWheelsSpeedMPS = cmdVel.velocity;
        //The speed command in m/s is converted in RPM
        requestedWheelsSpeedRPM = mpsToRpm(requestedWheelsSpeedMPS);
        // Initialize the command errors
        errorPreviousLeft = 0;
        errorPreviousRight = 0;
        errorSumLeft = 0;
        errorSumRight = 0;

    }

    void consignSteerCallback(const interfaces::msg::CmdSteer & cmdSteer){
        autonomousSteerAngle = cmdSteer.angle ;
        stopSteer = cmdSteer.stop;

    }

    /* Update currentSpeed from odometry [callback function]  :
    *
    * This function is called when a message is published on the "/odometry" topic
    * 
    */
    void pidCallback(const interfaces::msg::Pid & pid){
        Kp = pid.kp;
        Ki = pid.ki;
        Kd = pid.kd;
    }

    

    /* Update PWM commands : leftRearPwmCmd, rightRearPwmCmd, steeringPwmCmd
    *
    * This function is called periodically by the timer [see PERIOD_UPDATE_CMD in "car_control_node.h"]
    * 
    * In MANUAL mode, the commands depends on :
    * - requestedThrottle, reverse, requestedSteerAngle [from joystick orders]
    * - currentAngle [from motors feedback]
    */
    void updateCmd(){

        auto motorsOrder = interfaces::msg::MotorsOrder();

        if (!start){    //Car stopped
            leftRearPwmCmd = STOP;
            rightRearPwmCmd = STOP;
            steeringPwmCmd = STOP;


        }else{ //Car started

            //Manual Mode
            if (mode==0){
                
                manualPropulsionCmd(requestedThrottle, reverse, leftRearPwmCmd,rightRearPwmCmd);
                steeringCmd(requestedSteerAngle, currentAngle, steeringPwmCmd);


            //Autonomous Mode
            } else if (mode==1){

                if (requestedWheelsSpeedRPM == 0.0){
                    leftRearPwmCmd = STOP;
                    rightRearPwmCmd = STOP;
                    if (stopSteer)
                        steeringPwmCmd = STOP;
                    else
                        steeringCmd(autonomousSteerAngle, currentAngle, steeringPwmCmd);
                }
                else{
                    //Speed control on the left wheel speed in RPM
                    autoPropulsionCmd(requestedWheelsSpeedRPM, currentLeftSpeedRPM, leftRearPwmCmd, errorPreviousLeft, errorSumLeft, Kp, Ki, Kd); 
                    autoPropulsionCmd(requestedWheelsSpeedRPM, currentRightSpeedRPM, rightRearPwmCmd, errorPreviousRight, errorSumRight, Kp, Ki, Kd); 
                    
                    if (stopSteer)
                        steeringPwmCmd = STOP;
                    else
                        steeringCmd(autonomousSteerAngle, currentAngle, steeringPwmCmd);
                }
                
            }
        }


        //Send order to motors
        motorsOrder.left_rear_pwm = leftRearPwmCmd;
        motorsOrder.right_rear_pwm = rightRearPwmCmd;
        motorsOrder.steering_pwm = steeringPwmCmd;

        publisher_can_->publish(motorsOrder);
    }


    /* Start the steering calibration process :
    *
    * Publish a calibration request on the "/steering_calibration" topic
    */
    void startSteeringCalibration(){

        auto calibrationMsg = interfaces::msg::SteeringCalibration();
        calibrationMsg.request = true;

        RCLCPP_INFO(this->get_logger(), "Sending calibration request .....");
        publisher_steeringCalibration_->publish(calibrationMsg);
    }


    /* Function called by "steering_calibration" service
    * 1. Switch to calibration mode
    * 2. Call startSteeringCalibration function
    */
    void steeringCalibration([[maybe_unused]] std_srvs::srv::Empty::Request::SharedPtr req,
                            [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr res)
    {

        mode = 2;    //Switch to calibration mode
        RCLCPP_WARN(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
        startSteeringCalibration();
    }
    

    /* Manage steering calibration process [callback function]  :
    *
    * This function is called when a message is published on the "/steering_calibration" topic
    */
    void steeringCalibrationCallback (const interfaces::msg::SteeringCalibration & calibrationMsg){

        if (calibrationMsg.in_progress == true && calibrationMsg.user_need == false){
        RCLCPP_INFO(this->get_logger(), "Steering Calibration in progress, please wait ....");

        } else if (calibrationMsg.in_progress == true && calibrationMsg.user_need == true){
            RCLCPP_WARN(this->get_logger(), "Please use the buttons (L/R) to center the steering wheels.\nThen, press the blue button on the NucleoF103 to continue");
        
        } else if (calibrationMsg.status == 1){
            RCLCPP_INFO(this->get_logger(), "Steering calibration [SUCCESS]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        
        } else if (calibrationMsg.status == -1){
            RCLCPP_ERROR(this->get_logger(), "Steering calibration [FAILED]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        }
    
    }
    
    // ---- Private variables ----

    //General variables
    bool start;
    int mode;    //0 : Manual    1 : Auto    2 : Calibration

    
    //Motors feedback variables
    float currentAngle;
    float currentLeftSpeedRPM;
    float currentRightSpeedRPM;
    float currentCarSpeedMPS;
    float errorPreviousLeft;
    float errorPreviousRight;
    float errorSumLeft;
    float errorSumRight;


    //Manual Mode variables (with joystick control)
    bool reverse;
    float requestedThrottle;
    float requestedSteerAngle;

    //Auto Mode variables
    float requestedWheelsSpeedRPM;
    float requestedWheelsSpeedMPS;
    float autonomousSteerAngle ;
    bool stopSteer;

    //Wheels control variables
    uint8_t leftRearPwmCmd;
    uint8_t rightRearPwmCmd;
    uint8_t steeringPwmCmd;

    // PID coefficients
    float Kp;
    float Ki;
    float Kd;



    //Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steering_calibration_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_linear_speed_;
    rclcpp::Subscription<interfaces::msg::CmdVel>::SharedPtr subscription_consign_speed_;
    rclcpp::Subscription<interfaces::msg::Pid>::SharedPtr subscription_pid_;
    rclcpp::Subscription<interfaces::msg::CmdSteer>::SharedPtr subscription_consign_steer_;


    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Steering calibration Service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_calibration_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_control>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}