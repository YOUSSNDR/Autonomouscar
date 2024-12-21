#ifndef MOTOR_SUBSCRIBER_H
#define MOTOR_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"


// Library for gpio control
// pigpio must be installed
// Documentation : https://abyz.me.uk/rpi/pigpio/pdif2.html#set_PWM_dutycycle
// Comes with the set_PWM_dutycycle(pi, gpio, duty_cycle) function;
#include <pigpiod_if2.h> 
#include <vector>

#define NODE_NAME "Motor_subscriber"
#define TOPIC_NAME "/cmd_vel"
#define QUEUE_DEPTH 10


/**
 * @brief Subscriber to the cmd_vel topic
 * Can change the motors_velocity
*/

struct Motor_subscriber : public rclcpp::Node
{
    public:
        Motor_subscriber();
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmdvel_subscription;
        void init_params();
        void set_motor_duty_cycle(const unsigned int &motor_gpio, const unsigned int &duty_cycle) const;
        std::vector <unsigned int> convert_msg_to_duty_cycles(const geometry_msgs::msg::Twist::UniquePtr &msg) const;
       
        void set_left_motor_and_right_motor_duty_cycle(const geometry_msgs::msg::Twist::UniquePtr &msg) const;


        //pi daemon
        int _pi = -1; //Initalise to a negative number which should be changed to a positive number if we are able to connect to pigpiod

        /********************ROS_PARAMETERS**********************/

        /*
            Constants found in the yaml file
            Initiliazed to some default values, overriden by the this->get_parameter function called in the constructor
        */
        float _WHEEL_RADIUS = 0.0f; //in meters
        float _MAX_LINEAR_X = 0.7f; //in m/s
        float _MIN_LINEAR_X = 0.0f; //in m/s
        unsigned int _MAX_DUTY_CYCLE = 255U; //8-bit
        float _REAR_AXIS_LENGTH = 0.5f; //in m

        /*
            Motor gpios
        */

        //Left motor
        unsigned int _M1A_GPIO = 17U;
        unsigned int _M1B_GPIO = 18U;

        //Right motor
        unsigned int _M2A_GPIO = 23U;
        unsigned int _M2B_GPIO = 24U;

};

#endif