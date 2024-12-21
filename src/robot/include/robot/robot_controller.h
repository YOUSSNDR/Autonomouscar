#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"

/*
    Message type for encoders
*/
#include "std_msgs/msg/float32.hpp"

/*
    Message type for a 3D Point
    Used to locate the robot
*/
#include "geometry_msgs/msg/point.hpp"

/*
    Message type to control the motors
*/
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>



#define NODE_NAME "Robot_controller"


#define ENCODERS_TOPIC "encoders_topic"
#define ENCODERS_QUEUE_DEPTH 10

#define POSITION_TOPIC "position_topic"
#define POSITION_QUEUE_DEPTH 10

#define CMD_VEL_TOPIC "/cmd_vel"
#define CMD_VEL_QUEUE_DEPTH 10

struct Robot_controller : public rclcpp::Node{
    public:
        Robot_controller();

    private:
        /*
            Subbing to the encoders topic
        */
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _encoders_subscription;
        float _encoder_value = 0.0f;
        void encoders_callback(const std_msgs::msg::Float32::UniquePtr &msg);

        /*
            Subbing to the 2D position topic
        */
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _position_subscription;
        double _x = 0.0f;
        double _y = 0.0f;
        double _z = 0.0f;
        void position_callback(const geometry_msgs::msg::Point::UniquePtr &point_msg);

        /*
            Publish to the /cmd_vel topic
        */
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;
        void init_cmd_vel_publisher();

        
};

#endif