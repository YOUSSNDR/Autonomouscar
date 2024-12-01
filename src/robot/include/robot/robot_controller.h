#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#define NODE_NAME "Robot_controller_name"


#define ENCODERS_TOPIC "encoders_topic"
#define ENCODERS_QUEUE_DEPTH 10

#define POSITION_TOPIC "position_topic"
#define POSITION_QUEUE_DEPTH 10

struct Robot_controller : public rclcpp::Node{
    public:
        Robot_controller(const std::string &node_name);

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

        
};

#endif