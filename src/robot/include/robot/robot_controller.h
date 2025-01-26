#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"


// Message type for encoders

#include "std_msgs/msg/float32.hpp"


// Message type for a 3D Point
// Used to locate the robot

#include "geometry_msgs/msg/point.hpp"


// Message type to control the motors

#include "geometry_msgs/msg/twist.hpp"


// For the ms literal
#include <chrono>

// For sqrt
#include <math.h>

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
        
        //  ------ ROS PARAMETERS

        float _MIN_LINEAR_X = 0.0f; // m/s
        float _MAX_LINEAR_X = 0.7f; // m/s
        float _MIN_TURN_RATE = 0.0f; // rad/s
        float _MAX_TURN_RATE = 0.3f; // rad/s
        float _SAFETY_DISTANCE = 0.7f; // m
        float _ALPHA_POSITION_LOW_PASS = 0.995f; // unitless

        void init_ros_params();


        // Subscribing to the encoders topic
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _encoders_subscription;
        float _encoder_value = 0.0f;
        void encoders_callback(const std_msgs::msg::Float32::UniquePtr &msg);

        
        // Subscribing to the 2D position topic
        
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _position_subscription;

        // Position of the tag

        double _x_tag = 0.0f;
        double _y_tag = 0.0f;
        double _z_tag = 0.0f;

        void position_callback(const geometry_msgs::msg::Point::UniquePtr &point_msg);

        
        // Publish to the /cmd_vel topic
        
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;
        void init_cmd_vel_publisher();

        // Compute the distance between the robot and the tag point
        double _distance_to_tag = 0.0f;
        double compute_distance_to_tag() const;

        // Compute angle between the robot and the tag
        double compute_angle_to_tag() const;
    
        bool _turning = false;
        void turn_to_tag();
        void go_towards_tag() const;
};

#endif