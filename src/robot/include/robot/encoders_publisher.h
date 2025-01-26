#ifndef ENCODERS_PUBLISHER_H
#define ENCODERS_PUBLISHER_H
//Automatically needed
#include "rclcpp/rclcpp.hpp"
//Message type
#include "std_msgs/msg/float32.hpp"
#include <pigpiod_if2.h> 


//Needed for the "ms" literal
#include <chrono>

// The following headers are found in robot/include/robot/

#include "robot/encoders.h"

#define NODE_NAME "encoders_publisher"
#define TOPIC_NAME "encoders_topic"
#define QUEUE_DEPTH 10

#define LEFT_ENCODER_NAME "left_encoder"
#define RIGHT_ENCODER_NAME "right_encoder"

struct Encoders_publisher : public rclcpp::Node
{
    public:

        Encoders_publisher();
        ~Encoders_publisher();

    private:
        Encoders *_left_encoder;
        Encoders *_right_encoder;

        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _publisher;

        int _pi = -1;

        // -------------- ROS PARAMETERS -----------------
        void init_ros_params();
        unsigned int _A_CHANNEL_GPIO_M1 = 5;
        unsigned int _B_CHANNEL_GPIO_M1 = 6;
        unsigned int _A_CHANNEL_GPIO_M2 = 20;
        unsigned int _B_CHANNEL_GPIO_M2 = 21;
        
};

#endif