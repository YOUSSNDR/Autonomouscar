#ifndef ENCODERS_PUBLISHER_H
#define ENCODERS_PUBLISHER_H
//Automatically needed
#include "rclcpp/rclcpp.hpp"
//Message type
#include "std_msgs/msg/float32.hpp"


//Needed for the "ms" literal
#include <chrono>

/*
    The following headers are found in robot/include/robot/
*/

#include "robot/encoders.h"

#define NODE_NAME "encoders_publisher"
#define TOPIC_NAME "encoders_topic"
#define QUEUE_DEPTH 10

struct Encoders_publisher : public rclcpp::Node
{
    public:

        Encoders_publisher(const Encoders &encoder);

    private:
        const Encoders &_encoder;
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _publisher;
};

#endif