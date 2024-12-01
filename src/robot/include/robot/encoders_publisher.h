#include "rclcpp/rclcpp.hpp"
#include "encoders.h"
#include "std_msgs/msg/float32.hpp"

#ifndef ENCODERS_PUBLISHER_H
#define ENCODERS_PUBLISHER_H

#define NODE_NAME "encoders_publisher"
#define TOPIC_NAME "encoders_topic"
#define QUEUE_DEPTH 10

struct Encoders_publisher : public rclcpp::Node
{
    public:

        Encoders_publisher(const std::string &node_name,
                            const std::string &topic_name,
                            const Encoders &encoder,
                            const unsigned int &queue_depth);

    private:

        std::string _node_name = "default node name";
        std::string _topic_name = "default topic name";
        const Encoders &_encoder;
        const unsigned int _queue_depth = 10;
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _publisher;
};

#endif