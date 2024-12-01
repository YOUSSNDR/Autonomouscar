#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#ifndef ENCODERS_SUBSCRIBER_H
#define ENCODERS_SUBSCRIBER_H

#define NODE_NAME "encoders_subscriber"
#define TOPIC_NAME "encoders_topic"
#define QUEUE_DEPTH 10


struct Encoders_subscriber : public rclcpp::Node
{
    public:

        Encoders_subscriber(const std::string &node_name,
                            const std::string &topic_name,
                            const unsigned int &queue_depth);

    private:

        std::string _node_name = "default node name";
        std::string _topic_name = "default topic name";
        const unsigned int _queue_depth = 10;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _subscription;
};

#endif