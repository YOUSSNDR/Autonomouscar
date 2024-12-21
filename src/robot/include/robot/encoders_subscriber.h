#ifndef ENCODERS_SUBSCRIBER_H
#define ENCODERS_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


#define NODE_NAME "encoders_subscriber"
#define TOPIC_NAME "encoders_topic"
#define QUEUE_DEPTH 10


struct Encoders_subscriber : public rclcpp::Node
{
    public:

        Encoders_subscriber();

    private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _subscription;
};

#endif