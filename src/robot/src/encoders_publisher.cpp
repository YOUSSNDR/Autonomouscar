//Automatically needed
#include "rclcpp/rclcpp.hpp"

//Message type
#include "std_msgs/msg/float32.hpp"

//Needed for the "ms" literal
#include <chrono>

/*
    The following headers are found in robot/include/robot/
*/

#include "robot/encoders_publisher.h"
#include "robot/encoders.h"

/*
    Code inspired from the creating a publisher/subscriber tutorial on the ROS documentation
*/

Encoders_publisher::Encoders_publisher(const std::string &node_name,
                                        const std::string &topic_name,
                                        const Encoders &encoder,
                                        const unsigned int &queue_depth) 
                                    : Node(node_name),
                                     _topic_name(topic_name), 
                                     _encoder(encoder),
                                      _queue_depth(queue_depth)
{
    using namespace std::chrono_literals;

    _publisher = this->create_publisher<std_msgs::msg::Float32>(_topic_name, _queue_depth);
    auto timer_callback = [this]() -> void
    {
        // Create a message of type float32
        std_msgs::msg::Float32 message = std_msgs::msg::Float32();
        // Retrieve the rps value from encoders
        unsigned int rps = this->_encoder.get_rps();
        message.data = rps;

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f' - encoders_publisher", message.data);

        // Publish the message in the _topic_name topic
        this->_publisher->publish(message);
    };
    _timer = this->create_wall_timer(500ms, timer_callback);
};

int main(int argc, char* argv[])
{
    Encoders test_encoder;
    test_encoder.set_rps(34);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Encoders_publisher>(NODE_NAME, 
                                                    TOPIC_NAME, 
                                                    test_encoder, 
                                                    QUEUE_DEPTH));
    rclcpp::shutdown();
    std::cout << "lol" << std::endl;
    return 0;
}