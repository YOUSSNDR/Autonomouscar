
#include "robot/encoders_publisher.h"

/**
 * @brief Code inspired from the creating a publisher/subscriber tutorial on the ROS documentation
*/

Encoders_publisher::Encoders_publisher(const Encoders &encoder) 
                                    : Node(NODE_NAME),
                                     _encoder(encoder)
{
    using namespace std::chrono_literals;

    _publisher = this->create_publisher<std_msgs::msg::Float32>(TOPIC_NAME, QUEUE_DEPTH);
    auto timer_callback = [this]() -> void
    {
        // Create a message of type float32
        std_msgs::msg::Float32 message = std_msgs::msg::Float32();
        // Retrieve the rps value from encoders channel A. Only for testing
        unsigned int rps = this->_encoder.get_rps_A();
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
    test_encoder.set_rps_A(34);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Encoders_publisher>(test_encoder));
    rclcpp::shutdown();
    std::cout << "lol" << std::endl;
    return 0;
}
