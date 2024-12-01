#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "robot/encoders_subscriber.h"




Encoders_subscriber::Encoders_subscriber(const std::string &node_name,
                            const std::string &topic_name,
                            const unsigned int &queue_depth):
                            Node(node_name),
                            _topic_name(topic_name),
                            _queue_depth(queue_depth)
{
    auto topic_callback =
    [this](std_msgs::msg::Float32::UniquePtr msg) -> void
    {
        //std::cout << "I heard " << msg << std::endl;
        RCLCPP_INFO(this->get_logger(), "I heard %f", msg->data);
    };
    _subscription = this->create_subscription<std_msgs::msg::Float32>(_topic_name, _queue_depth, topic_callback);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Encoders_subscriber>(NODE_NAME, 
                                                        TOPIC_NAME, 
                                                        QUEUE_DEPTH));
    rclcpp::shutdown();
    return 0;
}
