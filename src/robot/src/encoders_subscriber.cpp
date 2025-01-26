#include "robot/encoders_subscriber.h"


Encoders_subscriber::Encoders_subscriber(): Node(NODE_NAME)
{
    auto topic_callback =
    [this](std_msgs::msg::Float32::UniquePtr msg) -> void
    {
        //std::cout << "I heard " << msg << std::endl;
        RCLCPP_INFO(this->get_logger(), "I heard %f", msg->data);
    };
    _subscription = this->create_subscription<std_msgs::msg::Float32>(TOPIC_NAME, QUEUE_DEPTH, topic_callback);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Encoders_subscriber>());
    rclcpp::shutdown();
    return 0;
}
