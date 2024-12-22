
#include "robot/encoders_publisher.h"

/**
 * @brief Code inspired from the creating a publisher/subscriber tutorial on the ROS documentation
*/



Encoders_publisher::Encoders_publisher(const Encoders &encoder) 
                                    : Node(NODE_NAME),
                                     _encoder(encoder)
{
    using namespace std::chrono_literals;

    init_ros_params();

    _pi = pigpio_start(NULL, NULL);  // Connect to localhost with default port

    if (_pi < 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to connect to pigpiod! Please run sudo pigpiod first and retry ");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Connected to pigpiod in motor_subscriber node");
    }

    //set_mode(_pi, _M1A_GPIO, PI_OUTPUT);
    set_mode(_pi, _A_CHANNEL_GPIO_M1, PI_INPUT);
    
    set_mode(_pi, _B_CHANNEL_GPIO_M1, PI_INPUT);


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

void Encoders_publisher::init_ros_params()
{
    this->declare_parameter<int>("A_CHANNEL_GPIO_M1", 5);
    _A_CHANNEL_GPIO_M1 = this->get_parameter("A_CHANNEL_GPIO_M1").as_int();

    this->declare_parameter<int>("B_CHANNEL_GPIO_M1", 6);
    _B_CHANNEL_GPIO_M1 = this->get_parameter("B_CHANNEL_GPIO_M1").as_int();

    RCLCPP_INFO(this->get_logger(), "ROS PARAMS SUCCESSFULLY INITIALIZED");
}

int main(int argc, char* argv[])
{
    Encoders encoder_A(M1A_CHANNEL,
                        M1B_CHANNEL,
                        LEFT_ENCODER_NAME);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Encoders_publisher>(encoder_A));
    rclcpp::shutdown();
    std::cout << "lol" << std::endl;
    return 0;
}
