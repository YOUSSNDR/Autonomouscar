
#include "robot/encoders_publisher.h"

/**
 * @brief Code inspired from the creating a publisher/subscriber tutorial on the ROS documentation
*/


Encoders_publisher::Encoders_publisher() : Node(NODE_NAME)
{
    using namespace std::chrono_literals;

    //init raspberry pi
    _pi = pigpio_start(NULL, NULL);  // Connect to localhost with default port
    if (_pi < 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to connect to pigpiod! Please run sudo pigpiod first and retry ");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Connected to pigpiod in motor_subscriber node");
    }


    init_ros_params();

    //set_mode(_pi, _M1A_GPIO, PI_OUTPUT);
    //set_mode(_pi, _A_CHANNEL_GPIO_M1, PI_INPUT);
    //set_mode(_pi, _B_CHANNEL_GPIO_M1, PI_INPUT);

    //set_mode(_pi, _A_CHANNEL_GPIO_M2, PI_INPUT);
    //set_mode(_pi, _B_CHANNEL_GPIO_M2, PI_INPUT);

    //Initialize left encoder and right encoder

    _left_encoder = new Encoders(_A_CHANNEL_GPIO_M1,
                                _B_CHANNEL_GPIO_M1,
                                LEFT_ENCODER_NAME);
    
    _right_encoder = new Encoders(_A_CHANNEL_GPIO_M2, 
                                _B_CHANNEL_GPIO_M2, 
                                RIGHT_ENCODER_NAME);

    _publisher = this->create_publisher<std_msgs::msg::Float32>(TOPIC_NAME, QUEUE_DEPTH);
    auto timer_callback = [this]() -> void
    {
        // Create a message of type float32
        std_msgs::msg::Float32 left_encoder_message = std_msgs::msg::Float32();
        std_msgs::msg::Float32 right_encoder_message = std_msgs::msg::Float32();

        // Retrieve the rps value from encoders channel A. Only for testing
        unsigned int left_encoder_rps_channel_A = this->_left_encoder->get_rps_A();
        unsigned int left_encoder_rps_channel_B = this->_left_encoder->get_rps_B();

        //best estimation of the rps value is the mean between rpsA and rpsB (least square method)
        float left_encoder_rps = (left_encoder_rps_channel_A + left_encoder_rps_channel_B)/2;
        left_encoder_message.data = left_encoder_rps;
        this->_publisher->publish(left_encoder_message);
        RCLCPP_INFO(this->get_logger(), "Publishing: left encoder %f", left_encoder_message.data);

        unsigned int right_encoder_rps_channel_A = this->_right_encoder->get_rps_A();
        unsigned int right_encoder_rps_channel_B = this->_right_encoder->get_rps_B();

        //Publish the left encoder message

        //best estimation of the rps value is the mean between rpsA and rpsB (least square method)
        float right_encoder_rps = (right_encoder_rps_channel_A + right_encoder_rps_channel_B)/2;
        right_encoder_message.data = right_encoder_rps;
        this->_publisher->publish(right_encoder_message);
        RCLCPP_INFO(this->get_logger(), "Publishing: right encoder %f", right_encoder_message.data);


        // Publish the message in the _topic_name topic
    };
    _timer = this->create_wall_timer(500ms, timer_callback);
};

void Encoders_publisher::init_ros_params()
{
    this->declare_parameter<int>("A_CHANNEL_GPIO_M1", 5);
    _A_CHANNEL_GPIO_M1 = this->get_parameter("A_CHANNEL_GPIO_M1").as_int();

    this->declare_parameter<int>("B_CHANNEL_GPIO_M1", 6);
    _B_CHANNEL_GPIO_M1 = this->get_parameter("B_CHANNEL_GPIO_M1").as_int();

    this->declare_parameter<int>("A_CHANNEL_GPIO_M2", 20);
    _A_CHANNEL_GPIO_M2 = this->get_parameter("A_CHANNEL_GPIO_M2").as_int();

    this->declare_parameter<int>("B_CHANNEL_GPIO_M2", 21);
    _B_CHANNEL_GPIO_M2 = this->get_parameter("B_CHANNEL_GPIO_M2").as_int();

    RCLCPP_INFO(this->get_logger(), "ROS PARAMS SUCCESSFULLY INITIALIZED");
}

Encoders_publisher::~Encoders_publisher()
{
    delete _left_encoder;
    delete _right_encoder;
    std::cout << "Encoders_publisher destroyed" << "\n";
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Encoders_publisher>());
    rclcpp::shutdown();
    return 0;
}
