#include "robot/robot_controller.h"

/**
 * @brief The main node controlling the robot
*/
Robot_controller::Robot_controller():Node(NODE_NAME)
{
    /*
        Subscribe to the encoders topic
    */
    _encoders_subscription = this->create_subscription<std_msgs::msg::Float32>(
        ENCODERS_TOPIC, 
        ENCODERS_QUEUE_DEPTH,
        std::bind(&Robot_controller::encoders_callback, this, std::placeholders::_1));

    /*
        Subscribe to the position topic
    */
    _position_subscription = this->create_subscription<geometry_msgs::msg::Point>(
        POSITION_TOPIC,
        POSITION_QUEUE_DEPTH,
        std::bind(&Robot_controller::position_callback, this, std::placeholders::_1));

    init_cmd_vel_publisher();
}

/**
 * @brief Stores the last encoder value
*/
void Robot_controller::encoders_callback(const std_msgs::msg::Float32::UniquePtr &msg)
{

    RCLCPP_INFO(this->get_logger(), "robot_controller encoders_callback function - I heard %f", msg->data);
    _encoder_value = msg->data;
}

/**
 * @brief Stores the coordinates of the point_msg
*/
void Robot_controller::position_callback(const geometry_msgs::msg::Point::UniquePtr &point_msg)
{
    /*
        Get the coordinates stored in point_msg and store them
    */
    double x = point_msg->x;
    double y = point_msg->y;
    double z = point_msg->z;
    RCLCPP_INFO(this->get_logger(), "robot_controller position_callback function - I heard x = %f, y = %f, z = %f", x, y, z);
    _x = x;
    _y = y;
    _z = z;
}


/**
 * @brief Initialize a publisher in the /cmd_vel topic
*/
void Robot_controller::init_cmd_vel_publisher()
{
    using namespace std::chrono_literals;

    _cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC, CMD_VEL_QUEUE_DEPTH);
    auto timer_callback = [this]() -> void
    {
        // Create a message of type Twist
        geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
        message.linear.x = 0.5f;
        message.angular.z = 0.1f;

        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %f and angular_z = %f", message.linear.x, message.angular.z);

        // Publish the message in the _topic_name topic
        this->_cmd_vel_publisher->publish(message);
    };
    _timer = this->create_wall_timer(500ms, timer_callback);

}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot_controller>());
    rclcpp::shutdown();
    return 0;
}

