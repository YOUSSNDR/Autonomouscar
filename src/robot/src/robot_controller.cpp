#include "robot/robot_controller.h"
#include "rclcpp/rclcpp.hpp"

//Message type for encoders
#include "std_msgs/msg/float32.hpp"

//Message type for a 3D Point
#include "geometry_msgs/msg/point.hpp"

Robot_controller::Robot_controller(const std::string &node_name):Node(node_name)
{
    _encoders_subscription = this->create_subscription<std_msgs::msg::Float32>(
        ENCODERS_TOPIC, 
        ENCODERS_QUEUE_DEPTH,
        std::bind(&Robot_controller::encoders_callback, this, std::placeholders::_1));

    _position_subscription = this->create_subscription<geometry_msgs::msg::Point>(
        POSITION_TOPIC,
        POSITION_QUEUE_DEPTH,
        std::bind(&Robot_controller::position_callback, this, std::placeholders::_1));

}


void Robot_controller::encoders_callback(const std_msgs::msg::Float32::UniquePtr &msg)
{
    /*
        Stores the last encoder value
    */

    RCLCPP_INFO(this->get_logger(), "robot_controller encoders_callback function - I heard %f", msg->data);
    _encoder_value = msg->data;

}

void Robot_controller::position_callback(const geometry_msgs::msg::Point::UniquePtr &point_msg)
{
    double x = point_msg->x;
    double y = point_msg->y;
    double z = point_msg->z;
    RCLCPP_INFO(this->get_logger(), "robot_controller position_callback function - I heard x = %f, y = %f, z = %f", x, y, z);
    _x = x;
    _y = y;
    _z = z;
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot_controller>(NODE_NAME));
    rclcpp::shutdown();
    return 0;
}