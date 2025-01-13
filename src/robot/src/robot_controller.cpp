#include "robot/robot_controller.h"

/**
 * @brief The main node controlling the robot
*/
Robot_controller::Robot_controller():Node(NODE_NAME)
{
    
    // ---------------------------- SUBSCRIBERS ------------------------------

    // Subscribe to the encoders topic

    _encoders_subscription = this->create_subscription<std_msgs::msg::Float32>(
        ENCODERS_TOPIC, 
        ENCODERS_QUEUE_DEPTH,
        std::bind(&Robot_controller::encoders_callback, this, std::placeholders::_1));

    
    // Subscribe to the position topic to read the tag location
    
    _position_subscription = this->create_subscription<geometry_msgs::msg::Point>(
        POSITION_TOPIC,
        POSITION_QUEUE_DEPTH,
        std::bind(&Robot_controller::position_callback, this, std::placeholders::_1));

    // ---------------------------- PUBLISHERS ----------------------------

    init_cmd_vel_publisher();

    // ---------------------------- ROS PARAMS ----------------------------
    
    init_ros_params();
}

/**
 * @brief Initialize the ros parameters found in the config.yaml file
 */

void Robot_controller::init_ros_params()
{

   this->declare_parameter<float>("MIN_LINEAR_X", 0.0f);
    _MIN_LINEAR_X = this->get_parameter("MIN_LINEAR_X").as_double();

    this->declare_parameter<float>("MAX_LINEAR_X", 0.7f);
    _MAX_LINEAR_X = this->get_parameter("MAX_LINEAR_X").as_double();

    this->declare_parameter<float>("MIN_TURN_RATE", 0.0f);
    _MIN_TURN_RATE = this->get_parameter("MIN_TURN_RATE").as_double();

    this->declare_parameter<float>("MAX_TURN_RATE", 0.3f);
    _MAX_TURN_RATE = this->get_parameter("MAX_TURN_RATE").as_double();

    this->declare_parameter<float>("SAFETY_DISTANCE", 0.7f);
    _SAFETY_DISTANCE = this->get_parameter("SAFETY_DISTANCE").as_double();

    this->declare_parameter<float>("ALPHA_POSITION_LOW_PASS", 0.995f);
    _ALPHA_POSITION_LOW_PASS = this->get_parameter("ALPHA_POSITION_LOW_PASS").as_double();

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
    
    // Get the coordinates stored in point_msg
    
    double x = point_msg->x;
    double y = point_msg->y;
    double z = point_msg->z;

    //RCLCPP_INFO(this->get_logger(), "robot_controller position_callback function - I heard : \n x = %f [m], \n y = %f [m],\n z = %f [m] \n", x, y, z);

    double distance_to_tag = compute_distance_to_tag();
    double angle_to_tag = compute_angle_to_tag();
    RCLCPP_INFO(this->get_logger(),"distance to tag =  = %f [m]", distance_to_tag);
    RCLCPP_INFO(this->get_logger(), "angle to tag : phi = %f [rad]", angle_to_tag);


    // Store the coordinates 
    _x_tag = _ALPHA_POSITION_LOW_PASS*_x_tag + (1-_ALPHA_POSITION_LOW_PASS)*x;
    _y_tag = _ALPHA_POSITION_LOW_PASS*_y_tag + (1-_ALPHA_POSITION_LOW_PASS)*y;
    _z_tag = z;

    RCLCPP_INFO(this->get_logger(), "position_callback function : \n x = %f [m], \n y = %f [m],\n z = %f [m]", _x_tag, _y_tag, z);

    turn_to_tag();

    if (!_turning)
    {
        go_towards_tag();
    }
}


/**
 * @brief Initialize a publisher in the /cmd_vel topic
*/
void Robot_controller::init_cmd_vel_publisher()
{
    //For the ms literal
    using namespace std::chrono_literals;

    //Initalize the publisher to send message of type geometry_msg::msg::Twist to control the motor angular velocity
    _cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC, CMD_VEL_QUEUE_DEPTH);

    //Callback function

    
    /*
    auto timer_callback = [this]() -> void
    {
        
        // Create a message of type Twist
        geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
        message.linear.x = 0.2f;
        message.angular.z = 0.0f;

        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %f and angular_z = %f", message.linear.x, message.angular.z);

        // Publish the message in the _topic_name topic
        
        this->_cmd_vel_publisher->publish(message);
    };
    */    

    //Publish the message every 500ms
    //_timer = this->create_wall_timer(500ms, timer_callback);

}


/**
 * @brief Compute the distance from the robot to the tag relative to the origin of <some frame to be determined> in [m]
 */
double Robot_controller::compute_distance_to_tag() const
{
    return sqrt(_x_tag*_x_tag + _y_tag*_y_tag);
}


/**
 * @brief Compute the angle between the robot and the tag in [rad]
 */
double Robot_controller::compute_angle_to_tag() const
{
    return atan2(_y_tag, _x_tag);
}


void Robot_controller::turn_to_tag()
{

    // Get the angle to the tag
    double angle_to_tag = compute_angle_to_tag(); // [rad]


    // Geometry message to publish turn rate (i.e. message.angular.z)
    // Turn rate is initialized to 0 by default;
    geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
    message.angular.z = 0.0;
    _turning = false;

    if (angle_to_tag > M_PI/4)
    {
        message.angular.z = _MAX_TURN_RATE;
        _turning = true;
    }

    else if(angle_to_tag < -M_PI/4)
    {
        message.angular.z = -_MAX_TURN_RATE;
        _turning = true;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing turn_rate = %f [rad/s]", message.angular.z);
    _cmd_vel_publisher->publish(message);
}


void Robot_controller::go_towards_tag() const
{
    double distance_to_tag = compute_distance_to_tag();
    geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    if (distance_to_tag >= _SAFETY_DISTANCE)
    {
        message.linear.x = 0.2;
    }
    _cmd_vel_publisher->publish(message);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot_controller>());
    rclcpp::shutdown();
    return 0;
}

