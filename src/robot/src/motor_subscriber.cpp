#include "robot/motor_subscriber.h"


Motor_subscriber* Motor_subscriber::_instance = nullptr;

void Motor_subscriber::signalHandler(int signal)
{
    if (signal){} //Line is just here to delete the warning after colcon build
    if(_instance)
    {
        RCLCPP_WARN(_instance->get_logger(), "Motors have been stopped");
        _instance->set_motor_duty_cycle(_instance->_M1A_GPIO, 0);
        _instance->set_motor_duty_cycle(_instance->_M2B_GPIO, 0);

        _instance->set_motor_duty_cycle(_instance->_M1B_GPIO, 0);
        _instance->set_motor_duty_cycle(_instance->_M2A_GPIO, 0);
    }
    rclcpp::shutdown();
}

Motor_subscriber::Motor_subscriber() 
: Node(NODE_NAME)
{
    init_params();
//
    /*
        Directly initializing the GPIO Pins here
        For some reason it does not work if that piece of code is called from another struct. It outputs this error :
        /home/youss/Documents/Projet/Autonomouscar/ros_ws/install/robot/lib/robot/motor_subscriber: 
        error while loading shared libraries: librcl_interfaces__rosidl_typesupport_cpp.so: cannot open shared object file: No such file or directory
        [ros2run]: Process exited with failure 127
        on rpi4.init function ????
        IT IS BEACAUSE OF "SUDO" IN THE MAKE_RPI4_WORK.SH SCRIPT BUT WE NEED SUDO TO USE THE INITIALIZE THE GPIOS
        SUDO RESETS THE ENVIRONMENT SO NO ROS ENVIRONMENT
        NEEDS TO BE ROOT TO WORK 
        std::cout << "Using pigpio version : " << gpioVersion() << "\n";

        //Disable builtin pigpio signal handling
        int cfg = gpioCfgGetInternals();
        cfg |= PI_CFG_NOSIGHANDLER;
        gpioCfgSetInternals(cfg);

        //Initialize pigpio library
        std::cout << "Initializing pigpio library" << "\n";
        
        
        int gpio_result = 0;
        gpio_result = gpioInitialise();
        if(gpio_result == PI_INIT_FAILED)
        {
            std::cout << "PI_INIT_FAILED" << "\n";
        }
        else
        {
            std::cout << "PI INIT SUCCESS" << "\n";
        }
    
    */

   //Instead, we are using pigpiod_if2 which seems to work

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
    set_mode(_pi, _M1B_GPIO, PI_OUTPUT);
    
    set_mode(_pi, _M2A_GPIO, PI_OUTPUT);
    //set_mode(_pi, _M2B_GPIO, PI_OUTPUT);

    _cmdvel_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
        TOPIC_NAME, 
        QUEUE_DEPTH,
        std::bind(&Motor_subscriber::set_left_motor_and_right_motor_duty_cycle, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "WHEEL RADIUS = %f", _WHEEL_RADIUS);

}


/**
 * @brief Initialize the ros parameters found in the config.yaml file
 * You must declare the parameter with this->declare_parameter<param_type>("param_name_in_yaml_file", default_value).
 * Then you can use this->get_parameter("param_name_in_yaml_file", value)
 * If you do not declare the parameter it will not work.
 * Actually you can avoid declaring parameters by doing some magic, but I am not doing it because the following code is simple enough.
 * Beware of typos while copy/pasting code.
*/
void Motor_subscriber::init_params()
{
    
    //  There is no "as_float" function, use as_double for the floats


    this->declare_parameter<float>("WHEEL_RADIUS", 0.034f);
    _WHEEL_RADIUS = this->get_parameter("WHEEL_RADIUS").as_double();

    this->declare_parameter<float>("MIN_LINEAR_X", 0.0f);
    _MIN_LINEAR_X = this->get_parameter("MIN_LINEAR_X").as_double();

    this->declare_parameter<float>("MAX_LINEAR_X", 0.7f);
    _MAX_LINEAR_X = this->get_parameter("MAX_LINEAR_X").as_double();

    this->declare_parameter<int>("MAX_DUTY_CYCLE", 255);
    _MAX_DUTY_CYCLE = this->get_parameter("MAX_DUTY_CYCLE").as_int();

    this->declare_parameter<float>("REAR_AXIS_LENGTH", 0.5);
    _REAR_AXIS_LENGTH = this->get_parameter("REAR_AXIS_LENGTH").as_double();

    
    //    Motors gpio

    this->declare_parameter<int>("M1A_GPIO", 17);
    _M1A_GPIO = this->get_parameter("M1A_GPIO").as_int();

    this->declare_parameter<int>("M1B_GPIO", 18);
    _M1B_GPIO = this->get_parameter("M1B_GPIO").as_int();
    
    this->declare_parameter<int>("M2A_GPIO", 23);
    _M2A_GPIO = this->get_parameter("M2A_GPIO").as_int();

    this->declare_parameter<int>("M2B_GPIO", 24);
    _M2B_GPIO = this->get_parameter("M2B_GPIO").as_int();

    RCLCPP_INFO(this->get_logger(), "ROS PARAMS INIT SUCCESS IN MOTOR_SUBSCRIBER NODE");

}


/**
 * @brief Take a geometry twist message containing the asked linear velocity about the x-axis and angular velocity about the z-axis
 * Convert the values into two duty_cycle numbers between 0 and 255: one for the left_motor and one for the right motor
 * 
 * Using V = omega*R 
     * where : 
     * R is the wheel radius in meters, 
     * omega is the angular velocity of the motor in rad/s
     * V is the linear velocity of the robot along the x axis, in m/s
 * 
 * Solving for alpha_right and alpha_left which are the duty_cycle values divided by _MAX_DUTY_CYCLE for the right and left motor :
 * V_x = V_max(alpha_right + alpha_left)/2
 * omega_z = V_max(alpha_right - alpha_left)/L 
 * where :
 * V_x is msg->linear.x in m/s
 * omega_z is msg->angular.z in rad/s
 * L is the length of the rear axis in meters i.e. _REAR_AXIS_LENGTH
 * We get :
 * alpha_right = (L*omega_z)/(2*V_max) + V_x/V_max
 * alpha_left = V_x/V_max - (L*omega_z)/(2*V_max)
 * Since alpha_right and alpha_left are divided by _MAX_DUTY_CYCLE, we have to return :
 * _MAX_DUTY_CYCLE*alpha_right and _MAX_DUTY_CYCLE*alpha_left and convert them to int
 * 
 * @param msg
 * @return duty_cycles: a vector containing left_duty_cycle and right_duty_cycle
*/
std::vector <unsigned int> Motor_subscriber::convert_msg_to_duty_cycles(const geometry_msgs::msg::Twist::UniquePtr &msg) const
{
    /*
        TODO : change the function so that the robot can turn
        For now, it can only go forward in one direction
        Also change the set_motor_duty_cycle functions
    */

    /**

    */
    float linear_x = msg->linear.x; //in m/s
    float angular_z = msg->angular.z; //the angular velocity of the robot about the z-axis, i.e the turn rate, in rad/s

    //Saturate the value of linear velocity
    if (linear_x > _MAX_LINEAR_X)
    {
        linear_x = _MAX_LINEAR_X;
    }
    else if(linear_x < _MIN_LINEAR_X)
    {
        linear_x = _MIN_LINEAR_X;
    }


    unsigned int left_duty_cycle = 0U; // a number between 0 and 255 for the left motor
    unsigned int right_duty_cycle = 0U; // a number between 0 and 255 for the right motor
    double alpha_left = 0.0f;
    double alpha_right = 0.0f;


    alpha_left = (linear_x - _REAR_AXIS_LENGTH*angular_z/2)/_MAX_LINEAR_X;
    alpha_right = (linear_x + _REAR_AXIS_LENGTH*angular_z/2)/_MAX_LINEAR_X; 

    /**
     * Using V = V_max*(duty_cycle/_MAX_DUTY_CYCLE) 
     * where :
     * 0 <= duty_cycle <= _MAX_DUTY_CYCLE
     * MAX_DUTY_CYCLE = 255 on a 8-bit PWM
    */
    left_duty_cycle = _MAX_DUTY_CYCLE*alpha_left + _LEFT_PWM_OFFSET;
    right_duty_cycle = _MAX_DUTY_CYCLE*alpha_right + _PWM_OFFSET;

    //Debugging
    /*
    
    RCLCPP_INFO(this->get_logger(), "I heard : %f linear.x and %f angular.z", linear_x, angular_z);
    RCLCPP_INFO(this->get_logger(), "Converted left_duty_cycle : %d", left_duty_cycle);
    RCLCPP_INFO(this->get_logger(), "Converted right_duty_cycle : %d", right_duty_cycle);
    RCLCPP_INFO(this->get_logger(), "Converted left_duty_cycle : %d", left_duty_cycle);
    RCLCPP_INFO(this->get_logger(), "Converted right_duty_cycle : %d", right_duty_cycle);
    */

    //Store the values in a vector of unsigned ints and return it
    std::vector<unsigned int> duty_cycles = {left_duty_cycle, right_duty_cycle};
    
    return duty_cycles;
}


/**
 * @brief set a motor to a PWM value
 * @param motor_gpio the motor gpio of the rpi4
 * @param duty_cycle a number between 0 and 255 for an 8-bit PWM
*/
void Motor_subscriber::set_motor_duty_cycle(const unsigned int &motor_gpio, 
                                            const unsigned int &duty_cycle) const                    
{
    set_PWM_dutycycle(_pi, motor_gpio, duty_cycle);
}

void Motor_subscriber::set_left_motor_and_right_motor_duty_cycle(const geometry_msgs::msg::Twist::UniquePtr &msg) const
{
    std::vector<unsigned int> duty_cycles = convert_msg_to_duty_cycles(msg);
    unsigned int left_motor_duty_cycle = duty_cycles[0];
    unsigned int right_motor_duty_cycle = duty_cycles[1];

    //Use _M1B_GPIO to control the right_motor
    set_motor_duty_cycle(_M1A_GPIO, right_motor_duty_cycle);

    //Use _M2A_GPIO to control the left motor
    set_motor_duty_cycle(_M2B_GPIO, left_motor_duty_cycle);
}

/**
 * @brief Static method
 * Set  _instance to motor_sub_pointer
 * Used to link the signalHandler to the node
 * @param motor_sub_pointer
 */
void Motor_subscriber::set_static_instance_pointer(Motor_subscriber* motor_sub_pointer)
{
    _instance = motor_sub_pointer;
}


int main(int argc, char *argv[])
{    
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<Motor_subscriber>();
    Motor_subscriber::set_static_instance_pointer(my_node.get());
    signal(SIGINT, Motor_subscriber::signalHandler);
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}