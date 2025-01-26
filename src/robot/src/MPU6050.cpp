// Link to the register list of the MPU6050
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
//
#include "../include/robot/MPU6050.h"

//TODO, not urgent : create a file specifically made for sensor calibration

//For calibration
int64_t _recorded_values = 0;
int64_t _number_of_values = 0;

/**
 * @brief The function handles what to do when we are pressing CTRL-C to stop the node
 * rclcpp::shutdown is necessary to stop the node, otherwise you will not able to stop it.
 * Mainly used for calibration purposes
 */
void signalHandler(int signal)
{
    //std::cout << "Received signal : " << signal << ". Shutting down" << std::endl;
    if (signal){} //Line is just here to delete the warning after colcon build
    //std::cout << ". Shutting down" << std::endl;
    //std::cout << _number_of_values << std::endl;
    //std::cout << _recorded_values/_number_of_values << std::endl;
    rclcpp::shutdown();

}

/**
 * @brief Constructor for the node
 */
MPU6050::MPU6050(): Node(NODE_NAME)
{
    init_i2c();
    init_ros_params();
    init_cmd_vel_publisher();

    //Calibration
    signal(SIGINT, signalHandler);

}

/**
 * @brief Initialize I2C for the MPU6050
 * MPU6050 address is set to be 0x68 by default according to the documentation
 * Libraries needed are located in the MPU6050.h file
 * We need the i2c/smbus.h library and the linux/i2c-dev.h library
 */
void MPU6050::init_i2c()
{
    char filename[20];
    snprintf(filename, _MAX_BUFFER_SIZE, "/dev/i2c-%d", _i2c_port_number);

    _file = open(filename, O_RDWR);
    if (_file < 0) 
    {
        std::cerr << "Error opening i2c file" << "\n";
        exit(1);
    }

    // Link : https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
    // Page 15
    // Address is set to 1101000 by default, which gives 0x68 in hexadecimal

    //Set the address of the I2C slave to mpu6050_address, if failure then exit
    if (ioctl(_file, I2C_SLAVE, MPU6050_ADDRESS) < 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Error setting MPU6050 i2c address");
        //std::cerr << "Error setting i2c address" << "\n";
        exit(1);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "I2C_SLAVE ADDRESS set to 0x%x", MPU6050_ADDRESS);
        //std::cout << "I2C_SLAVE ADDRESS set to 0x" << std::hex << MPU6050_ADDRESS << "\n";
    }

    //Wake up the device
    i2c_smbus_write_byte_data(_file, POWER_MANAGEMENT_1_REGISTER, WAKE_UP_VALUE);

    RCLCPP_INFO(this->get_logger(), "MPU6050 initialized");
}

/**
 * @brief Initialize the ros paramters in the config.yaml file
 */
void MPU6050::init_ros_params()
{
    this->declare_parameter<double>("PUBLISH_RATE", 1.0);
    _PUBLISH_RATE = this->get_parameter("PUBLISH_RATE").as_double();


    RCLCPP_INFO(this->get_logger(), "ROS parameters initialized for the mpu_publisher node");
}
/**
 * @brief Creates the callback function for the publisher
 */
void MPU6050::init_cmd_vel_publisher()
{   
    
    auto duration = std::chrono::duration<float>(1/_PUBLISH_RATE); //Publish at _PUBLISH_RATE [Hz] i.e. every 1/_PUBLISH_RATE [seconds]
    _cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(TOPIC_NAME, QUEUE_DEPTH);
    auto timer_callback = [this]()-> void
    {
        //calibrate_acc(GYRO_Z_OUT_HIGH_REGISTER, GYRO_Z_OUT_LOW_REGISTER);
        compute_yaw_rate();
    };
    _timer = this->create_wall_timer(duration, timer_callback);
    
}

/**
 * @brief Read the byte stored at register_address
 * If the result is supposed to be signed, we handle the "signess" in other functions
 * @param register_address
 */
__u8 MPU6050::read_register(const int &register_address) const
{

    //Declare an unsigned byte that will store the value read in the register
    __u8 result;
    result = i2c_smbus_read_byte_data(_file, register_address);

    // Uncomment for easier debugging
    // RCLCPP_INFO(this->get_logger(), "Read the value 0x%x at register 0x%x", result, register_address);
    return result;
}

void MPU6050::write_to_file(__s8 byte) const
{
    //Fixed size for buffer array to comply with ISO C++ VLA
    const unsigned int size_of_buffer = sizeof(byte);
    char buf[size_of_buffer];

    buf[0] = byte;
    if (write(_file, buf, size_of_buffer) != size_of_buffer)
    {
        std::cerr << "Size of buffer not good" << "\n";
    }   
}


/**
 * @brief Compute the yaw rate by combining the registers GYRO_Z_OUT_HIGH_REGISTER and GYRO_Z_OUT_LOW_REGISTER
 * Returned value is in [rad/s]
 */
float MPU6050::compute_yaw_rate() const
{
    __u8 high_byte = read_register(GYRO_Z_OUT_HIGH_REGISTER);
    __u8 low_byte = read_register(GYRO_Z_OUT_LOW_REGISTER);

    //Shift to the left the high byte by 8 bits to create a 16-bit word, then add low_byte at the end

    __s16 combined_bytes = (high_byte << 8) | low_byte;

    //Minus sign because the sensor is positionned upside down on the robot
    float yaw_rate_value_deg_per_sec = -(combined_bytes - YAW_OFFSET) / GYRO_CONFIG_0_SCALE; //yaw_rate in °/s
    float yaw_rate_value_rad_per_sec = yaw_rate_value_deg_per_sec * M_PI/180; //taw_rate in rad/s
    RCLCPP_INFO(this->get_logger(), "Yaw rate value : %f rad/s", yaw_rate_value_rad_per_sec);
    return yaw_rate_value_rad_per_sec;
}


/**
 * @brief Compute acceleration along x-axis
 * The sensor must be calibrated beforehand
 */
float MPU6050::compute_acc_x() const
{
    __u8 high_byte = read_register(ACC_X_OUT_HIGH_REGISTER);
    __u8 low_byte = read_register(ACC_X_OUT_LOW_REGISTER);

    //Shift to the left the high byte by 8 bits to create a 16-bit word, then add low_byte at the end

    __s16 combined_bytes = (high_byte << 8) | low_byte;

    float acc_x_in_g = (combined_bytes - ACC_X_OFFSET) / (ACC_CONFIG_0_SCALE); // in g
    float acc_x = acc_x_in_g * 9.81; //in m/s²

    //Uncomment this line for easier debugging    
    //RCLCPP_INFO(this->get_logger(), "acc_x rate value : %d", combined_bytes);
    RCLCPP_INFO(this->get_logger(), "acc_x rate value : %f m/s²", acc_x);
    return acc_x;
}

/**
 * @brief Compute the acceleration about the y-axis
 */
float MPU6050::compute_acc_y() const
{
    __u8 high_byte = read_register(ACC_Y_OUT_HIGH_REGISTER);
    __u8 low_byte = read_register(ACC_Y_OUT_LOW_REGISTER);

    //Shift to the left the high byte by 8 bits to create a 16-bit word, then add low_byte at the end

    __s16 combined_bytes = (high_byte << 8) | low_byte;

    float acc_y_in_g = (combined_bytes - ACC_Y_OFFSET) / (ACC_CONFIG_0_SCALE); // in g
    float acc_y = acc_y_in_g * 9.81; //in m/s²

    //Uncomment this line for easier debugging    
    //RCLCPP_INFO(this->get_logger(), "acc_y rate value : %d", combined_bytes);
    RCLCPP_INFO(this->get_logger(), "acc_y rate value : %f m/s²", acc_y);
    return acc_y; 
}

/**
 * @brief Compute the acceleration about the z-axis
 * If the robot is immobile, the result should be around +9.81 m/s²
 * A "+" sign because the sensor is placed upside down on the breadboard,
 * thus the z-axis is oriented towards the ground
 */
float MPU6050::compute_acc_z() const
{
    __u8 high_byte = read_register(ACC_Z_OUT_HIGH_REGISTER);
    __u8 low_byte = read_register(ACC_Z_OUT_LOW_REGISTER);

    //Shift to the left the high byte by 8 bits to create a 16-bit word, then add low_byte at the end

    __s16 combined_bytes = (high_byte << 8) | low_byte;

    float acc_z_in_g = -(combined_bytes - ACC_Z_OFFSET) / (ACC_CONFIG_0_SCALE); // in g
    float acc_z = acc_z_in_g * 9.81; //in m/s²

    //Uncomment this line for easier debugging    
    //RCLCPP_INFO(this->get_logger(), "acc_z rate value : %d", combined_bytes);
    RCLCPP_INFO(this->get_logger(), "acc_z rate value : %f m/s²", acc_z);
    
    return acc_z; 
}

void MPU6050::publish_acc_x() const
{
    // Do not publish to _cmd_vel

    //float acc_x = compute_acc_x();
    //geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
    //message.linear.x = acc_x;
    //RCLCPP_INFO(this->get_logger(), "Publishing: linear x = %f", message.linear.x);
    //this->_cmd_vel_publisher->publish(message);
}   
 

/**
 * @brief Publish the yaw rate to the cmd vel topic in rad/s
 */
void MPU6050::publish_yaw_rate() const
{
    float yaw_rate = compute_yaw_rate();
    geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
    message.angular.z = yaw_rate;
    RCLCPP_INFO(this->get_logger(), "Publishing: angular_z = %f", message.angular.z);
    this->_cmd_vel_publisher->publish(message);
}


/**
 * @brief Used for calibrating the sensor, i.e. finding the offsets to substract to the read values
 * @param ACC_OUT_HIGH_REGISTER
 * @param ACC_OUT_LOW_REGISTER
 */
void MPU6050::calibrate_acc(const int &ACC_OUT_HIGH_REGISTER, const int &ACC_OUT_LOW_REGISTER) const
{
    // Read the high and low registers
    __u8 high_byte = read_register(ACC_OUT_HIGH_REGISTER);
    __u8 low_byte = read_register(ACC_OUT_LOW_REGISTER);

    //Shift to the left the high byte by 8 bits to create a 16-bit word, then add low_byte at the end
    __s16 combined_bytes = (high_byte << 8) | low_byte;

    RCLCPP_INFO(this->get_logger(), "Combined bytes :%d", combined_bytes);

    //Add to the combined_bytes value to the _recorded_values variable and increment the number of seen values by 1
    _recorded_values += combined_bytes;
    _number_of_values += 1;
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050>());
    rclcpp::shutdown();
    return 0;
}