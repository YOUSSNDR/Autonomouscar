// The following headers are found in robot/include/robot/
#include "robot/DWM1001_reader.h"

/**
 * @brief The purpose of this code is to read the transmitted data from the Raspberry Pico W using Wifi.
 * It is to locate the tag the user holds.
 * Transmitted data should be (x,y) coordinates relative to the robot.
 * z = 0 = constant by default. It might be a good idea to change it.
*/

DWM1001_reader::DWM1001_reader(): Node(NODE_NAME)
{
    using namespace std::chrono_literals;

    _point.x = 0.0f;
    _point.y = 0.0f;
    _point.z = 0.0f;

    _publisher = this->create_publisher<geometry_msgs::msg::Point>(TOPIC_NAME, QUEUE_DEPTH);

    auto timer_callback = [this]() -> void
    {
        //The raspberry pi pico must be assigned an IP address 
        _pipe = popen(_read_command, "r");
        if(!_pipe)
        {
            //If the python script was not called successfully
            RCLCPP_ERROR(this->get_logger(), "Failed to run < python3 ./raspberry_pico_w_server.py > command");
            pclose(_pipe);
            throw DWM1001_Initialization_Exception("Failed to run < python3 ./raspberry_pico_w_server.py > command");
        }
        else
        {
            //If the python script was called successfully, convert the readings into point coordinates and publish the point
            
            RCLCPP_INFO(this->get_logger(), "Success running < python3 ./raspberry_pico_w_server.py > command");
            read_data();
            compute_x_from_readings();
            compute_y_from_readings();
        }

        update_point();
        this->_publisher->publish(_point);
        RCLCPP_INFO(this->get_logger(), "Publishing: x = %f, y = %f, z = %f", _point.x, _point.y, _point.z);
    };
    //Publish every 500 ms
    _timer = this->create_wall_timer(50ms, timer_callback);


    // _read_command is "python3 ./raspberry_pico_w_server.py" by default


}

/**
 * @brief Read the data coming from the raspberry pico w and store it in _readings
*/
void DWM1001_reader::read_data()
{
    _readings = "";
    while (fgets(_buffer, sizeof(_buffer), _pipe) != nullptr) 
    {
        _readings += _buffer;
    }

    fclose(_pipe);   
}


/**
 * @brief Extract the x-component from the DWM1001 readings
 * Data is received in the following form : "tagX,tagY", see raspberry_pico_w_server.py file
*/
void DWM1001_reader::compute_x_from_readings()
{
    
    size_t pos = _readings.find(",");
    std::string sub_string = _readings.substr(0, pos);
    std::cout << sub_string << std::endl;

    //Store the x coordinate in the _x variable
    _x = std::stod(sub_string);
    RCLCPP_INFO(this->get_logger(), "x_reading = %f", _x);

}

/**
 * @brief Extract the y-component from the DWM1001 readings
 * Data is received in the following form : "tagX,tagY", see raspberry_pico_w_server.py file
*/
void DWM1001_reader::compute_y_from_readings()
{
    size_t pos = _readings.find(",");
    std::string sub_string = _readings.substr(pos+1);

    //Store the y coordinate in the _y variable
    _y = std::stod(sub_string);
    RCLCPP_INFO(this->get_logger(), "y_reading = %f", _y);
    
}

/**
 * @brief Set _x to the value x
 * Mainly used for testing
 * @param x
*/
void DWM1001_reader::set_x(const double &x)
{
    _x = x;
}

/**
 * @brief Set _y to the value y
 * Mainly used for testing
 * @param y
*/

void DWM1001_reader::set_y(const double &y)
{
    _y = y;
}

/**
 * @brief Update the coordinates
 * Supposing we are in 2D, z = 0
*/

void DWM1001_reader::update_point()
{
    _point.x = _x;
    _point.y = _y;
    _point.z = 0;

}
/**
 * @brief Custom exceptions for the DWM1001 module
 * @param error_message the error message to be displayed
*/

DWM1001_Initialization_Exception::DWM1001_Initialization_Exception(const std::string &error_message)
{
    _error_message = error_message;
}

const char* DWM1001_Initialization_Exception::what() const noexcept
{
    return _error_message.c_str();
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DWM1001_reader>());
    rclcpp::shutdown();
    return 0;
}


