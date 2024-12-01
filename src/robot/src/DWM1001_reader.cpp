#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>

/*
    For the "ms" literal
*/
#include <chrono>
/*
    ROS message for a 3D point
    x,y,z coordinates
*/
#include <geometry_msgs/msg/point.hpp>

/*
    The following headers are found in robot/include/robot/
*/
#include "robot/DWM1001_reader.h"

/*
    The purpose of this code is to read the transmitted data from the Raspberry Pico W using Wifi.
    Transmitted data should be (x,y) coordinates relative to the robot.
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
        RCLCPP_INFO(this->get_logger(), "Publishing: x = '%f', y = %f, z = %f", _point.x, _point.y, _point.z);

        /*
            Testing by setting x and y to some values
        */
        set_x(3);
        set_y(4);
        
        update_point();
        this->_publisher->publish(_point);
    };
    _timer = this->create_wall_timer(500ms, timer_callback);


    // _read_command is "python3 ./raspberry_pico_w_server.py" by default
    _pipe = popen(_read_command, "r");
    if(!_pipe)
    {
        std::cerr << "Failed to run command" << std::endl;
        throw DWM1001_Initialization_Exception("Failed to run command");
    }

}

DWM1001_reader::DWM1001_reader(const bool &dummy) : Node(NODE_NAME)
{    
    using namespace std::chrono_literals;

    _point.x = 0.0f;
    _point.y = 0.0f;
    _point.z = 0.0f;
    _publisher = this->create_publisher<geometry_msgs::msg::Point>(TOPIC_NAME, QUEUE_DEPTH);

    auto timer_callback = [this]() -> void
    {
        RCLCPP_INFO(this->get_logger(), "Publishing: x = '%f', y = %f, z = %f - DWM1001_reader", _point.x, _point.y, _point.z);

        /*
            Testing by setting x and y to some values
        */
        set_x(3);
        set_y(4);
        
        update_point();
        this->_publisher->publish(_point);
    };
    _timer = this->create_wall_timer(500ms, timer_callback);

    if(dummy)
    {

    }
}

void DWM1001_reader::read_data()
{
    while (fgets(_buffer, sizeof(_buffer), _pipe) != nullptr) 
    {
        _readings += _buffer;
    }

    fclose(_pipe);   
}

void DWM1001_reader::compute_x_from_readings()
{
    //Extract the x-component from the DWM1001 readings
    
    size_t pos = _readings.find(":");
    std::string sub_string = _readings.substr(pos+1);
    size_t comma_pos = sub_string.find(",");
    _x = std::stod(sub_string.substr(0, comma_pos)); 
    std::cout << "x = " << _x << std::endl;
}

void DWM1001_reader::compute_y_from_readings()
{
    //Extract the y-component from the DWM1001 readings

    size_t pos = _readings.find(":");
    std::string sub_string = _readings.substr(pos + 1);
    size_t comma_pos = sub_string.find(",");
    _y = std::stod(sub_string.substr(comma_pos + 1));
    std::cout << "y = " << _y << std::endl;
}


void DWM1001_reader::set_x(const double &x)
{
    _x = x;
}

void DWM1001_reader::set_y(const double &y)
{
    _y = y;
}

void DWM1001_reader::update_point()
{
    /*
        Supposing we are in 2D.
        Thus z = 0
    */
    _point.x = _x;
    _point.y = _y;
    _point.z = 0;

    
}
//Custom exceptions for the DWM1001 module

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
    bool dummy = true;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DWM1001_reader>(dummy));
    rclcpp::shutdown();
    return 0;
}

