#ifndef DWM1001_READER_H
#define DWM1001_READER_H

#include <cstdio>
#include <cstdlib>
#include <cstring>


//Needed to use ros functions
#include "rclcpp/rclcpp.hpp"

// Message to create a 3D point
#include <geometry_msgs/msg/point.hpp>

// For the "ms" literal
#include <chrono>

#define NODE_NAME "DWM1001_node"
#define TOPIC_NAME "position_topic"
#define QUEUE_DEPTH 10

/*
    Publisher node
    Publishes the x,y coordinates read from the Raspberry Pi Pico W via WIFI that the operator holds
    We are in 2D thus z = 0
*/

struct DWM1001_reader : public rclcpp::Node
{
    public:
        DWM1001_reader();

        
        // Functions for testing
        void set_x(const double &x);
        void set_y(const double &y);
        DWM1001_reader(const bool &dummy);
    
    private:
        
        //Creating a node to publish a point message
        
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr _publisher;
        rclcpp::TimerBase::SharedPtr _timer;
        geometry_msgs::msg::Point _point;

        
        //Read from the raspberry pi pico W
        const char *_read_command = "python3 ./src/robot/src/raspberry_pico_w_server.py";
        std::string _readings;
        char _buffer[128];
        FILE *_pipe = nullptr;

        double _x = 0;
        double _y = 0;
    
        
        //Main functions
        void read_data();
        void compute_x_from_readings();
        void compute_y_from_readings();

        void update_point();
};

#endif

#ifndef DWM1001_INITIALIZATION_EXCEPTION
#define DWM1001_INITIALIZATION_EXCEPTION

struct DWM1001_Initialization_Exception : public std::exception
{
    public:
        DWM1001_Initialization_Exception(const std::string &error_message);
        const char* what() const noexcept override;
    private:
        std::string _error_message = "_error_message in the DWM1001_reader.h file";
};

#endif