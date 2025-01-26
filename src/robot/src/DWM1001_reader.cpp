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
    signal(SIGABRT, signalHandler);
    _point.x = 0.0f;
    _point.y = 0.0f;
    _point.z = 0.0f;

    _point_publisher = this->create_publisher<geometry_msgs::msg::Point>(POINT_TOPIC_NAME, POSITION_TOPIC_QUEUE_DEPTH);
    _marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>(MARKER_TOPIC_NAME, MARKER_TOPIC_QUEUE_DEPTH);
    auto timer_callback = [this]() -> void
    {
        //The raspberry pi pico must be assigned an IP address 
        _pipe = popen(_read_command, "r");
        if(!_pipe)
        {
            //If the python script was not called successfully
            // _read_command is "python3 ./raspberry_pico_w_server.py" by default
            RCLCPP_ERROR(this->get_logger(), "Failed to run < python3 ./raspberry_pico_w_server.py > command");
            pclose(_pipe);
            throw DWM1001_Initialization_Exception("Failed to run < python3 ./raspberry_pico_w_server.py > command");
        }
        else
        {
            //If the python script was called successfully, convert the readings into point coordinates and publish the point
            
            //RCLCPP_INFO(this->get_logger(), "Success running < python3 ./raspberry_pico_w_server.py > command");
            read_data();
            compute_x_from_readings();
            compute_y_from_readings();
        }

        update_point();
        this->_point_publisher->publish(_point);
        this->publish_point_to_rviz();
        RCLCPP_INFO(this->get_logger(), "Publishing: x = %f, y = %f, z = %f", _point.x, _point.y, _point.z);
    };
    //Publish every 500 ms
    _timer = this->create_wall_timer(50ms, timer_callback);




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
 * Axes x and y are swapped in the mobile application
*/
void DWM1001_reader::compute_y_from_readings()
{
    
    size_t pos = _readings.find(",");
    
    std::string sub_string = _readings.substr(pos+1);

    //Store the x coordinate in the _x variable
    double y_to_store = 0;
    y_to_store = std::stod(sub_string);

    if(y_to_store < _OUTLIER_VALUE)
    {
        _y = y_to_store;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "y : OUTLIER VALUE");
    }
    RCLCPP_INFO(this->get_logger(), "y_reading = %f", _y);

}

/**
 * @brief Extract the y-component from the DWM1001 readings
 * Data is received in the following form : "tagX,tagY", see raspberry_pico_w_server.py file
 * Axes x and y are swapped in the mobile application
*/
void DWM1001_reader::compute_x_from_readings()
{
    size_t pos = _readings.find(",");
    std::string sub_string = _readings.substr(0, pos);

    double x_to_store = 0;
    x_to_store = std::stod(sub_string);

    //Store the y coordinate in the _y variable
    if(x_to_store < _OUTLIER_VALUE)
    {
        _x = x_to_store;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "x: OUTLIER VALUE");
    }
    RCLCPP_INFO(this->get_logger(), "x_reading = %f", _x);
    
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

void DWM1001_reader::publish_point_to_rviz() const
{
    // We have to create a marker in order to display a point in rviz
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "geometry_point";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale and color of the point
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    marker.points.push_back(_point);
    _marker_publisher->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Publish marker to rviz");
    
}

/**
 * @brief Handle the abort signal
 * Possible reason : Raspberry Pico W server is not up.
 */
void DWM1001_reader::signalHandler(int signal)
{
    if (signal){} // THis line is only here to remove the warning after colcon build
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Aborting position_reader node. Possible reason : Raspberry Pico W server not up");
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


