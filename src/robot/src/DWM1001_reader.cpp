#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>

/*
    The following headers are found in robot/include/robot/
*/
#include "robot/DWM1001_reader.h"


/*
    The purpose of this code is to read the transmitted data from the Raspberry Pico W using Wifi.
    Transmitted data should be (x,y) coordinates relative to the robot.
*/

DWM1001_reader::DWM1001_reader()
{
    // _read_command is "python3 ./raspberry_pico_w_server.py" by default
    
    _pipe = popen(_read_command, "r");
    if(!_pipe)
    {
        std::cerr << "Failed to run command" << std::endl;
        throw DWM1001_Initialization_Exception("Failed to run command");
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


//Custom exceptions for the DWM1001 module

DWM1001_Initialization_Exception::DWM1001_Initialization_Exception(const std::string &error_message)
{
    _error_message = error_message;
}

const char* DWM1001_Initialization_Exception::what() const noexcept
{
    return _error_message.c_str();
}


