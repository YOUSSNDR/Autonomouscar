#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef DWM1001_READER_H
#define DWM1001_READER_H

struct DWM1001_reader
{
    public:
        DWM1001_reader();
    private:

        const char *_read_command = "python3 ./raspberry_pico_w_server.py";
        std::string _readings;
        char _buffer[128];
        FILE *_pipe = nullptr;

        double _x = 0;
        double _y = 0;
    
        void read_data();
        void compute_x_from_readings();
        void compute_y_from_readings();

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