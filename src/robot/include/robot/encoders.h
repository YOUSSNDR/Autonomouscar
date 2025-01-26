#ifndef ENCODERS_H
#define ENCODERS_H


#include <string.h>

// For std::function
#include <functional> //std::function

// For threads
#include <thread>
//For the ms literal
#include <chrono>

#include <iostream>
#include <signal.h>
#include <csignal>
#include <functional>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>


//Needed for the M_PI constant to convert revolutions per second to radians per second

#include <math.h>



// Library for gpio control
// pigpio must be installed
// sudo pigpiod must be run

#include <pigpiod_if2.h>

#define WHEEL_RADIUS 0.034 //meter

struct Encoders
{
    public:
        Encoders();
        Encoders(const unsigned int &channel_A_gpio,
                const unsigned int &channel_B_gpio,
                const std::string &encoder_name
                );

        unsigned int get_rps_A() const;
        unsigned int get_rps_B() const;
        /*
            Set either _rps_A or _rps_B
        */
        void set_rps(const unsigned int &rps);
        void set_rps_A(const unsigned int &rps);
        void set_rps_B(const unsigned int &rps);
        void start_encoder(const unsigned int &interval);

    private:
        std::string _encoder_name = "";

        int _pi = -1;
        // Gpios are initialized to default values.
        // Can be changed.
        
        const unsigned int _channel_A_gpio = 0;
        const unsigned int _channel_B_gpio = 0;
        /*
            Track changes in A and B channels.
            Used for measuring angular velocity of the wheels.
            If we have the radius of the wheels and the angular velocity of the wheels, we can deduce : 
                - the linear velocity of the robot
                - the angular velocity of the robot around the z-axis
        */

        /*
        unsigned int _channel_value_A = 0U;
        unsigned int _old_channel_value_A = 0U;

        unsigned int _channel_value_B = 0U;
        unsigned int _old_channel_value_B = 0U;

        bool _change_in_channel_A = false;
        bool _change_in_channel_B = false;


        */

        
         //_rps_A is the number of encoder revolutions per second measured in channel A
         //_rps_B is the number of encoder revolutions per second measured in channel B
    
        volatile unsigned int _rps_A = 0U;
        volatile unsigned int _rps_B = 0U;

        
        // Initalized at the top of the encoders.cpp file
        

        volatile unsigned int _number_of_changes_on_channel_A = 0U; 
        volatile unsigned int _number_of_changes_on_channel_B = 0U;
        
        //    PPR, (pulses per round) gear_ratio are found on the documentation of the motors :
        //    https://wiki.dfrobot.com/Micro_DC_Motor_with_Encoder-SJ01_SKU__FIT0450

        //    gear_ratio is said to be 120:1. 
        //    With not so precise experiments I found that 900:8 could be better.

        const unsigned int _PPR = 8;
        const double _gear_ratio = 900/8;

        
        // static Encoders *instance is needed for callbacks and handlers
        // *instance has to be initialized in the encoders.cpp file, e.g. Encoders* Encoders::instance = nullptr;
    
        static Encoders *instance;
        
        std::thread _timer_thread;
        bool _is_running =  false;

        bool configure_gpio_as_input(const unsigned int &gpio);

        void signal_handler_callback(int signal);
        static void signal_handler(int signal);

        /*
            Count the number of changes on a channel and compute the number of revolutions of the encoder using the PPR
        */
        void compute_encoder_rps();
        static void compute_encoder_rps_handler(Encoders &encoder);

        void timer_start(std::function<void(Encoders&)> func, 
                        const unsigned int &interval);

        
        // The function declaration must be static and must have int pi, unsigned int gpio, unsigned int level, and uint32_t tick declaration to work with the pigpio lib
        // void* instance to expect an Encoder struct to be passed
        static void count_number_of_changes_on_channel_A(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *instance_ptr);
        static void count_number_of_changes_on_channel_B(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void* instance_ptr);


};

#endif
