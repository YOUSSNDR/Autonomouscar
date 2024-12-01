#include <functional> //std::function
#include <string.h>
#include <thread>

#ifndef ENCODERS_H
#define ENCODERS_H

struct Encoders
{
    public:
        Encoders();
        Encoders(const unsigned int &MxA_gpio,
                const unsigned int &MxB_gpio,
                const unsigned int &channel_A_gpio,
                const unsigned int &channel_B_gpio,
                const unsigned int &duty_cycle
        );

        unsigned int get_rps() const;
        void set_rps(const unsigned int &rps);
        void start_encoder(const unsigned int &interval);

    private:
        /*
            Gpios are initialized to default values.
            Can be changed.
        */
        const unsigned int _MxA_gpio = 0;
        const unsigned int _MxB_gpio = 0;
        const unsigned int _channel_A_gpio = 0;
        const unsigned int _channel_B_gpio = 0;
        /*
            Track changes in A and B channels.
            Used for measuring angular velocity of the wheels.
            If we have the radius of the wheels and the angular velocity of the wheels, we can deduce : 
                - the linear velocity of the robot
                - the angular velocity of the robot around the z-axis
        */

        unsigned int _channel_value = 0;
        unsigned int _old_channel_value = 0;

        bool _change_in_channel = false;

        unsigned int _number_of_changes_on_channel = 0;
        unsigned int _rps = 0;


        /*
            PPR, (pulses per round) gear_ratio are found on the documentation of the motors :
            https://wiki.dfrobot.com/Micro_DC_Motor_with_Encoder-SJ01_SKU__FIT0450

            gear_ratio is said to be 120:1. 
            With not so precise experiments I found that 900:8 could be better.

        */
        const unsigned int _PPR = 8;
        const double _gear_ratio = 900/8;
        unsigned int _duty_cycle = 200; //max : 255

        /*
            static Encoders *instance is needed for callbacks and handlers
            *instance has to be initialized in the encoders.cpp file, e.g. Encoders* Encoders::instance = nullptr;
        */

        static Encoders *instance;
        
        std::thread _timer_thread;
        bool _is_running =  false;

        bool configure_gpio_as_input(const unsigned int &gpio);
        bool configure_gpio_as_output(const unsigned int &gpio);

        void signal_handler_callback(int signal);
        static void signal_handler(int signal);


        void count_number_of_changes(const unsigned int &channel_gpio);
        void compute_rps();
        static void compute_rps_handler(Encoders &encoder);

        void timer_start(std::function<void(Encoders&)> func, 
                        const unsigned int &interval);

        
        bool init_pwm(const unsigned int &gpio, 
                    const unsigned int &duty_cycle);


};

#endif
