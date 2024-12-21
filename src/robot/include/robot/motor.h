
#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>

/*
    Library for gpio control
    pigpio must be installed
*/
#include <pigpio.h>


#define MIN_DUTY_CYCLE 0
#define MAX_DUTY_CYCLE 255

/**
 * @brief A class to represent a motor 
 * Subscriber to the cmd_vel topic
*/

struct Motor
{
    public:

        Motor(const unsigned int &MxA_gpio,
            const unsigned int &MxB_gpio,
            const unsigned int &duty_cycle);

        void set_duty_cycle(const unsigned int &duty_cycle);
        bool stop();

    private:
        const unsigned int _MxA_gpio;
        const unsigned int _MxB_gpio;
        unsigned int _duty_cycle = 0;

        bool init_pwm(const unsigned int &gpio,
                            const unsigned int &duty_cycle) const;
        bool configure_gpio_as_output(const unsigned int &gpio) const;
};

#endif