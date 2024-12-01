#include <iostream>
#include <signal.h>
#include <csignal>
#include <thread>
#include <chrono>
#include <functional>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>


/*
    Library for gpio control
    pigpio must be installed
*/
#include <pigpio.h>

/*
    The following headers are found in robot/include/robot/
*/
#include "robot/encoders.h"


//This line is used/needed because of the callbacks and handlers.
Encoders* Encoders::instance = nullptr;


Encoders::Encoders()
/*
    Dummy constructor for quick testing
*/
{

}

Encoders::Encoders(const unsigned int &MxA_gpio,
                    const unsigned int &MxB_gpio,
                    const unsigned int &channel_A_gpio,
                    const unsigned int &channel_B_gpio,
                    const unsigned int &duty_cycle
                    )
                    : 
                    _MxA_gpio(MxA_gpio),
                    _MxB_gpio(MxB_gpio),
                    _channel_A_gpio(channel_A_gpio),
                    _channel_B_gpio(channel_B_gpio),
                    _duty_cycle(duty_cycle)
{
    // Initialize encoders with default pins/gpios.
    // However, we must initialize the rpi beforehand, otherwise it will return en error.

    // DO NOT COMMENT OUT INSTANCE = THIS;
    // Otherwise it will not work : you will see nothing but it won't crash
    instance = this;
    
    using namespace std;
    

    const int min_duty_cycle = 0;
    const int max_duty_cycle = 255;

    if (duty_cycle < min_duty_cycle)
    {
        cerr << "Error : duty_cycle has been set lower than 0 - encoders.cpp, init_pwm function" << endl;
        cerr << "Setting duty_cycle to 0" << endl;

        _duty_cycle = min_duty_cycle;
    }
    else if (duty_cycle > max_duty_cycle)
    {
        cerr << "Error : duty_cycle has been set higher than 255 - encoders.cpp, init_pwm function" << endl;
        cerr << "Setting duty_cycle to 255" << endl;
        _duty_cycle = max_duty_cycle;
    }
    else
    {
        _duty_cycle = duty_cycle;
    }

    init_pwm(_MxA_gpio, _duty_cycle);
    init_pwm(_MxB_gpio, 0);
    configure_gpio_as_output(_MxA_gpio);
    configure_gpio_as_output(_MxB_gpio);

    configure_gpio_as_input(_channel_A_gpio);
    configure_gpio_as_input(_channel_B_gpio);

    //const unsigned int interval = 1000; 
    //timer_start(compute_rps_handler, interval);
}

unsigned int Encoders::get_rps() const
{
    return _rps;
}

void Encoders::set_rps(const unsigned int &rps)
{
    /*
        For testing
    */
    _rps = rps;
}

void Encoders::start_encoder(const unsigned int &interval) 
{   
    //Start the encoder

    timer_start(compute_rps_handler, interval);
}

void Encoders::signal_handler_callback(int signal)
{
    // Allows for CTRL-C in the terminal
    // EDIT 11 Nov 2024 
    //     Not needed, see explanation in the next function
    


    using namespace std;
    cout << "Received signal : " << signal << ". Shutting down - encoders.cpp, signal_handler_callback function" << endl;

    //Stop the motor
    gpioPWM(_MxA_gpio, 0);
}

bool Encoders::configure_gpio_as_input(const unsigned int &gpio)
{
    // Set a gpio to input.
    // Used to record changes on channels.

    using namespace std;

    int gpio_result = 0;
    gpio_result = gpioSetMode(gpio, PI_INPUT);
    if(gpio_result != 0)
    {
        cerr << "gpio " << gpio << " : bad mode - encoders.cpp, configure_gpio_as_input function" << endl;
        return false;
    }
    cout << "Set mode INPUT success on gpio " << gpio << endl;

    return true;   
}

bool Encoders::configure_gpio_as_output(const unsigned int &gpio)
{
    // Configure a gpio as output.
    // Returns true if successful, false otherwise.


    using namespace std;
    int gpio_result = gpioSetMode(gpio, PI_OUTPUT);

    if(gpio_result!=0)
    {
        switch(gpio_result)
        {
            case PI_BAD_GPIO:
                cerr << gpio << "is a bad gpio pin - encoders.cpp, configure_gpio_as_output function" << endl;
                return false;
            case PI_BAD_MODE:
                cerr << "Bad mode for gpio - encoders.cpp, configure_gpio_as_output function" << gpio << endl;
                return false;
            default:
                cerr << "Unexpected error when configuring gpio - encoders.cpp, configure_gpio_as_output function" << gpio << endl;
                cout << "Result = " << gpio_result << endl;
                return false;
        }
    }
    return true;
}

void Encoders::signal_handler(int signal)
{
        // Needed for CTRL-C to work, this function is a work around.
        
        // EDIT 11 Nov 2024 
        //     Not needed, this function does not work when it is put in another file, e.g. a main.cpp file.
        //     It will understand the CTRL-C function, then call signal_handler_callback, but it won't stop the program.
        //     Then, if we do :  
        //         CTRL-Z,
        //         fg 
        //         ps, 
        //         kill <process id>
        //         fg
        //     and finally rerun the code, the terminal will output "PI_FAILED_TO_INIT" or something like that, even though we did not change the code.
        //     In that case, to be able to rerun the code, we have to turn off/turn on the rpi, which is annoying.
        //     The solution is to remove/deactivate the signal_handler in this file (encoders.cpp) and 
        //     to put the signal_handler in the main.cpp file, which is also much easier to implement.


    if(instance)
    {
        instance->signal_handler_callback(signal);
    }
}



void Encoders::timer_start(std::function<void(Encoders&)> func, 
                            const unsigned int &interval)
{
    // Initialize a timer to call a function every <interval> seconds.
    // <interval> is 1000 milliseconds in the code.

    using namespace std::chrono;
    
    _timer_thread = std::thread(
    [func, interval, this]()
    {
        while(true)
        {
            auto x = steady_clock::now() + milliseconds(interval);
            func(*this);
            std::this_thread::sleep_until(x);

        }
    }
    );

    _timer_thread.detach();

}

void Encoders::compute_rps()
{
    //
    //   Converts the number of changes recorded by the <count_number_of_changes> function to revolutions per second.
    //   Resets the number of changes on both channels by setting them both to 0.
    //


    using namespace std;

    //count_number_of_changes();

    unsigned int rps = 0;
    rps = (_number_of_changes_on_channel/_PPR)/_gear_ratio;
    _rps = rps;
    /*
        Debugging comments
    */

    // cout << "number of changes on " << _channel_name << " : " <<_number_of_changes_on_channel << endl;
    // cout << "rps on " << _channel_name  << " (gpio " << _channel_gpio << ") : " << " : " << rps << " rps" << endl;
    // cout << _channel_name << " "; 
    // cout << "rps : " << _rps << endl;

    _number_of_changes_on_channel = 0;
}


void Encoders::compute_rps_handler(Encoders &encoder)
{
    //
    //    Needed for the timer_start function
    //
   
    encoder.compute_rps();

}

bool Encoders::init_pwm(const unsigned int &gpio,
                        const unsigned int &duty_cycle
)
{
    // Initialize a PWM on a gpio.
    // Returns true if successful, false otherwise.
    

    using namespace std;

    int gpio_result = 0;
    gpio_result = gpioPWM(gpio, duty_cycle);
    if(gpio_result != 0)
    {
        cerr << "PWM Error on gpio " << gpio << " - encoders.cpp, init_pwm function" << endl;
        return false; 
    }
    cout << "PWM Success on gpio " << gpio << endl;
    return true;   
}


void Encoders::count_number_of_changes(const unsigned int &channel_gpio)
{
        
        // Counts the number of changes on value on channel A and B.
        // Used to determine linear velocity and angular velocity.
        
    _old_channel_value = _channel_value;

    _channel_value = gpioRead(channel_gpio);
    _change_in_channel = _channel_value != _old_channel_value;

    if (_change_in_channel && _channel_value == 1) 
    {
        _number_of_changes_on_channel+=1;
    }

}

/*
int main()
{
    std::cout << "coucou encoders.cpp" << std::endl;
    return 0;
}
*/

