/*
    The following header is found in robot/include/robot/
*/
#include "robot/encoders.h"



//This line is used/needed because of the callbacks and handlers.
Encoders* Encoders::instance = nullptr;

/*
    Must be initialized here for the ISR compute_number_of_changes_A and compute_number_of_changes_B
*/
volatile unsigned int Encoders::_number_of_changes_on_channel_A = 0U; 
volatile unsigned int Encoders::_number_of_changes_on_channel_B = 0U;

/**
 * @brief Dummy constructor for quick testing
*/
Encoders::Encoders()
{

}

/**
 * @brief Initialize encoders with default pins/gpios.
 * However, we must initialize the rpi beforehand, otherwise it will return en error.
 * @param MxA_gpio
 * @param MxB_gpio
 * @param channel_A_gpio
 * @param channel_B_gpio
 * @param duty_cycle between 0 and 255
*/
Encoders::Encoders(const unsigned int &channel_A_gpio,
                const unsigned int &channel_B_gpio,
                const std::string &encoder_name)
                :_encoder_name(encoder_name), 
                _channel_A_gpio(channel_A_gpio),
                 _channel_B_gpio(channel_B_gpio)
{

    // DO NOT COMMENT OUT INSTANCE = THIS;
    // Otherwise it will not work : you will see nothing but it won't crash
    instance = this;
    
    using namespace std;
    

    configure_gpio_as_input(_channel_A_gpio);
    configure_gpio_as_input(_channel_B_gpio);

    cout << "Channel A GPIO : " << _channel_A_gpio << endl;
    cout << "Channel B GPIO : " << _channel_B_gpio << endl;

    gpioSetAlertFunc(channel_A_gpio, count_number_of_changes_on_channel_A);
    gpioSetAlertFunc(channel_B_gpio, count_number_of_changes_on_channel_B);

    const unsigned int interval = 1000; 
    timer_start(compute_encoder_rps_handler, interval);

}

unsigned int Encoders::get_rps_A() const
{
    return _rps_A;
}

unsigned int Encoders::get_rps_B() const
{
    return _rps_B;
}

/**
 * @brief Set both _rps_A and _rps_B to an rps value 
 * Mainly used for quick testing
 * @param rps the value to set _rps_A and _rps_B to
*/
void Encoders::set_rps(const unsigned int &rps)
{
    set_rps_A(rps);
    set_rps_B(rps);
}
/**
 * @brief Set _rps_A to an rps value 
 * Mainly used for quick testing
 * @param rps the value to set _rps_A to
*/
void Encoders::set_rps_A(const unsigned int &rps)
{
    _rps_A = rps;
}

/**
 * @brief Set _rps_B to an rps value 
 * Mainly used for quick testing
 * @param rps the value to set _rps_B to
*/
void Encoders::set_rps_B(const unsigned int &rps)
{
    _rps_B = rps;
}

/**
 * @brief Start the encoder
 * @param interval in milliseconds
*/
void Encoders::start_encoder(const unsigned int &interval) 
{   
    timer_start(compute_encoder_rps_handler, interval);
}


/**
 * @brief Allows for CTRL-C in the terminal
 * EDIT 11 Nov 2024 
 *    Not needed, see explanation in the signal_handler function
*/

void Encoders::signal_handler_callback(int signal)
{
    using namespace std;
    cout << "Received signal : " << signal << ". Shutting down - encoders.cpp, signal_handler_callback function" << endl;
}


/**
 * @brief Set a gpio to input
 * Used to record changes on channels
 * Return true if successful, false otherwise
 * @param gpio the gpio number on the rpi4
*/
bool Encoders::configure_gpio_as_input(const unsigned int &gpio)
{

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

/**
 * @brief Initialize a timer to call a function every <interval> seconds.
 * @param interval is set to 1000 milliseconds in the code.
*/

void Encoders::timer_start(std::function<void(Encoders&)> func, 
                            const unsigned int &interval)
{

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

/**
 * @brief Converts the number of changes recorded by the <count_number_of_changes> function to the encoder revolutions per second
 * Resets the number of changes on both channels by setting them both to 0.
*/
void Encoders::compute_encoder_rps()
{

    using namespace std;
    
    //count_number_of_changes(_channel_A_gpio);

    unsigned int encoder_number_of_revolutions_per_second_A = _number_of_changes_on_channel_A/_PPR; //revolutions

    unsigned int encoder_number_of_revolutions_per_second_B = _number_of_changes_on_channel_B/_PPR;
    float best_rps_estimation = (encoder_number_of_revolutions_per_second_A + encoder_number_of_revolutions_per_second_B) / 2;
    /*
        Convert encoder revolutions per second into motor revolutions per second
    */
    float motor_rps_estimation = best_rps_estimation/_gear_ratio;
    /*
        Convert revolutions per second to radians per second
    */
    float motor_angular_velocity_estimation = motor_rps_estimation * 2*M_PI;
    float motor_linear_velocity_estimation = motor_angular_velocity_estimation*WHEEL_RADIUS;
    /*
        Debugging comments
    */
    cout << _encoder_name << " on channel A rps (revolutions per second): " << encoder_number_of_revolutions_per_second_A << endl;
    cout << _encoder_name << " on channel B rps (revolutions per second): " << encoder_number_of_revolutions_per_second_B << endl;
    cout << _encoder_name << " best rps estimation: " << best_rps_estimation << endl;
    cout << "estimated motor revolutions per second: " << motor_rps_estimation << endl;
    cout << "estimated linear velocity: " << motor_linear_velocity_estimation << " m/s"<<endl;
    cout << "compute_encodder_rps function in the encoder.cpp file" << endl;
    cout << "####" << endl;

    /*
        Resetting the values
    */
    _number_of_changes_on_channel_A = 0;
    _number_of_changes_on_channel_B = 0;
}

/**
 * @brief Needed for the timer_start function
*/
void Encoders::compute_encoder_rps_handler(Encoders &encoder)
{   
    encoder.compute_encoder_rps();
}


/**
 * @brief Counts the number of changes on value on channel A and B.
 * Used to determine linear velocity and angular velocity.*/

void Encoders::count_number_of_changes_on_channel_A(int gpio, int level, uint32_t tick)
{

   if (gpio || level || tick){} //This line is here to remove the warning after colcon build    
    _number_of_changes_on_channel_A += 1;
}


void Encoders::count_number_of_changes_on_channel_B(int gpio, int level, uint32_t tick)
{
   if (gpio || level || tick){} //This line is here to remove the warning after colcon build  
    _number_of_changes_on_channel_B += 1;
}


/*
int main()
{
    std::cout << "coucou encoders.cpp" << std::endl;
    return 0;
}
*/

