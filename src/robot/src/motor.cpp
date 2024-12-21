#include "robot/motor.h"

/**
 * @brief Constructor for a motor
 * @param MxA_gpio the M1A gpio or M2A gpio
 * @param MxB_gpio the M1B gpio or M2B gpio
 * @param duty_cycle a number between 0 and 255
*/
Motor::Motor(const unsigned int &MxA_gpio,
        const unsigned int &MxB_gpio,
        const unsigned int &duty_cycle):
    _MxA_gpio(MxA_gpio),
    _MxB_gpio(MxB_gpio),
    _duty_cycle(duty_cycle)
{
    init_pwm(_MxA_gpio, 0);
    init_pwm(_MxB_gpio, _duty_cycle);

    configure_gpio_as_output(_MxA_gpio);
    configure_gpio_as_output(_MxB_gpio);
}

/**
 * @brief Initialize a PWM on a gpio.
 * Returns true if successful, false otherwise.
 * @param gpio the gpio pin number of the rpi4
 * @param duty_cycle a number between 0 and 255
*/


bool Motor::init_pwm(const unsigned int &gpio,
                        const unsigned int &duty_cycle) const
{    
    using namespace std;

    int gpio_result = 0;

    if (duty_cycle > MAX_DUTY_CYCLE)
    {
        cerr << "Error : duty_cycle has been set higher than 255 - motor.cpp, init_pwm function" << endl;
    }

    gpio_result = gpioPWM(gpio, duty_cycle);
    if(gpio_result != 0)
    {
        cerr << "PWM Error on gpio " << gpio << " - motor.cpp, init_pwm function" << endl;
        return false; 
    }
    cout << "PWM Success on gpio " << gpio << endl;
    return true;   
}

/**
 * @brief Configure a gpio as output
 * Return true if successful, false otherwise.
*/
bool Motor::configure_gpio_as_output(const unsigned int &gpio) const
{

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

/**
 * @brief Sets the duty cycle of the motor to duty_cycle
 * Changes the angular speed of the motor
*/
void Motor::set_duty_cycle(const unsigned int &duty_cycle)
{
    if (duty_cycle > MAX_DUTY_CYCLE)
    {
        _duty_cycle = MAX_DUTY_CYCLE;
    }
    else
    {
        _duty_cycle = duty_cycle;
    }
    
}

bool Motor::stop()
{
    using namespace std;
    int gpio_result_A = 0;
    int gpio_result_B = 0;

    set_duty_cycle(0);
    gpio_result_A = gpioPWM(_MxA_gpio, _duty_cycle);
    gpio_result_B = gpioPWM(_MxB_gpio, _duty_cycle);

    if(gpio_result_A != 0)
    {
        cerr << "PWM Error on gpio " << _MxA_gpio << " - motor.cpp, stop - function" << endl;
        return false; 
    }

    if(gpio_result_B != 0)
    {
        cerr << "PWM Error on gpio " << _MxB_gpio << " - motor.cpp, stop - function" << endl;
        return false; 
    }

    cout << "PWM Success on stopping the motor " << endl;
    return true;
}