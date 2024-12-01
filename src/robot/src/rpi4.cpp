#include <iostream>

/*
    Library for gpio control,
    pigpio must be installed
*/
#include <pigpio.h>


/*
    The following headers are found in robot/include/robot/
*/
#include "robot/rpi4.h"

RPI4::RPI4()
{
}

bool RPI4::init()
{
    /*
    Initialise the Raspberry Pi 4 GPIOs with the pigpio library.
    This function requires sudo to run for some reason.
    If you try to run it without sudo or some work around, you will have the "PI_INIT_FAILED" error message.
    One possible work around is : 
        sudo chown root:root $total_path
        sudo chmod 4755 $total_path
    where total_path is the path to the compiled file.
    That solution has been found here : https://forums.raspberrypi.com/viewtopic.php?t=180784
    
    We created a small script called make_encoders_work.sh (make_rpi_work.sh could be a better name) to do that.
    It is found in the ros_ws/scripts directory.
    We must run that script after using <colcon build>, otherwise the rpi will fail to initialize and the programm will output an error.
    */


    using namespace std;

    cout << "Using pigpio version : " << gpioVersion() << endl;

    //Disable builtin pigpio signal handling
    int cfg = gpioCfgGetInternals();
    cfg |= PI_CFG_NOSIGHANDLER;
    gpioCfgSetInternals(cfg);

    //Initialize pigpio library
    cout << "Initializing pigpio library" << endl;

    int gpio_result = 0;
    gpio_result = gpioInitialise();
    if(gpio_result == PI_INIT_FAILED)
    {
        cout << "PI_INIT_FAILED" << endl;
        return false;
    }

    cout << "PI INIT SUCCESS" << endl;
    return true;

}