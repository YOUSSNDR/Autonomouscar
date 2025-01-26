#include <iostream>
#include <pigpio.h>
#include <signal.h>
#include <csignal>
#include <thread>
#include <chrono>
#include <functional>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

//#include "WiringPi/wiringPi/wiringPi.h"
//#include "WiringPi/wiringPi/wiringSerial.h"
#include <wiringSerial.h>


using namespace std::chrono;

const int TXD_UART = 14;
const int RXD_UART = 15 ;

const unsigned int M1A_PIN = 17;
const unsigned int M1B_PIN = 18;
const unsigned int A_channel = 5;
const unsigned int B_channel = 6;

int A_channel_value = 0;
int B_channel_value = 0;
int A_channel_old_value = 0;
int B_channel_old_value = 0;

unsigned int duty_cycle = 200; //sur 255

bool change_in_A_channel = false;
bool change_in_B_channel = false;
int cpt = 0;
double number_of_changes_A_channel = 0;
double number_of_changes_B_channel = 0;

float rotations = 0;
int period_ms = 1; //ms

const int PPR = 8;
const double gear_ratio = 900/8; //120/1;

int gpio_result = 0;
bool isRunning = false;

bool configurePinAsOutput(unsigned int gpio)
{
    int gpio_result = gpioSetMode(gpio, PI_OUTPUT);

    if(gpio_result!=0)
    {
        switch(gpio_result)
        {
            case PI_BAD_GPIO:
                std::cout<<gpio<< "is a bad gpio pin" << std::endl;
            case PI_BAD_MODE:
                std::cout<<"Bad mode for gpio " << gpio << std::endl;
            default:
                std::cout<<"Unexpected error when configuring gpio " << gpio << std::endl;
                std::cout << "Result = " << gpio_result << std::endl;
                return false;
        }
    }
    
    return true;
}

bool configurePins(unsigned int M1A_PIN, unsigned int M1B_PIN)
{
    if(!configurePinAsOutput(M1A_PIN))
    {
        std::cout << "M1A_PIN configuration bug" << std::endl;
        return false;
    }

    if(!configurePinAsOutput(M1B_PIN))
    {
        std::cout<<"M1B_PIN configuration bug" << std::endl;
        return false;
    }

    return true;

}

void signalHandler(int signal)
{
    std::cout << "Received signal : " << signal << ". Shutting down" << std::endl;
    gpioPWM(M1A_PIN, 0);
    isRunning = false;
}

void timer_start(std::function<void(void)> func, unsigned int interval)
{
    std::thread([func, interval]()
    {
        while(true)
        {
            auto x = steady_clock::now() + milliseconds(interval);
            func();
            std::this_thread::sleep_until(x);

        }
    }).detach();
}

void compute_rps()
{
    //int serial_port = open("/dev/ttyUSB0", O_RDWR);
    std::cout << "number of changes channel A  : " << number_of_changes_A_channel << std::endl;
    std::cout << "number of changes channel B  : " << number_of_changes_B_channel << std::endl;

    std::cout << "rps channel A : " << (number_of_changes_A_channel/PPR)/gear_ratio << std::endl;
    std::cout << "rps channel B : " << (number_of_changes_B_channel/PPR)/gear_ratio << std::endl;
    number_of_changes_A_channel = 0;
    number_of_changes_B_channel = 0;
}

int main(){
    std::cout<<"Using pigpio version :" << gpioVersion() << std::endl;
    
    //Disable builtin pigpio signal handling
    int cfg = gpioCfgGetInternals();
    cfg |= PI_CFG_NOSIGHANDLER;
    gpioCfgSetInternals(cfg);

    signal(SIGINT, signalHandler);

    //Initialize pigpio library
    std::cout << "Initializing pigpio library" << std::endl;
    gpio_result = gpioInitialise();
    if(gpio_result == PI_INIT_FAILED)
    {
        std::cout << "PI_INIT_FAILED" << std::endl;
        return -1;
    }

    std::cout<<"PI_INIT SUCCESS" << std::endl;

    //PWM INIITIALSIATION

    gpio_result = gpioPWM(M1A_PIN, duty_cycle);
    if(gpio_result != 0)
    {
        std::cout << "PWM Error on gpio " << M1A_PIN << std::endl;
        return -1; 
    }
    std::cout << "PWM Success on gpio " << M1A_PIN << std::endl;


    gpio_result = gpioPWM(M1B_PIN, 0);
    if(gpio_result != 0)
    {
        std::cout << "PWM error on gpio " << M1B_PIN << std::endl;
        return -1; 
    }
    std::cout << "PWM Success on gpio " << M1B_PIN << std::endl;

    //ENCODER READINGS

    gpio_result = gpioSetMode(A_channel, PI_INPUT);
    if(gpio_result != 0)
    {
        std::cout << "GPIO " 
        << A_channel 
        << " : bad mode" << std::endl;
        return -1;
    }
    std::cout << "Set mode INPUT success on gpio " << A_channel << std::endl;

    gpio_result = gpioSetMode(B_channel, PI_INPUT);
    if(gpio_result != 0){
        std::cout << "GPIO " 
        << B_channel 
        << " : bad mode" << std::endl;
        return -1;
    }
    std::cout << "Set mode INPUT success on gpio " << B_channel << std::endl;

    int pin = 7;

    //UART
    gpioSetMode(TXD_UART, PI_OUTPUT);
    gpioSetMode(RXD_UART, PI_INPUT);
    int serialDeviceId = 0;
    //const char *uart_path = "/dev/ttyS0";
    serialDeviceId = serialOpen("/dev/ttyS0", 115200);

    if(serialDeviceId == -1)
    {
        std::cerr << "Failed to open UART" << std::endl;
        return 1;
    }

    /*if(serialDeviceId == -1)
    {
        std::cerr << "Error opening uart" << strerror(errno)<<std::endl;
        return 1;
    }
*/
    unsigned char bytesToSend[] = {0x04, 0x00};
    


    

    isRunning = true;
    timer_start(compute_rps, 1000);

    while(isRunning)
    {
        //std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
        //duty_cycle+=1;
        //cpt+=1;
        //if(duty_cycle>255) duty_cycle=0;
        //auto start = high_resolution_clock::now();

        for(unsigned char byte : bytesToSend)
        {
            serialPutchar(serialDeviceId, byte);
            std::cout << "Sent bytes : 0x" << (int)byte << std::endl;

        }


        if(serialDataAvail(serialDeviceId))
        {
            unsigned char response_byte = serialGetchar(serialDeviceId);
            std::cout << "Received byte : 0x" << std::hex << (int)response_byte << std::dec << std::endl;
        }
        else{
            std::cout << "No data received" << std::endl;
        }

        gpio_result = gpioPWM(M1A_PIN, duty_cycle);
        if(gpio_result != 0)
        {
            std::cout << "PWM Error on gpio " << M1A_PIN << std::endl;
            return -1; 
        }
        A_channel_old_value = A_channel_value;
        B_channel_old_value = B_channel_value;

        A_channel_value = gpioRead(A_channel);
        B_channel_value = gpioRead(B_channel);

        change_in_A_channel = A_channel_value != A_channel_old_value;
        change_in_B_channel = B_channel_value != B_channel_old_value;

        if (change_in_A_channel && A_channel_value == 1) 
        {
            number_of_changes_A_channel+=1;
        }

        if (change_in_B_channel && B_channel_value == 1)
        {
            number_of_changes_B_channel+=1;
        }

        //isRotating =  change_in_A_channel || change_in_B_channel;

        //std::cout << "duty_cycle : " << duty_cycle << std::endl;
        //std::cout << "A channel value : " << A_channel_value << std::endl;
        //std::cout << "B channel value : " << B_channel_value << std::endl;
        //std::cout << "number of changes channel A  : " << number_of_changes_A_channel << std::endl;

       /*if(number_of_changes_A_channel == 900) //960, 900 a l'air d'être plus précis pour indiquer un tour de roue complet
       {
        gpioPWM(M1A_PIN, 0);
        auto end = high_resolution_clock::now();
        double duration_sec = duration_cast<microseconds>(end-start).count()*1E-6;
        std::cout << "duration : " << duration_sec << std::endl;
        std::cout << "rps : " << 1/duration_sec  << std::endl;
        isRunning = false;
       }
       */

        /*if(cpt % (1000/period_ms) == 0) //c'est pas la condition optimale mais ça marche pour le moment
        {
            std::cout << "cpt : " << cpt << std::endl;
            std::cout << "rps : " << number_of_changes_A_channel/cpt << std::endl;
            rotations = number_of_changes_A_channel/cpt;
            //number_of_changes_A_channel = 0;
            //cpt = 0; 
        }
        */
        //std::cout << "########" << std::endl;

    }


    return 0;
}