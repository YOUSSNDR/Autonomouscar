#include <iostream>
#include <signal.h>
#include <csignal>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <thread>
/*
    Library for gpio control
    pigpio must be installed
*/
#include <pigpio.h>

/*
    The following headers are found in robot/include/robot/
*/
#include "robot/rpi4.h"
#include "robot/encoders.h"

RPI4 rpi;
bool is_running = false;

/*
    Raspberry PI4 pinout
*/

const unsigned int M1A_GPIO = 17;
const unsigned int M1B_GPIO = 18;
const unsigned int M1_A_channel = 5;
const unsigned int M1_B_channel = 6;
const unsigned int M1_duty_cycle = 200;

const unsigned int M2A_GPIO = 23;
const unsigned int M2B_GPIO = 24;
const unsigned int M2_A_channel = 20;
const unsigned int M2_B_channel = 21;
const unsigned int M2_duty_cycle = 200;

void signalHandler(int signal)
{
    std::cout << "Received signal : " << signal << ". Shutting down" << std::endl;
    is_running = false;
}



int main(){

    signal(SIGINT, signalHandler);
    const unsigned int interval = 1000; //milliseconds
    if(!rpi.init())
    {
        std::cout << "Failed to init rpi4 - main.cpp" << std::endl;
        return -1;
    }

    Encoders encoder_A(M1A_GPIO,
                        M1B_GPIO,
                        M1_A_channel,
                        M1_B_channel,
                        M1_duty_cycle);

    Encoders encoder_B(M2A_GPIO,
                        M2B_GPIO,
                        M2_A_channel,
                        M2_B_channel,
                        M2_duty_cycle);
    //std::thread encoder_A_thread(Encoders::compute_rps_handler, std::ref(encoder_A));
    //std::thread encoder_B_thread(Encoders::compute_rps_handler, std::ref(encoder_B));

    encoder_A.set_rps(10);
    encoder_B.set_rps(50);

    encoder_A.start_encoder(interval);
    encoder_B.start_encoder(interval);
    is_running = true;

    while (is_running)
    {

    }
    return 0;
}
