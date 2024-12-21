
//    ---------------------------VERY IMPORTANT----------------------------------
//    Requires sudo to run
//    To make the code work, use the command "make_rpi4_work" after colcon build




#include <iostream>
#include <signal.h>
#include <csignal>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <thread>

//    Library for gpio control
//    pigpio must be installed
#include <pigpio.h>


//    The following headers are found in robot/include/robot/

#include "robot/rpi4.h"
#include "robot/encoders.h"


// Header to create a motor object
#include "robot/motor.h"


// Header to create a Robot_controller object

#include "robot/robot_controller.h"

#include "rclcpp/rclcpp.hpp"



//RPI4 rpi;
bool is_running = false;



// Raspberry PI4 pinout

#define M1A_GPIO 17
#define M1B_GPIO 18
#define M1A_CHANNEL 5
#define M1B_CHANNEL 6
#define M1_DUTY_CYCLE 255
#define LEFT_ENCODER_NAME "left_encoder"

#define M2A_GPIO 23
#define M2B_GPIO 24
#define M2A_CHANNEL 20
#define M2B_CHANNEL 21
#define M2_DUTY_CYCLE 0
#define RIGHT_ENCODER_NAME "right_encoder"

#define INTERVAL 1000 //milliseconds


void signalHandler(int signal)
{
    std::cout << "Received signal : " << signal << ". Shutting down" << std::endl;
    is_running = false;
}


/*

int main(){

    signal(SIGINT, signalHandler);
    

    
    //    rpi.init function works if the main object file is owned by root (can be created by youss, but owned by root afterwards using chmod)
    //    You can check permissions with ls -l /home/youss/Documents/Projet/Autonomouscar/ros_ws/build/robot/
    //    If it is not created by root (e.g. by youss), rpi.init will not work
    //    This file is not set in a ros environement, there are no nodes here so sudo resetting the environement won't matter for this file.
    //    However, it will be very annoying for other nodes like the motor_subscriber node


    if(!rpi.init())
    {
        std::cout << "Failed to init rpi4 - main.cpp" << std::endl;
        return -1;
    }
    

    //  Must create Motor objects AFTER calling rpi.init()
    //   Otherwise the gpioInitialise function of the pigpio library won't be called by rpi.init() and the code won't work
    
   
    Motor left_motor(M1A_GPIO, 
                    M1B_GPIO, 
                    M1_DUTY_CYCLE);

    Motor right_motor(M2A_GPIO, 
                    M2B_GPIO, 
                    M2_DUTY_CYCLE);

    
    
    //    Encoders have an ISR to detect rising edge

    Encoders encoder_A(M1A_CHANNEL,
                        M1B_CHANNEL,
                        LEFT_ENCODER_NAME);

    Encoders encoder_B(M2A_CHANNEL,
                        M2B_CHANNEL,
                        RIGHT_ENCODER_NAME);
    

        
//  std::thread encoder_A_thread(Encoders::compute_encoder_rps_handler, std::ref(encoder_A));
//  std::thread encoder_B_thread(Encoders::compute_encoder_rps_handler, std::ref(encoder_B));

    //encoder_A.set_rps(10);
    //encoder_B.set_rps(50);

    //encoder_A.start_encoder(INTERVAL);
    //encoder_B.start_encoder(INTERVAL);
    is_running = true;

    while (is_running)
    {
        
        //    Encoders revolutions per second will be printed out thanks to their ISR, even though there is no code
        
    }
    left_motor.stop();
    right_motor.stop();
    return 0;
}
*/



#include <iostream>
#include <pigpiod_if2.h>  // Include the header for pigpio API over sockets

int main() {
    signal(SIGINT, signalHandler);

    // Connect to the pigpio daemon on localhost (default address)
    int pi = pigpio_start(NULL, NULL);  // Connect to localhost with default port

    if (pi < 0) {
        std::cerr << "Unable to connect to pigpiod!" << std::endl;
        return -1;
    }

    //    Encoders have an ISR to detect rising edge

    /*
    Encoders encoder_A(M1A_CHANNEL,
                        M1B_CHANNEL,
                        LEFT_ENCODER_NAME);

    Encoders encoder_B(M2A_CHANNEL,
                        M2B_CHANNEL,
                        RIGHT_ENCODER_NAME);
    */
    set_mode(pi, M1A_GPIO, PI_OUTPUT);
    set_PWM_dutycycle(pi, M1A_GPIO, 50);
    is_running = true;
    while (is_running)
    {
                
    }
    set_PWM_dutycycle(pi, M1A_GPIO, 0);
    // Close the connection to the pigpio daemon
    pigpio_stop(pi);

    return 0;
}