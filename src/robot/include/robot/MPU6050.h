#ifndef MPU6050_H
#define MPU6050_H

// The MPU6050 is both a gyroscope and an accelerometer.
// You can estimate the angular velocity of the robot and the acceleration of the robot

//ROS
#include "rclcpp/rclcpp.hpp"

//Publish the angular velocity values in rad/s
#include "geometry_msgs/msg/twist.hpp"


// Link as to why I have to put extern "C" :
// https://stackoverflow.com/questions/50154296/undefined-reference-to-i2c-smbus-read-word-dataint-unsigned-char
// Compile the code with  "g++ -o test.exe MPU6050.cpp -li2c", don't forget -li2c otherwise it will not work
extern "C"
{
    #include <i2c/smbus.h>
    #include <linux/i2c-dev.h>
}

//For snprintf
#include <cstdio>

//For exit function
#include <cstdlib>


//For open function and O_RDWR (the letter O not the digit 0)
#include <fcntl.h>


//For ioctl function
#include <sys/ioctl.h>

//For write function
#include <unistd.h>

//For the PI constant
#include <math.h>

//For the ms literal
#include <chrono>


//For calibration purposes
#include <vector>



#define NODE_NAME "mpu_publisher"
#define TOPIC_NAME "cmd_vel"
#define QUEUE_DEPTH 10


#define MPU6050_ADDRESS 0x68
#define WAKE_UP_VALUE 0x00

// ---------------------------- SENSOR OFFSETS -----------------------------------------

//Found by calibrating the sensor with no acceleration

#define ACC_X_OFFSET 5021
#define ACC_Y_OFFSET 1099

// (1 - estimated_mean_value_via_calibration*ACC_CONFIG_0_SCALE)*ACC_CONFIG_0_SCALE
#define ACC_Z_OFFSET 4319

#define YAW_OFFSET 35


// ---------------------------- REGISTERS -----------------------------------------

// Link to the register list of the MPU6050
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

#define POWER_MANAGEMENT_1_REGISTER 0x6B

// If we read the value of the WHO_AM_I_REGISTER, it should return the value of the MPU_ADDRESS i.e. 0x68
// Use it to verify the connection
#define WHO_AM_I_REGISTER 0x75 //register 117

#define FIFO_REGISTER            0x23 //register 35


// ----------------- Gyroscope registers -----------------

// Gyroscope for yaw rate estimation registers

#define GYRO_Z_OUT_HIGH_REGISTER 0x47 // register 71
#define GYRO_Z_OUT_LOW_REGISTER  0x48 // register 72

// Gyroscope configuration register 

#define GYRO_CONFIG_REGISTER     0x1B // register 27

//GYRO_CONFIG_X_SCALE is chosen by looking at the value of GYRO_CONFIG_REGISTER
#define GYRO_CONFIG_0_SCALE 131.0 // in LSB/°/second
#define GYRO_CONFIG_1_SCALE 65.5 // in LSB/°/second
#define GYRO_CONFIG_2_SCALE 32.8 // in LSB/°/second
#define GYRO_CONFIG_3_SCALE 16.4 // in LSB/°/second

// ----------------- Accelerometer registers -----------------

// Accelerometer configuration register
#define ACC_CONFIG_REGISTER 0x1C // register 28
#define ACC_CONFIG_0_SCALE 16384.0 // in LSB/g, ".0" to ensure that the result of a division is a double

#define ACC_X_OUT_HIGH_REGISTER  0x3B // register 59
#define ACC_X_OUT_LOW_REGISTER   0x3C // register 60
#define ACC_Y_OUT_HIGH_REGISTER  0x3D // register 61
#define ACC_Y_OUT_LOW_REGISTER   0x3E // register 62
#define ACC_Z_OUT_HIGH_REGISTER  0x3F // register 63
#define ACC_Z_OUT_LOW_REGISTER   0x40 // register 64



struct MPU6050 : public rclcpp::Node
{
    public:

        MPU6050();
        __u8 read_register(const int &register_address) const;
        void write_to_file(__s8 byte) const;
    
    private:

        // ------- I2C --------

        int _file = -1;
        int _i2c_port_number = 1;
        int _MAX_BUFFER_SIZE = 19;
        void init_i2c();

        // ------- ROS_PARAMETERS ------

        void init_ros_params();
        float _PUBLISH_RATE = 1000.0f;

        // ------- Compute measurements --------

        float compute_yaw_rate() const;
        float compute_acc_x() const;
        float compute_acc_y() const;
        float compute_acc_z() const;
    

        // ------- ROS PUBLISHER --------

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;
        rclcpp::TimerBase::SharedPtr _timer;
        void init_cmd_vel_publisher();

        void publish_yaw_rate() const;
        void publish_acc_x() const;

        // ------- Calibration --------

        void calibrate_acc(const int &ACC_OUT_HIGH_REGISTER, const int &ACC_OUT_LOW_REGISTER) const;

};  


#endif