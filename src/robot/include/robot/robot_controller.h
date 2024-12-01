#include "rclcpp/rclcpp.hpp"

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

struct Robot_controller : public rclcpp::Node{
    public:
        Robot_controller();
    private:
        int a = 2;
        int b = 3;
};

#endif