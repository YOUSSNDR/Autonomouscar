#ifndef RPI4_H
#define RPI4_H
#include <iostream>

/*
    Library for gpio control,
    pigpio must be installed
*/
#include <pigpio.h>

struct RPI4{
    public:
        RPI4();
        bool init();

};

#endif