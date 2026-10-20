#ifndef DCMotor_h
#define DCMotor_h

#include <Arduino.h>

class DCMotor
{
    int en_r, en_l, pwm_r, pwm_l;

    void setupPins();
public:
    DCMotor();
    DCMotor(int, int, int, int);
    void moveMotor(int motorSpeed = 35);
};

#endif
