#ifndef IRSensor_h
#define IRSensor_h

#include <Arduino.h>

class IRSensor
{
    void (*callbackFunction)(void);

public:
    int inputPin, edge, debounceDelay;
    long currentTime, lastTime;

    IRSensor(void (*callbackFunction)(void), int inputPin = 2, int edge = RISING, int debounceDelay = 50);
};

#endif
