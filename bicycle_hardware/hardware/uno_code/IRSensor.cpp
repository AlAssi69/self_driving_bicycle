#include "IRSensor.h"

IRSensor::IRSensor(void (*callbackFunction)(void), int inputPin = 2, int edge = RISING, int debounceDelay = 50)
{
    this->inputPin = inputPin;
    this->edge = edge;
    this->debounceDelay = debounceDelay;
    this->currentTime = this->lastTime = 0;
    this->callbackFunction = callbackFunction;

    pinMode(this->inputPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(this->inputPin), this->callbackFunction, this->edge);
}
