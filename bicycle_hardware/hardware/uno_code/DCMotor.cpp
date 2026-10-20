#include "DCMotor.h"

void DCMotor::setupPins()
{
    pinMode(this->en_r, OUTPUT);
    pinMode(this->en_l, OUTPUT);
    pinMode(this->pwm_r, OUTPUT);
    pinMode(this->pwm_l, OUTPUT);

    digitalWrite(this->en_r, HIGH);
    digitalWrite(this->en_l, HIGH);
}

DCMotor::DCMotor()
{
    this->en_r = 8;
    this->en_l = 5;
    this->pwm_r = 3;
    this->pwm_l = 6;

    setupPins();
}

DCMotor::DCMotor(int en_r, int en_l, int pwm_r, int pwm_l)
{
    this->en_r = en_r;
    this->en_l = en_l;
    this->pwm_r = pwm_r;
    this->pwm_l = en_l;

    setupPins();
}

void DCMotor::moveMotor(int motorSpeed)
{
    // TODO: Fix the polarity
    if (motorSpeed < 0)
        motorSpeed = 0;
    // if (motorSpeed < 0)
    // {
    //     analogWrite(this->pwm_r, abs(motorSpeed));
    // }
    // else
    // {
    //     analogWrite(this->pwm_l, abs(motorSpeed));
    // }
    analogWrite(this->pwm_l, motorSpeed);
}
