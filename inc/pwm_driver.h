#pragma once
#include <stdio.h>
#include "pico/stdlib.h"

class Pwm_driver
{
protected:
    uint8_t pin;       // given by the user in the constructor
    uint slice_num;    // defined in ServoInit() method form the pin variable
    void SetPwm(uint16_t value);
public:
    Pwm_driver(uint8_t pin);
    void InitPwm();
    void Enable();
    void Disable();
};