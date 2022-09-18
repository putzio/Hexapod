#include "pwm_driver.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/double.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

void Pwm_driver::SetPwm(uint16_t value)
{
    pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), value);
}

Pwm_driver::Pwm_driver(uint8_t pin)
{
    if(pin<20 && pin >= 0){
        this->pin = pin;
        InitPwm();
    }
}

void Pwm_driver::InitPwm()
{
    // Tell GPIO it is allocated to the PWM
    gpio_set_function(this->pin, GPIO_FUNC_PWM);
    // Find out which PWM slice is connected to GPIO
    this->slice_num = pwm_gpio_to_slice_num(this->pin);
    // Set period of 20000 cycles (0 to 20000 inclusive)
    pwm_set_wrap(this->slice_num, 20000);
    // setting period = 20ms
    // set clk div to 38
    pwm_set_clkdiv_int_frac(this->slice_num, 125, 9);
}

void Pwm_driver::Enable()
{
    // Set the PWM running
    pwm_set_enabled(slice_num, true);
}

void Pwm_driver::Disable()
{
    // Reset the PWM
    pwm_set_enabled(slice_num, false);
}