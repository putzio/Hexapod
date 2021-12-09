#include "S.h"

void Servo::ServoInit()
    {
        // Tell GPIO it is allocated to the PWM
        gpio_set_function(this->pin, GPIO_FUNC_PWM);
        // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
        this->slice_num = pwm_gpio_to_slice_num (this->pin);    
        // Set period of 4 cycles (0 to 3 inclusive)
        pwm_set_wrap(this->slice_num, 20000);
        //setting period = 20ms
        //set clk div to 38 
        pwm_set_clkdiv_int_frac(this->slice_num,125,9);
    }
void Servo::WriteMs()
    {
        // Set channel output high for one cycle before dropping
        pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), msPosition);
    }
uint16_t Servo::map(uint8_t x, uint16_t sMin, uint16_t sMax, uint16_t dMin, uint16_t dMax )
    {
        return((x-sMin) * (dMax - dMin) /(sMax - sMin) + dMin);
    }
Servo::Servo(uint8_t chosen_pin=0)
{
    if(chosen_pin<16)
        {
            this->pin = chosen_pin;
            ServoInit();
        }
}
void Servo::write(uint8_t newPosition)
    {
        if(newPosition<=180 && newPosition>=0)
        {
            this->position = newPosition;
            this->msPosition = map(this->position,0,180,msServoMin,msServoMax);
            WriteMs();
        }
    }
void Servo::enable()
    {
        // Set the PWM running
        pwm_set_enabled(slice_num, true);
    }