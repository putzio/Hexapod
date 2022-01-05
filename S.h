#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/double.h"
#include "hardware/clocks.h"

class Servos{
    protected:
    uint8_t pin;
    uint slice_num;
    volatile uint16_t msPosition;
    uint16_t msServoMin = 550;
    uint16_t msServoMax = 2400;
    void ServoInit();
    void WriteMs();
    uint16_t map(uint8_t x, uint16_t sMin, uint16_t sMax, uint16_t dMin, uint16_t dMax );

    public:
    volatile uint8_t position;
    Servos(uint8_t chosen_pin=0);
    void write(uint8_t newPosition);
    
    void enable();
};