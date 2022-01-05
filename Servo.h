#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/double.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
//Servo motion parameters
#define SERVO_MIN_MS 550//time in microseconds for 0 degrees
#define SERVO_MAX_MS 2400//time in microseconds for 180 degrees
#define CYCLE_TIME 20 //ms
#define ACCELLERATION_PER_S 50.0
#define ACCELERATION 0.2//ACCELERATION_PER_S*CYCLE_TIME *(SERVO_MAX_MS - SERVO_MIN_MS)/180
#define DECCELERATION 1
#define MAX_VELOCITY 20 //20/(2400-550)/0.02 = 97*/s //in datasheet max speed -> 60*/0.1s = 600*/s
#define MIN_VELOCITY 1
#define DISTANCE_DECCELERATION  200
#define POS_90_TIME 200 //time for the leg to move to 90* before the next starts to move (to lower the current)
class Servo{
    protected:
    uint8_t pin;//given by the user in the constructor
    uint slice_num;//defined in ServoInit() method form the pin variable
    bool left = false; //if the servo is on the other side it has to move the opposite way -> left = true
    
    //msPosition:
    //calculated from position in the Write() method of destination 
    //--------OR----- 
    //Changes value in the ChangePosition() method and then moves servo from currentPosition 
    //to the changed one by using GoToPosition() method
    volatile uint16_t msPosition;  
    float velocity = MIN_VELOCITY;// is changed and calculated in CalculateVelocity() method

    uint16_t CalculateLeft(uint16_t pos);
    void ServoInit();
    void WriteMs();
    
    public:
    bool done = true;
    bool enableSlave = false;
    bool slaveBack = true;
    volatile float position;//position given by the user
    int currentPosition = (SERVO_MIN_MS + SERVO_MAX_MS)/2;
    Servo(uint8_t chosen_pin=0, bool leftServo = false);    
    void GoToPosition();
    void CalculateVelocity();
    void ChangePosition(uint8_t pos);
    void SlavePosition(float pos);
    
    uint8_t Calculate(int pos);
    void Write(uint8_t newPosition);
    
    void Enable();
};