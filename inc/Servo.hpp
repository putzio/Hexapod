#pragma once
#include <stdio.h>
#include "pwm_driver.h"
// Servo motion parameters
#define SERVO_MIN_MS 550  // time in microseconds for 0 degrees
#define SERVO_MAX_MS 2400 // time in microseconds for 180 degrees
#define CYCLE_TIME 20     // ms
#define ACCELLERATION_PER_S 50.0
#define ACCELERATION 0.2 // ACCELERATION_PER_S*CYCLE_TIME *(SERVO_MAX_MS - SERVO_MIN_MS)/180
#define DECCELERATION 1
#define MAX_VELOCITY 20 // 20/(2400-550)/0.02 = 97*/s //in datasheet max speed -> 60*/0.1s = 600*/s
#define MIN_VELOCITY 1
#define DISTANCE_DECCELERATION 200
#define POS_90_TIME 200 // time for the leg to move to 90* before the next starts to move (to lower the current)

//
#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])
#define IS_BETWEEN(x, min, max) (x > min && x < max)
#define h 1.7 // max = 1,76, min = 1,5???

uint16_t map(float x, uint16_t sMin, uint16_t sMax, uint16_t dMin, uint16_t dMax)
{
    return ((x - (float)sMin) * (dMax - dMin) / (sMax - sMin) + dMin);
}
class Servo::
{
protected:
    uint8_t pin;       // given by the user in the constructor
    uint slice_num;    // defined in ServoInit() method form the pin variable
    bool left = false; // if the servo is on the other side it has to move the opposite way -> left = true
    float calibrationValue = 0;
    // msPosition:
    // calculated from position in the Write() method of destination
    //      OR
    // Changes value in the ChangePosition() method and then moves servo from currentPosition
    // to the changed one by using GoToPosition() method
    volatile uint16_t msPosition;
    float velocity;  // is changed and calculated in CalculateVelocity() method
    int maxVelocity; // privte?
    int minVelocity;
    uint16_t CalculateLeft(uint16_t pos)
    {
        int pos90 = map(90, 0, 180, SERVO_MIN_MS, SERVO_MAX_MS);
        // get the distance from 90 degrees
        int distance = pos90 - pos;
        // return another direction
        return pos90 + distance;
    }
    void ServoInit()
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
    void WriteMs()
    {
        if (left)
            pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), CalculateLeft(currentPosition) + calibrationValue);
        else
            // Set channel output high for one cycle before dropping
            pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), currentPosition + calibrationValue);
    }

public:
    bool done;
    float position; // position given by the user
    int currentPosition;

    Servo(uint8_t chosen_pin = 0, bool leftServo = false, int16_t calibration = 0)
    {
        velocity = MIN_VELOCITY;
        maxVelocity = MAX_VELOCITY;
        minVelocity = MIN_VELOCITY;
        currentPosition = (SERVO_MIN_MS + SERVO_MAX_MS) / 2;
        calibrationValue = calibration;
        done = true;
        if (chosen_pin < 16 && chosen_pin >= 2)
        {
            pin = chosen_pin;
            ServoInit();
            left = leftServo;
        }
    }
    void GoToPosition()
    {
        CalculateVelocity();
        if (!done)
        {
            // this->currentPosition+=((this->currentPosition-this->msPosition)/300);
            if (this->currentPosition > this->msPosition)
                this->currentPosition -= this->velocity;
            else
                this->currentPosition += this->velocity;

            // if(currentPosition - msPosition < velocity &&
            //    currentPosition - msPosition > -velocity)
            if (IS_BETWEEN(currentPosition - msPosition, -velocity, velocity))
                this->currentPosition = this->msPosition;

            // Move servo
            WriteMs();

            // If the servo has reached the final position move is done
            if (this->currentPosition == this->msPosition)
                done = true;
        }
    }

    void ChangeVelocityLimits(int v)
    {
        minVelocity = v;
        maxVelocity = 3 * v + 5;
    }
    void CalculateVelocity()
    {
        // if(currentPosition - msPosition < DISTANCE_DECCELERATION &&
        //     currentPosition - msPosition > - DISTANCE_DECCELERATION )
        if (IS_BETWEEN(currentPosition - msPosition, -DISTANCE_DECCELERATION, DISTANCE_DECCELERATION))
        {
            // deccelerate
            velocity -= DECCELERATION;
        }
        else
        {
            // accelerate
            velocity += ACCELERATION;
        }

        if (velocity < minVelocity)
            velocity = minVelocity;
        if (velocity > maxVelocity)
            velocity = maxVelocity;
    }
    void ChangePosition(uint8_t pos)
    {
        msPosition = map(pos, 0, 180, SERVO_MIN_MS, SERVO_MAX_MS);
        position = pos;
        done = false;
    }
    void Write(uint8_t newPosition)
    {
        if (newPosition <= 180 && newPosition >= 0)
        {
            this->position = newPosition;
            this->msPosition = map(this->position, 0, 180, SERVO_MIN_MS, SERVO_MAX_MS);
            currentPosition = msPosition;
            WriteMs();
        }
    }

    void Enable()
    {
        // Set the PWM running
        pwm_set_enabled(slice_num, true);
    }
    void Disable()
    {
        // Set the PWM running
        pwm_set_enabled(slice_num, false);
    }
};

class SlaveServo : public Servo
{
private:
    bool slaveBack = true;

public:
    bool enableSlave = false;
    SlaveServo(uint8_t chosen_pin = 0, bool leftServo = false, int16_t calibration = 0, bool sBack = false)
        : Servo(chosen_pin, leftServo, calibration)
    {
        slaveBack = sBack;
    }
    uint8_t Calculate(int masterPosition)
    {
        float alfa = masterPosition - 90.0;
        if (alfa < 0)
            alfa = -alfa;
        float rad = 3.1415 / 180.0;
        float sinPosNAlfa = (h - cos((float)alfa * rad));
        // position = asin(sinPosNAlfa)/rad - alfa;
        // float calculatedH = cos(alfa * rad) + sin((position + alfa) * rad);
        // sinPosNAlfa += (h - calculatedH);
        if (slaveBack)
            position = asin(sinPosNAlfa) / rad - alfa;
        else
            position = 180.0 - asin(sinPosNAlfa) / rad + alfa;
        return position;
    }
    // sets SlaveServo at the right position, so the leg heigth does not change
    void SlavePosition(float masterPosition)
    {
        if (enableSlave)
        {
            position = Calculate(masterPosition);
            Write(position);
        }
    }
};