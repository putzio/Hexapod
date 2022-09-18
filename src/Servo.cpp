#include "Servo.h"
#include "pico/double.h"

uint16_t Servo::CalculateLeft(uint16_t pos)
{
    int pos90 = map(90, 0, 180, SERVO_MIN_MS, SERVO_MAX_MS);
    // get the distance from 90 degrees
    int distance = pos90 - pos;
    // return another direction
    return pos90 + distance;
}
void Servo::WriteMs()
{
    if (left)
        SetPwm(CalculateLeft(currentPosition) + calibrationValue);
    else
        // Set channel output high for one cycle before dropping
        SetPwm(currentPosition + calibrationValue);
}

Servo::Servo(uint8_t chosen_pin, bool leftServo, int16_t calibration)
:Pwm_driver(chosen_pin)
{
    velocity = MIN_VELOCITY;
    maxVelocity = MAX_VELOCITY;
    minVelocity = MIN_VELOCITY;
    currentPosition = (SERVO_MIN_MS + SERVO_MAX_MS) / 2;
    calibrationValue = calibration;
    done = true;
}

void Servo::GoToPosition()
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

void Servo::ChangeVelocityLimits(int v)
{
    minVelocity = v;
    maxVelocity = 3 * v + 5;
}
void Servo::CalculateVelocity()
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
void Servo::ChangePosition(uint8_t pos)
{
    msPosition = map(pos, 0, 180, SERVO_MIN_MS, SERVO_MAX_MS);
    position = pos;
    done = false;
}
void Servo::Write(uint8_t newPosition)
{
    if (newPosition <= 180 && newPosition >= 0)
    {
        this->position = newPosition;
        this->msPosition = map(this->position, 0, 180, SERVO_MIN_MS, SERVO_MAX_MS);
        currentPosition = msPosition;
        WriteMs();
    }
}

SlaveServo::SlaveServo(uint8_t chosen_pin, bool leftServo, int16_t calibration, bool sBack)
    : Servo(chosen_pin, leftServo, calibration)
{
    slaveBack = sBack;
}
uint8_t SlaveServo::Calculate(int masterPosition)
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
void SlaveServo::SlavePosition(float masterPosition)
{
    if (enableSlave)
    {
        position = Calculate(masterPosition);
        Write(position);
    }
}

uint16_t map(float x, uint16_t sMin, uint16_t sMax, uint16_t dMin, uint16_t dMax)
{
    return ((x - (float)sMin) * (dMax - dMin) / (sMax - sMin) + dMin);
}