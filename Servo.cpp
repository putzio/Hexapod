    #include "Servo.h"
    uint16_t Servo::CalculateLeft(uint16_t pos)
    {
        int pos90 = map(90,0,180,SERVO_MIN_MS,SERVO_MAX_MS);
        //get the distance from 90 degrees
        int distance = pos90 - pos;
        //return another direction
        return pos90 + distance;
    }
    void Servo::ServoInit()
    {
        // Tell GPIO it is allocated to the PWM
        gpio_set_function(this->pin, GPIO_FUNC_PWM);
        // Find out which PWM slice is connected to GPIO 
        this->slice_num = pwm_gpio_to_slice_num (this->pin);    
        // Set period of 20000 cycles (0 to 20000 inclusive)
        pwm_set_wrap(this->slice_num, 20000);
        //setting period = 20ms
        //set clk div to 38 
        pwm_set_clkdiv_int_frac(this->slice_num,125,9);
    }
    void Servo::WriteMs()
    {
        if(left)
        pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), CalculateLeft(msPosition));
        else
        // Set channel output high for one cycle before dropping
        pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), msPosition);
        this->currentPosition = msPosition;
    }   

    Servo::Servo(uint8_t chosen_pin=0, bool leftServo = false)
    {
        if(chosen_pin<16 && chosen_pin>=2)
        {            
            this->pin = chosen_pin;
            ServoInit();
            this->left = leftServo;
        }

    }
    void Servo::GoToPosition()
    {
        CalculateVelocity();
        if(!done)
        {
            // this->currentPosition+=((this->currentPosition-this->msPosition)/300);                       
            if(this->currentPosition > this->msPosition)
                this->currentPosition-=this->velocity;
            else
                this->currentPosition+=this->velocity;
            
            // if(currentPosition - msPosition < velocity && 
            //    currentPosition - msPosition > -velocity)
            if(IS_BETWEEN(currentPosition-msPosition,-velocity,velocity))
                this->currentPosition = this->msPosition; 

            //Move servo
            if(left)
                pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), CalculateLeft(this->currentPosition));
            else
                pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), this->currentPosition);
            
            //If the servo has reached the final position move is done            
            if(this->currentPosition==this->msPosition)
                done = true;
        }        
    }
    void Servo::CalculateVelocity()
    {
        // if(currentPosition - msPosition < DISTANCE_DECCELERATION && 
        //     currentPosition - msPosition > - DISTANCE_DECCELERATION )
        if(IS_BETWEEN(currentPosition - msPosition,-DISTANCE_DECCELERATION,DISTANCE_DECCELERATION))
        {
            //deccelerate
            velocity -= DECCELERATION; 
        }
        else
        {
            //accelerate
            velocity +=ACCELERATION;
        }

        if(velocity<MIN_VELOCITY)
            velocity = MIN_VELOCITY;
        if(velocity>MAX_VELOCITY)
            velocity = MAX_VELOCITY;
    }
    void Servo::ChangePosition(uint8_t pos)
    {
        msPosition = map(pos,0,180,SERVO_MIN_MS,SERVO_MAX_MS);
        position = pos;
        done = false;
    }
    void Servo::SlavePosition(float pos)
    {
        if(enableSlave)
        {
            position = Calculate(pos);
            Write(position);         
        }        
    }
    
    uint8_t Servo::Calculate(int pos)
    {
        float alfa =  pos - 90.0;     
        if(alfa < 0)
            alfa = - alfa;
        float rad = 3.1415/180.0;
        float sinPosNAlfa = (h  - cos((float)alfa * rad));
        // position = asin(sinPosNAlfa)/rad - alfa;
        // float calculatedH = cos(alfa * rad) + sin((position + alfa) * rad); 
        // sinPosNAlfa += (h - calculatedH);
        if(slaveBack)
            position = asin(sinPosNAlfa)/rad - alfa;  
        else 
            position = 180.0 - asin(sinPosNAlfa)/rad + alfa;
        return position;
    }
    void Servo::Write(uint8_t newPosition)
    {
        if(newPosition<=180 && newPosition>=0)
        {            
            this->position = newPosition;
            this->msPosition = map(this->position,0,180,SERVO_MIN_MS,SERVO_MAX_MS);
            WriteMs();
        }
    }
    
    void Servo::Enable()
    {
        // Set the PWM running
        pwm_set_enabled(slice_num, true);
    }