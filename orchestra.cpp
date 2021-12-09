#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/double.h"
#include "hardware/clocks.h"
//#include "hardware/uart.h"

//----------------------my program-------------------------------------------------

#define ARRAY_SIZE(a) sizeof(a)/sizeof(a[0])
#define h 1.7
#define rx 22
#define tx 21
class Servo{
    protected:
    uint8_t pin;//given by the user in the constructor
    uint slice_num;//defined in ServoInit()
    volatile uint16_t msPosition;//calculated from position in the write() function
    uint16_t msServoMin = 550;//time in microseconds for 0 degrees
    uint16_t msServoMax = 2400;//time in microseconds for 180 degrees
    void ServoInit()
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
    void WriteMs()
    {
        // Set channel output high for one cycle before dropping
        pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), msPosition);
    }
    uint16_t map(float x, uint16_t sMin, uint16_t sMax, uint16_t dMin, uint16_t dMax )
    {
        return((x-(float)sMin) * (dMax - dMin) /(sMax - sMin) + dMin);
    }
    
    public:
    volatile float position;//position given by the user
    Servo(uint8_t chosen_pin=0)
    {
        if(chosen_pin<16)
        {
            this->pin = chosen_pin;
            ServoInit();
        }

    }
    void SlavePosition(float pos)
    {
        
        float alfa =  pos - 90;     
        float rad = 3.1415/180.0;
        float cosGamma = (h  - cos((float)alfa * rad));
        position = acos(cosGamma)/rad;
        float calculatedH = cos(alfa * rad) + sin(position * rad);
        for(int i = 0; i< 5; i++)    
        {
            if((calculatedH < h+0.1 || calculatedH > h - 0.1))
                break;
            cos(alfa*rad) + sin(position*rad);
            cosGamma += calculatedH - h;
            position = asin(cosGamma)/rad;
            calculatedH = cos(alfa * rad) + sin(position * rad);
        }                 
        write(position);
    }
    
    uint8_t Calculate(int pos)
    {
        float alfa =  pos - 90;     
        float rad = 3.1415/180.0;
        float cosGamma = (h  - cos((float)alfa * rad));
        position = acos(cosGamma)/rad;
        float calculatedH = cos(alfa * rad) + sin(position * rad);
        for(int i = 0; i< 5; i++)    
        {
            if((calculatedH < h+0.1 || calculatedH > h - 0.1))
                break;
            cos(alfa*rad) + sin(position*rad);
            cosGamma += calculatedH - h;
            position = asin(cosGamma)/rad;
            calculatedH = cos(alfa * rad) + sin(position * rad);
        }                
        return position;
    }
    void write(uint8_t newPosition)
    {
        if(newPosition<=180 && newPosition>=0)
        {
            this->position = newPosition;
            this->msPosition = map(this->position,0,180,msServoMin,msServoMax);
            WriteMs();
        }
    }
    
    void enable()
    {
        // Set the PWM running
        pwm_set_enabled(slice_num, true);
    }
};

class SlaveServo : public Servo{
    private:
    Servo master;
    

    public:
    SlaveServo(uint8_t chosen_pin = 0)
    {        
        this->pin = chosen_pin;
        ServoInit();
        position = 90; 
        enable();       
        WriteMs();
    }
    void attachMaster(Servo Master)
    {
        this->master = Master;
    }
    void SlavePosition()
    {         
        int8_t alfa =  master.position - 90;   
        float rad = 3.1415/180.0;
        float calculatedH = 0;
        for(int i = 0; i< 5; i++)    
        {
            if((calculatedH<h+0.1 || calculatedH > h - 0.1))
                break;
            cos(alfa*rad) + sin(position*rad);
            float cosGamma = (h  - cos((float)alfa * rad));
            position = asin(cosGamma)/rad;
            calculatedH = cos(alfa * rad) + sin(position * rad);
        }                     
        WriteMs();
    }
    
};
void UART_INIT(int baud)
{
    uart_init(uart0, baud);

    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(22, GPIO_FUNC_UART);
    gpio_set_function(21, GPIO_FUNC_UART);
}

int main() 
{
    UART_INIT(9600);
    uart_puts(uart0, "Hello world!");
    Servo mServos[6];
    Servo sServos[ARRAY_SIZE(mServos)];
    for(int i = 0; i< ARRAY_SIZE(mServos); i++)
    {
        mServos[i] = Servo(2*i);
        sServos[i] = Servo(2*i + 1);
    }
    
    // ser[0].write(90);
    Servo s(0);
    s.write(90);
    s.enable();
    Servo s2(2);
    s2.write(90);
    s2.enable();
    Servo s1(1);
    s1.write(90);
    s1.enable();
    Servo s3(1);
    s3.write(90);
    s3.enable();
    // SlaveServo ss(1);
    // ss.attachMaster(s);
    sleep_ms(2000);
    for(int i = 90; i<120; i++)
    {
        s.write(i);
        sleep_ms(100);
        s2.SlavePosition(i);
    }
    int LED_PIN = 15;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    // for(int i = 60; i<30; i--)
    // {
    //     ser[0].write(i);
    //     //sSer[0].SlavePosition();
    //     sleep_ms(100);
    // }
    while(1)
    {
        uart_puts(uart0, "Hello world!");
        gpio_put(LED_PIN,1);
        for(float i = 120; i>80; i-=0.5)
        {
            s.write(i);
            //sSer[0].SlavePosition();
            sleep_ms(25);
            s2.SlavePosition(i);
        }
        for(int i = s2.position; i>0; i--)
        {
            s2.write(i);
            sleep_ms(25);
        }
        gpio_put(LED_PIN,0);
        for(float i = 80; i<120; i+=0.5)
        {
            s.write(i);
            //sSer[0].SlavePosition();
            sleep_ms(25);            
        }
        for(int i = s2.position; i<s2.Calculate(s.position);i++)
        {
            s2.write(i);
            sleep_ms(25);  
        }
        sleep_ms(1000);
    }
}
//----------------------copied------------------------
// float clockDiv = 64;
// float wrap = 39062;

// void setMillis(int servoPin, float millis)
// {
//     pwm_set_gpio_level(servoPin, (millis/20000.f)*wrap);
// }

// void setServo(int servoPin, float startMillis)
// {
//     gpio_set_function(servoPin, GPIO_FUNC_PWM);
//     uint slice_num = pwm_gpio_to_slice_num(servoPin);

//     pwm_config config = pwm_get_default_config();
    
//     uint64_t clockspeed = clock_get_hz(clk_sys);//5
//     clockDiv = 64;
//     wrap = 39062;

//     while (clockspeed/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64; 
//     wrap = clockspeed/clockDiv/50;

//     pwm_config_set_clkdiv(&config, clockDiv);
//     pwm_config_set_wrap(&config, wrap);

//     pwm_init(slice_num, &config, true);

//     setMillis(servoPin, startMillis);
// }

// bool direction = true;
// int currentMillis = 400;
// int servoPin = 0;

// int main()
// {
//     setServo(servoPin, currentMillis);
//     while (true)
//     {
//         currentMillis += (direction)?5:-5;
//         if (currentMillis >= 2400) direction = false;
//         if (currentMillis <= 400) direction = true;
//         setMillis(servoPin, currentMillis);
//         sleep_ms(10);
//     }
// }
//---------------------------------------------------------------------------------