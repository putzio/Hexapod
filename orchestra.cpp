#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/double.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

//----------------------my program-------------------------------------------------
#define UART_ID uart0
#define BAUD_RATE 9600

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define ARRAY_SIZE(a) sizeof(a)/sizeof(a[0])
#define h 1.8
#define ADC_V 5 *4.7/14.7
#define MIN_ADC_VALUE ADC_V*(1<<12)/3.3
//pins 
#define rx 1
#define tx 0
//Servos - 2-13
#define LED_VIN_LOW 26
#define EN_VIN_CHECK 27
#define VIN_CHECK_GPIO 28 
#define VIN_CHECK_A_INPUT VIN_CHECK_GPIO-26

#define LED_PIN 15
#define msServoMin 550//time in microseconds for 0 degrees
#define msServoMax 2400//time in microseconds for 180 degrees
enum State
    {
        Stop,
        Forward,
        ForwardLoop,
        Back,
        BackLoop,
        Up,
        Up1,
        UpLoop1,
        Up2,
        UpLoop2,
        Down,
        Down1,
        DownLoop1,
        Down2,
        DownLoop2,
        Left,
        Right,
        Reset
    };
State state = Stop;

State forwardStates[] = {
    Stop,
    Forward,
    ForwardLoop,
    Down1,
    DownLoop1,
    Down2,
    DownLoop2,
    Back,
    BackLoop,
    Up1,
    UpLoop1,
    Up2,
    UpLoop2
};
State backStates[] = {
    Stop,
    Forward,
    ForwardLoop,
    Up1,
    UpLoop1,
    Up2,
    UpLoop2,
    Back,
    BackLoop,
    Down1,
    DownLoop1,
    Down2,
    DownLoop2
};
State leftStates[] = {
    Stop,
    Right,//L1 Back
    ForwardLoop,
    Down1,
    DownLoop1,
    Down2,
    DownLoop2,
    Left,//L1 Forward
    BackLoop,
    Up1,
    UpLoop1,
    Up2,
    UpLoop2
};
State rightStates[] = {
    Stop,
    Right,
    ForwardLoop,
    Up1,
    UpLoop1,
    Up2,
    UpLoop2,
    Left,
    BackLoop,
    Down1,
    DownLoop1,
    Down2,
    DownLoop2
};
State resetStates[] = {
    Stop,
    Back,
    BackLoop,
    Down1,
    DownLoop1   
};
volatile bool enableProgram = true;
uint16_t map(float x, uint16_t sMin, uint16_t sMax, uint16_t dMin, uint16_t dMax )
    {
        return((x-(float)sMin) * (dMax - dMin) /(sMax - sMin) + dMin);
    }
void SendFloat(int x){
    // Get battery voltage from ADC result
    int batteryV = (int)(x * 330 *(147)/47.0/4096.0);
    char buff[5];      
    buff[0] = batteryV/100 + 48;
    buff[1] = '.';
    batteryV/=10;
    buff[2] = batteryV/10 + 48;
    buff[3] = batteryV%10 + 48;
    buff[4] = 'V';
    uart_puts(uart0, "\n");
    uart_puts(uart0, "Battery Voltage:\n");
    for(int i = 0 ; i < ARRAY_SIZE(buff) ;i++)
        {            
            uart_putc(uart0, (char)(buff[i]));            
        }
    uart_puts(uart0, "\n");
    
}
void on_uart_rx();
void UART_INIT(int baud)
{
    uart_init(uart0, baud);

    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(rx, GPIO_FUNC_UART);
    gpio_set_function(tx, GPIO_FUNC_UART);

    //interrupts
    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}

void on_uart_rx() {
    //----------getting the robot moves-----------------
    
    while (uart_is_readable(UART_ID)) {
        char mode = uart_getc(UART_ID);
        switch (mode)
        {
        case 'f':
            state = Forward;
            break;
        case 'b':
            state = Back;
            break;
        case 'l':
            state = Left;
            break;
        case 'r':
            state = Right;
            break;
        case 's':
            state = Stop;
            break;
        case 'R':
            state = Reset;
            break;
        }
    }
    //----------echo-------------------
    // char buffer[20];
    // int chars_rxed = 0;
    // while (uart_is_readable(UART_ID)) {
    //     buffer[chars_rxed] = uart_getc(UART_ID);
    //     chars_rxed++;
    //     if(chars_rxed>=20)
    //         break;
    // }
    // for(int i = 0; i <= chars_rxed; i++)
    //     if (uart_is_writable(UART_ID)) 
    //     {
    //         uart_putc(UART_ID, buffer[i]);
    //     }
    // uart_putc(UART_ID,10);
}

void ADC_INIT()
{           
    //init ADC
    adc_init();    
    // // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(VIN_CHECK_GPIO);
    // Select ADC input 0 (GPIO26)
    adc_select_input(VIN_CHECK_A_INPUT);
    //set LED and gpio, which turns on the transistor as output  
    gpio_init(LED_VIN_LOW);
    gpio_set_dir(LED_VIN_LOW, GPIO_OUT);
    gpio_put(LED_VIN_LOW,1);
    gpio_init(EN_VIN_CHECK);
    gpio_set_dir(EN_VIN_CHECK, GPIO_OUT);    
}
bool MeasureBattery()
{
    bool enableProgram;
    //turn on the transistor    
    gpio_put(EN_VIN_CHECK,1);    
    sleep_ms(100);
    //measure voltage   
    uint16_t result = adc_read();
    gpio_put(EN_VIN_CHECK,0);    
    //SendFloat(result);
    if(result < MIN_ADC_VALUE)
    {
        for(int i = 0; i < 3; i++)
        {
            gpio_put(LED_VIN_LOW,1);
            sleep_ms(500);
            gpio_put(LED_VIN_LOW,0);
            sleep_ms(500);
        }
        enableProgram = false;
    }
    else 
        enableProgram = true;
    return enableProgram;
}

class Servo{
    protected:
    uint8_t pin;//given by the user in the constructor
    bool left = false;
    uint slice_num;//defined in ServoInit()
    volatile uint16_t msPosition;//calculated from position in the write() function    
    
    uint16_t CalculateLeft(uint16_t pos)
    {
        int pos90 = map(90,0,180,msServoMin,msServoMax);
        //get the distance from 90 degrees
        int distance = pos90 - pos;
        //return another direction
        return pos90 + distance;
    }
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
        if(left)
        pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), CalculateLeft(msPosition));
        else
        // Set channel output high for one cycle before dropping
        pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), msPosition);
        this->currentPosition = msPosition;
    }   
    
    public:
    bool done = true;
    bool enableSlave = false;
    volatile float position;//position given by the user
    int currentPosition = 550;
    Servo(uint8_t chosen_pin=0, bool leftServo = false)
    {
        if(chosen_pin<16 && chosen_pin>=2)
        {            
            this->pin = chosen_pin;
            ServoInit();
            this->left = leftServo;
        }

    }
    void GoToPosition()
    {
        if(!done)
        {
            // this->currentPosition+=((this->currentPosition-this->msPosition)/300);                       
            if(this->currentPosition-this->msPosition > 0)
                this->currentPosition-=5;
            else
                this->currentPosition+=5;
            if(currentPosition-this->msPosition<5 && currentPosition-this->msPosition>-5)
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
        gpio_put(LED_PIN,done);
    }
    void ChangePosition(uint8_t pos)
    {
        msPosition = map(pos,0,180,msServoMin,msServoMax);
        position = pos;
        done = false;
        gpio_put(LED_PIN,done);
    }
    void SlavePosition(float pos)
    {
        if(enableSlave)
        {
            float alfa =  pos - 90;     
            float rad = 3.1415/180.0;
            float sinPosNAlfa = (h  - cos((float)alfa * rad));
            position = asin(sinPosNAlfa)/rad - alfa;
            float calculatedH = cos(alfa * rad) + sin((position + alfa) * rad);
            // for(int i = 0; i< 5; i++)    
            // {
            //     if((calculatedH < h+0.1 && calculatedH > h - 0.1))
            //         break;
            //     sinPosNAlfa = (h  - cos((float)alfa * rad)) + h - calculatedH;
            //     position = asin(sinPosNAlfa)/rad - alfa;
            //     calculatedH = cos(alfa * rad) + sin((position + alfa) * rad);
            // }                 
            sinPosNAlfa += (h - calculatedH);
            position = asin(sinPosNAlfa)/rad - alfa;
            write(position);
        }        
    }
    
    uint8_t Calculate(int pos)
    {
        float alfa =  pos - 90;     
        float rad = 3.1415/180.0;
        float sinPosNAlfa = (h  - cos((float)alfa * rad));
        position = asin(sinPosNAlfa)/rad - alfa;
        float calculatedH = cos(alfa * rad) + sin((position + alfa) * rad); 
        sinPosNAlfa += (h - calculatedH);
        position = asin(sinPosNAlfa)/rad - alfa;           
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

class Leg
{
    public:
    Servo master , slave;    
    
    void initLeg(int pinMaster, int pinSlave, bool leftLeg = false)
    {
        master = Servo(pinMaster, leftLeg);
        slave = Servo(pinSlave, leftLeg);
        master.write(90);
        master.enable();
        slave.SlavePosition(master.Calculate(90));
        slave.enable();
    }
    public: Leg(int pinMaster = 2, int pinSlave = false, bool leftLeg = false)
    {
        master = Servo(pinMaster, leftLeg);
        slave = Servo(pinSlave, leftLeg);
        master.write(90);
        master.enable();
        slave.SlavePosition(master.Calculate(90));
        slave.enable();
    }
    void writeMaster(int position, bool slaveEnabled)
    {
        master.write(position);
        if(slaveEnabled)
        {
            slave.enableSlave = true;
            slave.SlavePosition(position);
        }
        else
        {
            slave.enableSlave = false;
        }
    }
    void ChangePosition(uint8_t pos, bool slaveEnabled)
    {
        master.ChangePosition(pos);
        slave.enableSlave = slaveEnabled;
    }    
    void GoToPosition()
    {
        master.GoToPosition();
        slave.SlavePosition(map(master.currentPosition,msServoMin,msServoMax,0,180));
    }
    void ChangePositionSlave(uint8_t pos)
    {
        slave.ChangePosition(pos);
    }
    void GoToPositionSlave()
    {
        slave.GoToPosition();
    }
    void ChooseMove(State s, bool enableSlave)
    {
        switch(s)
        {
            case Forward:
            {
                ChangePosition(110,enableSlave);
                break;
            }
            case Down:
            {
                ChangePositionSlave(slave.Calculate(master.position));
                break;
            }
            case Back:
            {
                ChangePosition(90,enableSlave);
                break;
            }
            case Up:
            {
                ChangePositionSlave(0);
                break;
            }            
        }            
    }
    bool Move()
    {
        if(!master.done)
        {
            GoToPosition();
            return false;
        }
        else if (!slave.done)
        {
            GoToPositionSlave();
            return false;
        }
        else
            return true;
    }
    bool MoveDone()
    {
        if(master.done&&slave.done)
            return true;
        else
            return false;
    }
};
class Body
{
    public:
    Leg legs[6];
    uint8_t legTeam1[3] = {0,3,4};
    uint8_t legTeam2[3] = {1,2,5};   
    State movingStates[ARRAY_SIZE(forwardStates)];
    State moveType = Stop;
    int state = 0;
    Body(uint8_t masterPins[6], uint8_t slavePins[6])
    {
        for(int i = 0; i< 6; i++)
        {
            legs[i] = Leg(masterPins[i],slavePins[i],i%2 == 1);
        }        
    }

    void ChangeToForward()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = forwardStates [i];
        }
        moveType = Forward;
        state = 1;
    }
    void ChangeToBack()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = backStates [i];
        }
        moveType = Back;
        state = 1;
    }
    void ChangeToLeft()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = leftStates [i];
        }
        moveType = Left;
        state = 1;
    }
    void ChangeToRight()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = rightStates [i];
        }
        moveType = Right;
        state = 1;
    }
    void ChangeToStop()
    {
        state = 0;
        moveType = Stop;
    }
    void ChangeToReset()
    {
        for (int i = 0; i < ARRAY_SIZE(resetStates); i++)
        {
            movingStates[i] = resetStates [i];
        }
        moveType = Reset;
        state = 1;
    }
    void ResetPositionMove()
    {
        switch(movingStates[state])
        {
            case Stop:

            break;

            case Back:
            {
                //masters in the right position
                for(int i = 0; i<ARRAY_SIZE(legs); i++)
                {
                    legs[i].ChooseMove(Back, false);
                }
                state++;
                break;
            }
            case BackLoop:
            {
                if(MovesDone())
                    state++;
                break;
            }
            case Down1:
            {
                //Slaves down
                for(int i = 0; i<ARRAY_SIZE(legs); i++)
                {
                    legs[i].ChooseMove(Down, false);
                }
                state++;
                break;
            }
            case DownLoop1:
            {
                if(MovesDone())
                    state=0;
                break;
            }   
            
        }
        
        
    }
    void Move()
    {        
        switch(movingStates[state])
        {
            case Stop:
            {

                break;
            }
            case Reset:
            {
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].GoToPosition();
                if(MovesDone())
                    state = 0;
            }
            case Forward:
            {
                bool back = (moveType == Back);//Forward => false
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {                    
                    legs[legTeam1[i]].ChooseMove(Forward, back);//Forward => false
                    legs[legTeam2[i]].ChooseMove(Back, !back);//Forward => true
                }
                state++;
                break;
            }
            case ForwardLoop:
            {                
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].GoToPosition();
                if(MovesDone())
                    state++;         
                break;
            }
            case Down1:
            {
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {
                    legs[legTeam1[i]].ChooseMove(Down, false);
                }
                state++;
                break;
            }
            case DownLoop1:
            {
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].GoToPositionSlave();
                if(MovesDone())
                    state++;        
                break;
            }
            case Down2:
            {
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {
                    legs[legTeam2[i]].ChooseMove(Up, false);
                }
                state++;
                break;
            }
            case DownLoop2:
            {
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].GoToPositionSlave();
                if(MovesDone())
                    state++;        
                break;
            }
            case Back:
            {
                bool forward = (moveType == Forward);//Forward => true
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {
                    legs[legTeam1[i]].ChooseMove(Back, forward);//Forward => true
                    legs[legTeam2[i]].ChooseMove(Forward, !forward);//Forward => false
                }
                state++;
                break;
            }
            case BackLoop:
            {
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].GoToPosition();
                if(MovesDone())
                    state++;        
                break;
            }
            case Up1:
            {
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {
                    legs[legTeam2[i]].ChooseMove(Down, false);
                }
                state++;
                break;
            }
            case UpLoop1:
            {
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].GoToPositionSlave();
                if(MovesDone())
                    state++;         
                break;
            }
            case Up2:
            {
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {
                    legs[legTeam1[i]].ChooseMove(Up, false);
                }
                state++;
                break;
            }
            case UpLoop2:
            {
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].GoToPositionSlave();
                if(MovesDone())
                    state++;         
                break;
            }

            //Left/Right:
            case  Left:
            {
                
                bool right = (moveType == Right);
                legs[0].ChooseMove(Forward, !right);
                legs[1].ChooseMove(Forward, right);
                legs[2].ChooseMove(Back, right);
                legs[3].ChooseMove(Back, !right);
                legs[4].ChooseMove(Forward, !right);
                legs[5].ChooseMove(Forward, right);
                state++;
                break;
            }
            case  Right:
            {
                bool right = (moveType == Right);
                legs[0].ChooseMove(Back, right);
                legs[1].ChooseMove(Back, !right);
                legs[2].ChooseMove(Forward, !right);
                legs[3].ChooseMove(Forward, right);
                legs[4].ChooseMove(Back, right);
                legs[5].ChooseMove(Back, !right);
                state++;
                break;
            }
        }
        if(state >= ARRAY_SIZE(movingStates))
            state = 1;
    }
    bool MovesDone()
    {
        for(int i = 0; i< ARRAY_SIZE(legs);i++)
        {
            if(!legs[i].MoveDone())
                return false;
        }
        return true;
    }
    void StateChanged(State s)
    {        
        switch(s)
        {
            case Stop:
            {
                ChangeToStop();
                break;
            }
            case Reset:
            {
                ChangeToReset();
                break;
            }
            case Forward:
            {
                ChangeToForward();
                break;
            }
            case Back:
            {
                ChangeToBack();
                break;
            }
            case Left:
            {
                ChangeToLeft();
                break;
            }
            case Right:
            {
                ChangeToRight();
                break;
            }
        }
    }
};
int main() 
{
    // Servo mServos[2];
    
    // mServos[0]= Servo(2,false);
    // mServos[0].write(90);
    // mServos[0].enable();

    // mServos[1]= Servo(4,true);
    // mServos[1].write(90);
    // mServos[1].enable();

    // while(1)
    // {
    //     sleep_ms(1000);
    // }
    stdio_init_all();
    UART_INIT(9600);
    uart_puts(uart0, "Hello world!\n");
    
    ADC_INIT();
    MeasureBattery();
    
    // Servo mServos[6];
    // Servo sServos[ARRAY_SIZE(mServos)];
    // Leg legs[ARRAY_SIZE(mServos)];
    uint8_t mser[6],sser[6];
    for(int i = 0; i < ARRAY_SIZE(mser); i++)
    {
        // mServos[i] = Servo(2*i);
        // mServos[i].write(90);
        // mServos[i].enable();
        // sServos[i] = Servo(2*i + 1);
        // sServos[i].enableSlave = true;
        // sServos[i].SlavePosition(90);
        // sServos[i].enable();
        // legs[i].initLeg(2*i,2*i + 1);
        mser[i] = 2*(i + 1);//(i+1) because we start at gpio 2
        sser[i] = 2*(i + 1) + 1;
    }
    // uart_puts(uart0, "1\n");
    sleep_ms(2000);
    Body body(mser,sser);
    // uart_puts(uart0, "2\n");
    // sleep_ms(2000);
    body.StateChanged(state);
    
    gpio_init(16);
    gpio_init(25);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_set_dir(25, GPIO_OUT);
    // uart_puts(uart0, "3\n");
    while(1)
    {
        // gpio_put(LED_PIN,1);
        if(!enableProgram)
            gpio_put(25,1);
        gpio_put(16,1);
        
        do
        {
            body.Move();
            sleep_ms(20); 
        }
        while(!body.MovesDone());
        gpio_put(25,0);
        gpio_put(16,0);
        // enableProgram = MeasureBattery();
        if(state != body.moveType)
        {
            body.StateChanged(state);
        }
        sleep_ms(100);

    }
}


/*
1. włącz/wyłącz slave w zależności od up, down
2. 

tryby pracy:
łydka góra/dół
->włącz/wyłącz slave w zależności od up, down
->Set/Reset enableSlave

udo przód/tył

F,B,L,R
L[0] up, L[1] down...
foreach(leg)
switch(state)
{

}
*/
