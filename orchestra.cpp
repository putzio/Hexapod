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

#define ARRAY_SIZE(a) sizeof(a)/sizeof(a[0])
#define IS_BETWEEN(x,min,max) (x>min&&x<max)
#define h 1.7

//pins 
#define RX_PIN 1
#define TX_PIN 0
//Servos - 2-13
#define LED_VIN_LOW 26
#define EN_VIN_CHECK 27
#define VIN_CHECK_GPIO 28 
#define VIN_CHECK_A_INPUT VIN_CHECK_GPIO-26

#define LED_PIN 15

//Servo motion parameters
#define SERVO_MIN_MS 550//time in microseconds for 0 degrees
#define SERVO_MAX_MS 2400//time in microseconds for 180 degrees
#define CYCLE_TIME 20 //ms
#define ACCELLERATION_PER_S 50.0
#define ACCELERATION 0.2//ACCELERATION_PER_S*CYCLE_TIME *(SERVO_MAX_MS - SERVO_MIN_MS)/180
#define DECCELERATION 1
#define MAX_VELOCITY 20 //20/(2400-550)/0.02 = 97*/s //in datasheet max speed -> 60*/0.1s = 600*/s
#define MIN_VELOCITY 1
#define DISTANCE_DECCELERATION  100
#define POS_90_TIME 200 //time for the leg to move to 90* before the next starts to move (to lower the current)

//Measure Battery
#define RESISTOR_RATIO (147.0/47.0)
#define RP_VOLTAGE 3.3
#define ADC_MAX (1<<12)
#define ADC_V 5 *4.7/14.7
#define MIN_ADC_VALUE ADC_V*ADC_MAX/RP_VOLTAGE
#define MASTER_SERVO_MIN_POS 100
#define MASTER_SERVO_MAX_POS 130
#define SLAVE_UP_POSITION 0
//States 
enum State
    {
        Stop,
        Forward,
        MasterLoop,
        SlaveLoop,
        Back,
        Up,
        Up1,
        Up2,
        Down,
        Down1,
        Down2,
        Left,
        Right,
        ResetMaster,
        ResetSlave
    };
enum Mode 
{
    StopMode,
    ForwardMode,
    BackMode,
    LeftMode,
    RightMode,
    ResetMode,
    Pos90Mode
};
Mode state = StopMode;
State forwardStates[] = {
    Stop,
    Forward,
    MasterLoop,
    Down1,
    SlaveLoop,
    Down2,
    SlaveLoop,
    Back,
    MasterLoop,
    Up1,
    SlaveLoop,
    Up2,
    SlaveLoop
};
State backStates[] = {
    Stop,
    Forward,
    MasterLoop,
    Up1,
    SlaveLoop,
    Up2,
    SlaveLoop,
    Back,
    MasterLoop,
    Down1,
    SlaveLoop,
    Down2,
    SlaveLoop
};
State leftStates[] = {
    Stop,
    Right,//L1 Back
    MasterLoop,
    Down1,
    SlaveLoop,
    Down2,
    SlaveLoop,
    Left,//L1 Forward
    MasterLoop,
    Up1,
    SlaveLoop,
    Up2,
    SlaveLoop
};
State rightStates[] = {
    Stop,
    Right,
    MasterLoop,
    Up1,
    SlaveLoop,
    Up2,
    SlaveLoop,
    Left,
    MasterLoop,
    Down1,
    SlaveLoop,
    Down2,
    SlaveLoop
};
State resetStates[] = {
    Stop,
    ResetMaster,
    MasterLoop,
    ResetSlave,
    SlaveLoop,
    Stop   
};
volatile bool enableProgram = true;
uint16_t map(float x, uint16_t sMin, uint16_t sMax, uint16_t dMin, uint16_t dMax )
    {
        return((x-(float)sMin) * (dMax - dMin) /(sMax - sMin) + dMin);
    }
void SendFloat(int x){
    // Get battery voltage from ADC result
    int batteryV = (int)(x * 100 * RP_VOLTAGE*RESISTOR_RATIO/(float)ADC_MAX);
    char buff[5];      
    buff[0] = batteryV/100 + '0';
    buff[1] = '.';
    batteryV/=10;
    buff[2] = batteryV/10 + '0';
    buff[3] = batteryV%10 + '0';
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
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);

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
        uart_puts(uart0, "Recieved:\n");
        uart_putc(uart0,mode);
        uart_puts(uart0, "\n");
        switch (mode)
        {
        case 'f':
            state = ForwardMode;
            break;
        case 'b':
            state = BackMode;
            break;
        case 'l':
            state = LeftMode;
            break;
        case 'r':
            state = RightMode;
            break;
        case 's':
            state = StopMode;
            break;
        case 'R':
            state = ResetMode;
            break;
        case 'P':
            state = Pos90Mode;
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
    uint slice_num;//defined in ServoInit() method form the pin variable
    bool left = false; //if the servo is on the other side it has to move the opposite way -> left = true
    
    //msPosition:
    //calculated from position in the Write() method of destination 
    //--------OR----- 
    //Changes value in the ChangePosition() method and then moves servo from currentPosition 
    //to the changed one by using GoToPosition() method
    volatile uint16_t msPosition;  
    float velocity = MIN_VELOCITY;// is changed and calculated in CalculateVelocity() method

    uint16_t CalculateLeft(uint16_t pos)
    {
        int pos90 = map(90,0,180,SERVO_MIN_MS,SERVO_MAX_MS);
        //get the distance from 90 degrees
        int distance = pos90 - pos;
        //return another direction
        return pos90 + distance;
    }
    void ServoInit()
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
    bool slaveBack = true;
    float position;//position given by the user
    int currentPosition = (SERVO_MIN_MS + SERVO_MAX_MS)/2;
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
    void CalculateVelocity()
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
    void ChangePosition(uint8_t pos)
    {
        msPosition = map(pos,0,180,SERVO_MIN_MS,SERVO_MAX_MS);
        position = pos;
        done = false;
    }
    void SlavePosition(float pos)
    {
        if(enableSlave)
        {
            position = Calculate(pos);
            Write(position);         
        }        
    }
    
    uint8_t Calculate(int pos)
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
    void Write(uint8_t newPosition)
    {
        if(newPosition<=180 && newPosition>=0)
        {            
            this->position = newPosition;
            this->msPosition = map(this->position,0,180,SERVO_MIN_MS,SERVO_MAX_MS);
            WriteMs();
        }
    }
    
    void Enable()
    {
        // Set the PWM running
        pwm_set_enabled(slice_num, true);
    }
};

class Leg
{
    public:
    Servo master , slave;    
    int maxPos = MASTER_SERVO_MAX_POS;
    int minPos = MASTER_SERVO_MIN_POS;
    int upPos = SLAVE_UP_POSITION;

    void initLeg()
    {
        master.Enable();
        master.Write(90);
        slave.enableSlave = true;
        slave.Enable();
        slave.SlavePosition(master.position);        
    }
    public: Leg(int pinMaster = 2, int pinSlave = 3, bool leftLeg = false)
    {
        master = Servo(pinMaster, leftLeg);
        slave = Servo(pinSlave, leftLeg);
    }
    void WriteMaster(int position, bool slaveEnabled)
    {
        master.Write(position);
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
        slave.SlavePosition(map(master.currentPosition,SERVO_MIN_MS,SERVO_MAX_MS,0.0,180.0));
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
                ChangePosition(maxPos,enableSlave);
                break;
            }
            case Down:
            {
                ChangePositionSlave(slave.Calculate(master.position));
                break;
            }
            case Back:
            {
                ChangePosition(minPos,enableSlave);
                break;
            }
            case Up:
            {
                ChangePositionSlave(upPos);
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
    Mode moveType = StopMode;
    int step = 0;
    bool reset = false;
    Body(uint8_t masterPins[6], uint8_t slavePins[6])
    {
        for(int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            legs[i] = Leg(masterPins[i],slavePins[i],i%2 == 1);            
        }    

        //The other way for the first 2 legs
        for(int i = 0; i< 2; i++)
        {
            legs[i].maxPos = 180 - MASTER_SERVO_MIN_POS;
            legs[i].minPos = 180 - MASTER_SERVO_MAX_POS;
            legs[i].upPos = 180 - SLAVE_UP_POSITION;
            legs[i].slave.slaveBack = false;
        }       
        for(int i = 0; i< ARRAY_SIZE(legs);i++)
        {
            legs[i].initLeg();
            sleep_ms(POS_90_TIME);
        } 
    }

    void ChangeToForward()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = forwardStates [i];
        }
        moveType = ForwardMode;
        step = 1;
        if(reset == false)
        {
            ChangeToResetTemp();
        }
        else
            reset = false;
    }
    void ChangeToBack()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = backStates [i];
        }
        moveType = BackMode;
        step = 1;
        if(reset == false)
        {
            ChangeToResetTemp();
        }
        else
            reset = false;
    }
    void ChangeToLeft()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = leftStates [i];
        }
        moveType = LeftMode;
        step = 1;
        if(reset == false)
        {
            ChangeToResetTemp();
        }
        else
            reset = false;
    }
    void ChangeToRight()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = rightStates [i];
        }
        moveType = RightMode;
        step = 1;
        if(reset == false)
        {
            ChangeToResetTemp();
        }
        else
            reset = false;
    }
    void ChangeToStop()
    {
        step = 0;
        moveType = StopMode;
    }
    void ChangeToReset()
    {
        for (int i = 0; i < ARRAY_SIZE(resetStates); i++)
        {
            movingStates[i] = resetStates [i];
        }
        moveType = ResetMode;
        step = 1;
    }
    void ChangeToResetTemp()
    {        
        for (int i = 0; i < ARRAY_SIZE(resetStates); i++)
        {
            movingStates[i] = resetStates [i];
        }
        step = 1;
    }
    void ChangeTo90()
    {
        step = 0;
        moveType = Pos90Mode;
        for(int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            legs[i].master.Write(90);
            legs[i].slave.Write(90);
            sleep_ms(POS_90_TIME);
        }
    }

    void Move()
    {        
        switch(movingStates[step])
        {
            case Stop:
            {
                //if ChangeToResetTemp(), the robot is already reset, so we can move on
                if(moveType != ResetMode && reset == true)
                {
                    StateChanged(moveType);
                }

                break;
            }
            case ResetMaster:
            {
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].ChooseMove(Back,false);
                step++;
                break;
            }
            case ResetSlave:
            {
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].ChooseMove(Down, false);
                step++;
                reset = true;
                break;
            }
            case Forward:
            {
                bool back = (moveType == BackMode);//Forward => false
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {                    
                    legs[legTeam1[i]].ChooseMove(Forward, back);//Forward => false
                    legs[legTeam2[i]].ChooseMove(Back, !back);//Forward => true
                }
                step++;
                break;
            }
            case MasterLoop:
            {                
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].GoToPosition();
                if(MovesDone())
                    step++;         
                break;
            }
            case Down1:
            {
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {
                    legs[legTeam1[i]].ChooseMove(Down, false);
                }
                step++;
                break;
            }
            case SlaveLoop:
            {
                for(int i = 0; i< ARRAY_SIZE(legs);i++)
                    legs[i].GoToPositionSlave();
                if(MovesDone())
                    step++;        
                break;
            }
            case Down2:
            {
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {
                    legs[legTeam2[i]].ChooseMove(Up, false);
                }
                step++;
                break;
            }
            case Back:
            {
                bool forward = (moveType == ForwardMode);//Forward => true
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {
                    legs[legTeam1[i]].ChooseMove(Back, forward);//Forward => true
                    legs[legTeam2[i]].ChooseMove(Forward, !forward);//Forward => false
                }
                step++;
                break;
            }
            case Up1:
            {
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {
                    legs[legTeam2[i]].ChooseMove(Down, false);
                }
                step++;
                break;
            }
            case Up2:
            {
                for(int i = 0; i<ARRAY_SIZE(legTeam1); i++)
                {
                    legs[legTeam1[i]].ChooseMove(Up, false);
                }
                step++;
                break;
            }

            //Left/Right:
            case  Left:
            {
                
                bool right = (moveType == RightMode);
                legs[0].ChooseMove(Forward, !right);
                legs[1].ChooseMove(Forward, right);
                legs[2].ChooseMove(Back, right);
                legs[3].ChooseMove(Back, !right);
                legs[4].ChooseMove(Forward, !right);
                legs[5].ChooseMove(Forward, right);
                step++;
                break;
            }
            case  Right:
            {
                bool right = (moveType == RightMode);
                legs[0].ChooseMove(Back, right);
                legs[1].ChooseMove(Back, !right);
                legs[2].ChooseMove(Forward, !right);
                legs[3].ChooseMove(Forward, right);
                legs[4].ChooseMove(Back, right);
                legs[5].ChooseMove(Back, !right);
                step++;
                break;
            }
        }
        if(step >= ARRAY_SIZE(movingStates))
            step = 1;
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
    void StateChanged(Mode s)
    {        
        switch(s)
        {
            case StopMode:
            {
                ChangeToStop();
                break;
            }
            case ResetMode:
            {
                ChangeToReset();
                break;
            }
            case ForwardMode:
            {
                ChangeToForward();
                break;
            }
            case BackMode:
            {
                ChangeToBack();
                break;
            }
            case LeftMode:
            {
                ChangeToLeft();
                break;
            }
            case RightMode:
            {
                ChangeToRight();
                break;
            }
            case Pos90Mode:
            {
                ChangeTo90();
                break;                
            }
        }
    }
};
int main() 
{
    stdio_init_all();

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    // gpio_put(25,1);
    // sleep_ms(500);
    UART_INIT(9600);
    uart_puts(uart0, "Hello world!\n");

    // gpio_put(25,0);
    // sleep_ms(500);
    
    ADC_INIT();
    MeasureBattery();
    
    // gpio_put(25,1);
    // sleep_ms(500);

    uint8_t mser[6],sser[6];
    for(int i = 0; i < ARRAY_SIZE(mser); i++)
    {
        mser[i] = 2*(i + 1);//(i+1) because we start at gpio 2
        sser[i] = 2*(i + 1) + 1;
    }
    // sleep_ms(2000);
    
    // gpio_put(25,0);
    // sleep_ms(500);

    Body body(mser,sser);
    // body.legs[0].slave.Write(0);
    // sleep_ms(1000);
    // body.legs[0].slave.Write(90);
    state = ResetMode;
    body.StateChanged(state);
    
    gpio_init(16);
    // gpio_put(25,1);
    // sleep_ms(500);
    while(1)
    {
        // uart_puts(uart0, "Recieved:\n");
        // uart_putc(uart0,body.step + 48);
        // uart_puts(uart0, "\n");
        gpio_put(25,1);
        gpio_put(16,1);
        // int i = 0;
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
        sleep_ms(500);

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
