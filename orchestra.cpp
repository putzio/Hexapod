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
#define h 1.7 // max = 1,76, min = 1,5???

//pins 
#define RX_PIN 1
#define TX_PIN 0
//Servos - 2-13
#define LED_VIN_LOW 26
#define EN_VIN_CHECK 27
#define VIN_CHECK_GPIO 28 
#define VIN_CHECK_A_INPUT (VIN_CHECK_GPIO-26)
#define BUILD_IN_LED 25

#define LED_PIN 15

//Servo motion parameters
#define SERVO_MIN_MS 550//time in microseconds for 0 degrees
#define SERVO_MAX_MS 2400//time in microseconds for 180 degrees
#define CYCLE_TIME 20 //ms
#define ACCELLERATION_PER_S 50.0
#define ACCELERATION 0.5//ACCELERATION_PER_S*CYCLE_TIME *(SERVO_MAX_MS - SERVO_MIN_MS)/180
#define DECCELERATION 1
#define MAX_VELOCITY 20 //20/(2400-550)/0.02 = 97*/s //in datasheet max speed -> 60*/0.1s = 600*/s
#define MIN_VELOCITY 4
#define DISTANCE_DECCELERATION  100
#define POS_90_TIME 200 //time for the leg to move to 90* before the next starts to move (to lower the current)

//Measure Battery
#define RESISTOR_3  4700 //kOHM
#define RESISTOR_2 10000 //kOHM
#define RP_VOLTAGE 3.3 //V
#define MIN_BATTERY_VOLATAGE 7.0 //V
#define ADC_MAX 4096 //12-bit
#define RESISTOR_3_MIN_VOLTAGE (MIN_BATTERY_VOLATAGE / (RESISTOR_2 + RESISTOR_3) * RESISTOR_3)
#define MIN_ADC_VALUE (RESISTOR_3_MIN_VOLTAGE * ADC_MAX / RP_VOLTAGE)

//SERVO POSITIONS
#define MASTER_SERVO_MIN_POS 100
#define MASTER_SERVO_MAX_POS 130
#define SLAVE_UP_POSITION 5
const int16_t SERVO_CALIB[12] ={
    155,
    -60,
    -155,
    -80,
    20,
    95,
    -30,
    60,
    55,
    -70,
    90,
    -50
};
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
        ResetSlave,
    };
enum Mode 
{
    StopMode,
    ForwardMode,
    BackMode,
    LeftMode,
    RightMode,
    ResetMode,
    Pos90Mode,
};
Mode mode = Pos90Mode;
const State forwardStates[] = {
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
const State backStates[] = {
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
const State leftStates[] = {    
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
const State rightStates[] = {
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
const State resetStates[] = {
    Stop,
    ResetMaster,
    MasterLoop,
    ResetSlave,
    SlaveLoop,
    Stop   
};
const State pos90States[] = {
    Stop,
    SlaveLoop,
    Stop
};

//volatile bool enableProgram = true;
uint16_t map(float x, uint16_t sMin, uint16_t sMax, uint16_t dMin, uint16_t dMax )
    {
        return((x-(float)sMin) * (dMax - dMin) /(sMax - sMin) + dMin);
    }

void OnUartRx();
void UART_INIT()
{
    uart_init(uart0, BAUD_RATE);

    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);

    //interrupts
    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, OnUartRx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}
bool changeVelocity = false;
int velocity = MIN_VELOCITY;
bool velocityChanged = false;
void OnUartRx() {
    //----------getting the robot moves-----------------
    
    while (uart_is_readable(UART_ID)) {
        char recMode = uart_getc(UART_ID);
        uart_puts(uart0, "Recieved:\n");
        uart_putc(uart0,recMode);
        uart_puts(uart0, "\n");
        if(changeVelocity)
        {
            if(recMode == '&')
            {
                velocityChanged = true;
                changeVelocity = false;
            }
            else
            {
                velocity *=10;
                velocity += (int)recMode - 48;
            }
            
        }
        else
        {
            switch (recMode)
            {
                case 'f':
                    mode = ForwardMode;
                    break;
                case 'b':
                    mode = BackMode;
                    break;
                case 'l':
                    mode = LeftMode;
                    break;
                case 'r':
                    mode = RightMode;
                    break;
                case 's':
                    mode = StopMode;
                    break;
                case 'R':
                    mode = ResetMode;
                    break;
                case 'P':
                    mode = Pos90Mode;
                    break;
                case 'V':
                {
                    changeVelocity = true;
                    velocity = 0;
                    break;
                }                
            }
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
    //gpio_put(LED_VIN_LOW,1);
    gpio_init(EN_VIN_CHECK);
    gpio_set_dir(EN_VIN_CHECK, GPIO_OUT);    
    gpio_init(BUILD_IN_LED);
    gpio_set_dir(BUILD_IN_LED, GPIO_OUT);  
}
bool MeasureBattery()
{
    //turn on the transistor    
    gpio_put(EN_VIN_CHECK,1);    
    sleep_ms(100);
    //measure voltage   
    uint16_t result = adc_read();
    gpio_put(EN_VIN_CHECK,0);    
    if(result < MIN_ADC_VALUE)
    {
        for(int i = 0; i < 3; i++)
        {
            gpio_put(LED_VIN_LOW,1);
            gpio_put(BUILD_IN_LED,1);
            sleep_ms(500);
            gpio_put(LED_VIN_LOW,0);
            gpio_put(BUILD_IN_LED,0);
            sleep_ms(500);
        }
        return false;
    }    
    return true;
}

class Servo{
    protected:
    uint8_t pin;//given by the user in the constructor
    uint slice_num;//defined in ServoInit() method form the pin variable
    bool left = false; //if the servo is on the other side it has to move the opposite way -> left = true    
    float calibrationValue = 0;
    //msPosition:
    //calculated from position in the Write() method of destination 
    //--------OR----- 
    //Changes value in the ChangePosition() method and then moves servo from currentPosition 
    //to the changed one by using GoToPosition() method
    volatile uint16_t msPosition;  
    float velocity;// is changed and calculated in CalculateVelocity() method
    
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
        pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), CalculateLeft(currentPosition) + calibrationValue);
        else
        // Set channel output high for one cycle before dropping
        pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), currentPosition + calibrationValue);
    }   
    
    public:
    bool done;   
    float position;//position given by the user
    int currentPosition;
    int maxVelocity;
    int minVelocity;
    
    Servo(uint8_t chosen_pin=0, bool leftServo = false, int16_t calibration = 0)
    {
        velocity = MIN_VELOCITY;
        maxVelocity = MAX_VELOCITY;
        minVelocity = MIN_VELOCITY;
        currentPosition = (SERVO_MIN_MS + SERVO_MAX_MS)/2;
        calibrationValue = calibration;
        done = true;
        if(chosen_pin<16 && chosen_pin>=2)
        {            
            pin = chosen_pin;
            ServoInit();
            left = leftServo;
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
            WriteMs();
            
            //If the servo has reached the final position move is done            
            if(this->currentPosition==this->msPosition)
                done = true;
        }        
    }
    
    void ChangeVelocityLimits(int v)
    {
        minVelocity = v;
        maxVelocity = 3*v+5;
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

        if(velocity<minVelocity)
            velocity = minVelocity;
        if(velocity>maxVelocity)
            velocity = maxVelocity;
    }
    void ChangePosition(uint8_t pos)
    {
        msPosition = map(pos,0,180,SERVO_MIN_MS,SERVO_MAX_MS);
        position = pos;
        done = false;
    }
    void Write(uint8_t newPosition)
    {
        if(newPosition<=180 && newPosition>=0)
        {            
            this->position = newPosition;
            this->msPosition = map(this->position,0,180,SERVO_MIN_MS,SERVO_MAX_MS);
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

class SlaveServo: public Servo{
    private:
    bool slaveBack = true;    
    public :
    bool enableSlave = false;
    SlaveServo(uint8_t chosen_pin=0, bool leftServo = false, int16_t calibration = 0, bool sBack = false)
    :Servo(chosen_pin, leftServo, calibration)
    {
        slaveBack = sBack;
    }
    uint8_t Calculate(int masterPosition)
    {
        float alfa =  masterPosition - 90.0;     
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
    //sets SlaveServo at the right position, so the leg heigth does not change
    void SlavePosition(float masterPosition)
    {
        if(enableSlave)
        {
            position = Calculate(masterPosition);
            Write(position);         
        }        
    }
    
};

class Leg
{
    public:
    Servo master;
    SlaveServo slave;    
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
    public: 
    Leg(int pinMaster = 2, int pinSlave = 3, bool leftLeg = false, bool sBack = true, int16_t calibrationMaster = 0,int16_t calibrationSlave = 0)
    {
        master = Servo(pinMaster, leftLeg,calibrationMaster);
        slave = SlaveServo(pinSlave, leftLeg, calibrationSlave,sBack);
        if(!sBack)
        {
            maxPos = 180 - MASTER_SERVO_MIN_POS;
            minPos = 180 - MASTER_SERVO_MAX_POS;
            upPos = 180 - SLAVE_UP_POSITION;
        }
    }
    void ChangeLegVelocityLimits(int v)
    {
        master.ChangeVelocityLimits(v);
        slave.ChangeVelocityLimits(v);
    }
    //Writes master position and if the slave is enabled slave adjusts its angle, 
    //so the height of the leg does not change
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
    //Changes the desired position of the master
    void ChangePosition(uint8_t pos, bool slaveEnabled)
    {
        master.ChangePosition(pos);
        slave.enableSlave = slaveEnabled;
    }    

    void GoToPositionMaster()
    {
        master.GoToPosition();
        slave.SlavePosition(map(master.currentPosition,SERVO_MIN_MS,SERVO_MAX_MS,0.0,180.0));
    }
    //Changes only Slave desired position
    void ChangePositionSlave(uint8_t pos)
    {
        slave.ChangePosition(pos);
    }
    //Both servos are getting closer to the position set in ChangePosition()
    void GoToPosition()
    {
        if(!master.done)
            master.GoToPosition();
        if(!slave.done)
            slave.GoToPosition();
    }
    
    //automatically change position depending on the State, enableSlave and Leg properties (minPos, maxPos, upPos)
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
            GoToPositionMaster();
            return false;
        }
        else if (!slave.done)
        {
            GoToPosition();
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
    void DisableLeg()
    {
        master.Disable();
        slave.Disable();
    }
};

class Body
{
    private:
    uint8_t legTeam1[3] = {0,3,4};
    uint8_t legTeam2[3] = {1,2,5}; 
    State movingStates[ARRAY_SIZE(forwardStates)];
    Leg legs[6]; 
    uint8_t slowModeLegMove = 0;//leg:0-5 is movedover the ground
    public:    
    Mode modeType = StopMode;
    int step = 0;
    bool reset = false;
    Body(uint8_t masterPins[6], uint8_t slavePins[6], const int16_t calibration[12])
    {
        for(int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            bool left = (i%2 == 1);//left legs are leg 1,3,5
            bool oppositeSlave = !(i<4);//for first 4 legs the slave servo is positioned the opposite way
            legs[i] = Leg(masterPins[i],slavePins[i],left,oppositeSlave, calibration[i*2],calibration[i*2+1]);            
        }    
     
        for(int i = 0; i< ARRAY_SIZE(legs);i++)
        {
            legs[i].initLeg();
            sleep_ms(POS_90_TIME);
        } 
    }
    void ChangeBodyVelocityLimits(int v)
    {
        for(int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            legs[i].ChangeLegVelocityLimits(v);
        }
    }
    void ChangeToForward()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = forwardStates [i];
        }
        modeType = ForwardMode;
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
        modeType = BackMode;
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
        modeType = LeftMode;
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
        modeType = RightMode;
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
        modeType = StopMode;
    }
    void ChangeToReset()
    {
        for (int i = 0; i < ARRAY_SIZE(resetStates); i++)
        {
            movingStates[i] = resetStates [i];
        }
        modeType = ResetMode;
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
        for (int i = 0; i < ARRAY_SIZE(pos90States); i++)
        {
            movingStates[i] = pos90States[i];
        }
        step = 1;
        modeType = Pos90Mode;
        for(int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            legs[i].ChangePosition(90,false);
            legs[i].ChangePositionSlave(90);
        }
    }
    void Move()
    {        
        switch(movingStates[step])
        {
            case Stop:
            {
                //if ChangeToResetTemp(), the robot is already reset, so we can move on
                if(modeType != ResetMode && reset == true)
                {
                    ModeChanged(modeType);
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
                //1st leg is moving forward
                bool back = (modeType == BackMode);//Forward => false
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
                    legs[i].GoToPositionMaster();
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
                    legs[i].GoToPosition();
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
                //1st leg is moving back
                bool forward = (modeType == ForwardMode);//Forward => true
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
                
                bool right = (modeType == RightMode);
                legs[0].ChooseMove(Forward, right);
                legs[1].ChooseMove(Forward, !right);
                legs[2].ChooseMove(Back, !right);
                legs[3].ChooseMove(Back, right);
                legs[4].ChooseMove(Forward, right);
                legs[5].ChooseMove(Forward, !right);
                step++;
                break;
            }
            case  Right:
            {
                bool right = (modeType == RightMode);
                legs[0].ChooseMove(Back, !right);
                legs[1].ChooseMove(Back, right);
                legs[2].ChooseMove(Forward, right);
                legs[3].ChooseMove(Forward, !right);
                legs[4].ChooseMove(Back, !right);
                legs[5].ChooseMove(Back, right);
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
    void ModeChanged(Mode s)
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
    void DisableLegs()
    {
        for(int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            legs[i].DisableLeg();
        }
    }
};

int main() 
{
    stdio_init_all();

    gpio_init(BUILD_IN_LED);
    gpio_set_dir(BUILD_IN_LED, GPIO_OUT);

    UART_INIT();
    
    ADC_INIT();
    MeasureBattery();

    uint8_t mser[6],sser[6];
    for(int i = 0; i < ARRAY_SIZE(mser); i++)
    {
        mser[i] = 2*(i + 1);//(i+1) because we start at gpio 2
        sser[i] = 2*(i + 1) + 1;
    }

    Body body(mser,sser,SERVO_CALIB);
    
    if(MeasureBattery())
        body.ModeChanged(mode); 
    
    while(1)
    {
        if(!MeasureBattery())
        {
            body.DisableLegs();
            sleep_ms(10000);
        }
        else
        {
            if(velocityChanged)
            {
                body.ChangeBodyVelocityLimits(velocity);
            }
            // enableProgram = MeasureBattery();
            if(mode != body.modeType)
            {
                body.ModeChanged(mode);
            }
            gpio_put(BUILD_IN_LED,1);
            do
            {
                body.Move();
                sleep_ms(20); //20
            }
            while(!body.MovesDone());

            gpio_put(BUILD_IN_LED,0);
            
        }
        sleep_ms(20);
    }
}