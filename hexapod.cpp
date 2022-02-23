#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/double.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "lib/Servo.hpp"
#include "lib/Leg.hpp"
#include "lib/Body.hpp"

//----------------------my program-------------------------------------------------
#define UART_ID uart0
#define BAUD_RATE 9600

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])

// pins
#define RX_PIN 1
#define TX_PIN 0
// Servos - 2-13
#define LED_VIN_LOW 26
#define EN_VIN_CHECK 27
#define VIN_CHECK_GPIO 28
#define VIN_CHECK_A_INPUT (VIN_CHECK_GPIO - 26)
#define BUILD_IN_LED 25

#define LED_PIN 15

// Measure Battery
#define RESISTOR_3 4700          // OHM
#define RESISTOR_2 10000         // OHM
#define RP_VOLTAGE 3.3           // V
#define MIN_BATTERY_VOLATAGE 7.0 // V
#define ADC_MAX 4096             // 12-bit
#define RESISTOR_3_MIN_VOLTAGE (MIN_BATTERY_VOLATAGE / (RESISTOR_2 + RESISTOR_3) * RESISTOR_3)
#define MIN_ADC_VALUE (RESISTOR_3_MIN_VOLTAGE * ADC_MAX / RP_VOLTAGE)

const int16_t SERVO_CALIB[12] = {
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
    -50};

void OnUartRx();
void UART_INIT()
{
    uart_init(uart0, BAUD_RATE);

    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);

    // interrupts
    //  Set up a RX interrupt
    //  We need to set up the handler first
    //  Select correct interrupt for the UART we are using
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
void OnUartRx() // bool *changeVelocity, *velocityChanged, *velocity, *mode
{
    //----------getting the robot moves-----------------

    while (uart_is_readable(UART_ID))
    {
        char recMode = uart_getc(UART_ID);
        uart_puts(uart0, "Recieved:\n");
        uart_putc(uart0, recMode);
        uart_puts(uart0, "\n");
        if (changeVelocity)
        {
            if (recMode == '&')
            {
                velocityChanged = true;
                changeVelocity = false;
            }
            else
            {
                velocity *= 10;
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
    // init ADC
    adc_init();
    // // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(VIN_CHECK_GPIO);
    // Select ADC input 0 (GPIO26)
    adc_select_input(VIN_CHECK_A_INPUT);
    // set LED and gpio, which turns on the transistor as output
    gpio_init(LED_VIN_LOW);
    gpio_set_dir(LED_VIN_LOW, GPIO_OUT);
    // gpio_put(LED_VIN_LOW,1);
    gpio_init(EN_VIN_CHECK);
    gpio_set_dir(EN_VIN_CHECK, GPIO_OUT);
    gpio_init(BUILD_IN_LED);
    gpio_set_dir(BUILD_IN_LED, GPIO_OUT);
}
bool MeasureBattery()
{
    // turn on the transistor
    gpio_put(EN_VIN_CHECK, 1);
    sleep_ms(100);
    // measure voltage
    uint16_t result = adc_read();
    gpio_put(EN_VIN_CHECK, 0);
    if (result < MIN_ADC_VALUE)
    {
        for (int i = 0; i < 3; i++)
        {
            gpio_put(LED_VIN_LOW, 1);
            gpio_put(BUILD_IN_LED, 1);
            sleep_ms(500);
            gpio_put(LED_VIN_LOW, 0);
            gpio_put(BUILD_IN_LED, 0);
            sleep_ms(500);
        }
        return false;
    }
    return true;
}

int main()
{
    stdio_init_all();

    gpio_init(BUILD_IN_LED);
    gpio_set_dir(BUILD_IN_LED, GPIO_OUT);

    UART_INIT();
    ADC_INIT();
    MeasureBattery();

    uint8_t mser[6], sser[6];
    for (int i = 0; i < ARRAY_SIZE(mser); i++)
    {
        mser[i] = 2 * (i + 1); //(i+1) because we start at gpio 2
        sser[i] = 2 * (i + 1) + 1;
    }

    Body body(mser, sser, SERVO_CALIB);

    if (MeasureBattery())
        body.ModeChanged(mode);

    while (1)
    {
        if (!MeasureBattery())
        {
            body.DisableLegs();
            sleep_ms(10000);
        }
        else
        {
            if (velocityChanged)
            {
                body.ChangeBodyVelocityLimits(velocity);
            }
            // enableProgram = MeasureBattery();
            if (mode != body.modeType)
            {
                body.ModeChanged(mode);
            }
            gpio_put(BUILD_IN_LED, 1);
            do
            {
                body.Move();
                sleep_ms(20); // 20
            } while (!body.MovesDone());

            gpio_put(BUILD_IN_LED, 0);
        }
        sleep_ms(20);
    }
}