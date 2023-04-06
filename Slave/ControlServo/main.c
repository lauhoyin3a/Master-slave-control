// Bluetooth Serial Echo program

// Author: Xu Zhijian Oct,20,2022

// Simple Bluetooth serial reading from Host device (PC/Phone)

/*
Hardware connection:
RXD -> PE5
TXD -> PE4
GND -> GND
VCC -> VBUS
EN -> VCC (for AT mode)

Software Implementation:
UART0 used to communicate with computer (only send in this demo)
UART5 used to communicate with HC05 (only receive in this demo)

Procedures:
1. Smartphone/PC with bluetooth use bluetooth serial to send message to HC05,
2. HC05 is connected to the UART5 on Tiva board. HC05 sends the message via UART5.
3. Board receive character from HC05 in UART5 and forward to UART0 to computer.
4. (recommended tools are listed in slide appendix).
*/


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"

#define PWM_FREQUENCY 55

char receive[20];
int pointer = 0;

int x = 20;
int z = 20;

int main(void) {
    //ui8Adjust is used to contol the servo angle.999
    //We initialize ui8Adjust to 83 to make sure the servo is at the center position.
    volatile uint32_t ui32Load;
    volatile uint32_t ui32PWMClock;
    volatile uint32_t ui8Adjust;
    volatile uint32_t ui8Adjustu;
    ui8Adjust = 83;

    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    // enable UART0 and GPIOA.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure PA0 for RX
    // Configure PA1 for TX
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    // Set PORTA pin0 and pin1 as UART type
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // set UART base addr., clock get and baud rate.
    // used to communicate with computer
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 38400,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable PWM1 to generate PWM signals
    // Enable GPIOD to output signals to servo
    // Enable GPIOF to use buttons
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);

    // set GPIOs for buttons
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)  |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    ui32PWMClock = SysCtlClockGet()/64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

    // set the servo's initial position
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load/1000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8Adjust * ui32Load/1000);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);


    // enable UART5 and GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure board PE4 for RX
    // configure board PE5 for TX
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    // set PORTE pin4 and pin5 as UART type
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // set UART base addr., system clock, baud rate
    // used to communicate with HC-05
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 38400,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_3, 2) ;

    // set interrupt for receiving and showing values
    IntMasterEnable();
    IntEnable(INT_UART5);
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);

    UARTCharPut(UART0_BASE, 'W');
    UARTCharPut(UART0_BASE, 'a');
    UARTCharPut(UART0_BASE, 'i');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, 'i');
    UARTCharPut(UART0_BASE, 'n');
    UARTCharPut(UART0_BASE, 'g');
    UARTCharPut(UART0_BASE, '.');
    UARTCharPut(UART0_BASE, '.');
    UARTCharPut(UART0_BASE, '.');
    UARTCharPut(UART0_BASE, '\n');

    while (1)
    {

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, z * ui32Load/1000);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, x * ui32Load/1000);

        // since the main controlling function is implemented in while loop
        // we need to use delay function to control the rotating speed of the servo.

        SysCtlDelay(100000);

    }

}

//check whether there are any items in the FIFO of UART5.
//get characters from UART5 that communicates with bluetooth.
//send received characters to UART0 that communicates with PC.
void UART5IntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART5_BASE, true); //get interrupt status

    UARTIntClear(UART5_BASE, ui32Status); //clear the asserted interrupts

    while(UARTCharsAvail(UART5_BASE)) //loop while there are chars
    {
        //UARTCharPut(UART0_BASE, UARTCharGet(UART5_BASE)); //echo character
        receive[pointer] = UARTCharGetNonBlocking(UART5_BASE);
        UARTCharPutNonBlocking(UART0_BASE, receive[pointer]);

        if (receive[pointer] == 'u'){
                x = (int)(receive[0]-48)*100 + (int)(receive[1]-48)*10 + (int)(receive[2]-48);
                z = (int)(receive[4]-48)*100 + (int)(receive[5]-48)*10 + (int)(receive[6]-48);

                pointer = 0;
                }
        else{
            pointer++;
        }

        SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay some time
    }
}

