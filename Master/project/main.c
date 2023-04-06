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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include <math.h>

#include "utils/uartstdio.h"
#include "driverlib/uart.h"

#include "include.h"

// A boolean that is set when a MPU6050 command has completed.
volatile bool g_bMPU6050Done;

// I2C master instance
tI2CMInstance g_sI2CMSimpleInst;

//Device frequency
int clockFreq;

char newval[3];
char sent[20];
int pointer = 0;
char stop;
void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);


    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.
    // I2C data transfer rate set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());

}

void delayMS(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;  // less accurate
}

// The function that is provided by this example as a callback when MPU6050
// transactions have completed.
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status){
    // See if an error occurred.
    if (ui8Status != I2CM_STATUS_SUCCESS){
        // An error occurred, so handle it here if required.
    }
    // Indicate that the MPU6050 transaction has completed.
    g_bMPU6050Done = true;
}


// The interrupt handler for the I2C module.
void I2CMSimpleIntHandler(void){
    // Call the I2C master driver interrupt handler.
    I2CMIntHandler(&g_sI2CMSimpleInst);
}


// read data from MPU6050.
static const float dt = 1 / 200.0;
static const int ZERO_OFFSET_COUN = (int)(200);

static const float dt_2 = 1 / 150.0;
static const int ZERO_OFFSET_COUN_2 = (int)(150);

static int g_GetZeroOffset = 0;
static float gyroX_offset = 0.0f, gyroY_offset = 0.0f, gyroZ_offset = 0.0f;


void MPU6050Example(int *pitch, int *roll, int *yaw)
{
    double fAccel[3], fGyro[3];
    double tmp;
    float gyroX, gyroY, gyroZ;

    MPU6050_Read(&fAccel[0], &fAccel[1],&fAccel[2], &fGyro[0],&fGyro[1],&fGyro[2],&tmp);

    gyroX = fGyro[0];
    gyroY = fGyro[1];
    gyroZ = fGyro[2];



    if (g_GetZeroOffset++ < ZERO_OFFSET_COUN)
    {
        gyroX_offset += gyroX * dt;
        gyroY_offset += gyroY * dt;
        gyroZ_offset += gyroZ * dt;
    }

    // remove zero shift
    gyroX -= gyroX_offset;
    gyroY -= gyroY_offset;
    gyroZ -= gyroZ_offset;

    static float integralX = 0.0f, integralY = 0.0f, integralZ = 0.0f;
    if (g_GetZeroOffset > ZERO_OFFSET_COUN_2)
    {
        integralX += gyroX * dt_2;
        integralY += gyroY * dt_2;
        integralZ += gyroZ * dt_2;
        if (integralX > 360)
            integralX -= 360;
        if (integralX < -360)
            integralX += 360;
        if (integralY > 360)
            integralY -= 360;
        if (integralY < -360)
            integralY += 360;
        if (integralZ > 360)
            integralZ -= 360;
        if (integralZ < -360)
            integralZ += 360;
    }

    *pitch = (int)integralX;
    *roll = (int)integralY;
    *yaw = (int)integralZ;
    delayMS(5);
}

void conversion(int val){
    int con = val;
    if (val < 0){
        con = val * -1;
    }
    newval[0] = con / 100 + 48;
    newval[1] = con % 100 / 10 + 48;
    newval[2] = con % 100 % 10 + 48;


}

void putval(int val){
    if (val < 0){
        UARTCharPut(UART0_BASE, '-');
    }
    UARTCharPut(UART0_BASE, newval[0]);
    UARTCharPut(UART0_BASE, newval[1]);
    UARTCharPut(UART0_BASE, newval[2]);
    UARTCharPut(UART0_BASE, ' ');
}

void sentval(int val){
    if (val < 0){
        UARTCharPut(UART5_BASE, '-');
    }
    UARTCharPut(UART5_BASE, newval[0]);
    UARTCharPut(UART5_BASE, newval[1]);
    UARTCharPut(UART5_BASE, newval[2]);
}

int main(void) {

    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // enable UART0 and GPIOA.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

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
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_3, 2) ;

    // set interrupt for receiving and showing values
    IntMasterEnable();
    IntEnable(INT_UART5);
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);

    InitI2C0();

    MPU6050_Config(0x68, 1, 1);
    MPU6050_Calib_Set(903, 156, 1362, -4, 56, -16);

    int X = 0, Y = 0, Z = 0;

    while (1)
    {
        pointer = 0;
        // get raw data from MPU6050
        MPU6050Example(&X, &Y, &Z);
        // scale to a proper Master rotation

        UARTCharPut(UART0_BASE, 'R');
        UARTCharPut(UART0_BASE, ' ');

        X *= 10;
        Y *= 10;
        Z *= 10;

        if (X > 180) X = 180;
        else if (X < -180) X = -180;

        // Constraint (Pitch)
        if (Y > 110) Y = 110;
        else if (Y < 20) Y = 20;

        // Constraint (Yaw)
        if (Z > 160) Z = 160;
        else if (Z < 20) Z = 20;
        if(Y==110 && Z>100)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x02);
            SysCtlDelay(200000);
        }
        else if(Z>100)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x08);
            SysCtlDelay(200000);
        }
        else if(Y==110)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x04);
            SysCtlDelay(200000);
        }
        else {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
                        SysCtlDelay(200000);
        }


        conversion(X);
        putval(X);
        conversion(Y);
        sentval(Y);
        UARTCharPut(UART5_BASE, 'l');
        putval(Y);
        conversion(Z);
        sentval(Z);
        UARTCharPut(UART5_BASE, 'u');
        putval(Z);
        UARTCharPut(UART0_BASE, 'x');
        UARTCharPut(UART0_BASE, '\r');

        SysCtlDelay(SysCtlClockGet() / (10000000 * 3));

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


    {
        UARTCharPut(UART0_BASE, UARTCharGet(UART5_BASE)); //echo character
        SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay some time
    }
}
void UART0IntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status

    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts

    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    {
        UARTCharPut(UART5_BASE, UARTCharGet(UART0_BASE)); //echo character
        SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay some time
    }
}


