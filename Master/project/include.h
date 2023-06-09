/*
 * include.h
 *
 *  Created on: 2022��10��19��
 *      Author: xiangyu
 */

#ifndef INCLUDE_H_
#define INCLUDE_H_


// Libraries for supporting bool, int, string and math
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// API driver library

#include "driverlib/gpio.h"
#include "driverlib/can.h"
#include "driverlib/eeprom.h"
#include "driverlib/i2c.h"
#include "driverlib/lcd.h"
//#include "driverlib/lcd16x2.h"
#include "driverlib/mpu.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"

// Hardware supported libraries
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

// User libraries
#include "I2C/I2C.h"
#include "TIMER/TIMER.h"
#include "MPU6050.h"

#include "stdlib.h"         // atof() to read number

// User-defined constants
#define M_PI 3.14159265358979323846


#endif /* INCLUDE_H_ */
