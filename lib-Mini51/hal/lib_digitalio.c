/* 
Copyright 2013 Brad Quick

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "hal.h"
#include "lib_digitalio.h"
#include "drv_gpio.h"

// This code controls the simple digital IO

// Usage:

// #define LEDOUTPUT (DIGITALPORTB | 7)     // defines the reference for pin 7 on PORTB
// lib_digitalio_initpin(LEDOUTPUT,DIGITALOUTPUT | PULLUPRESISTOR); // set the pin as an output (also turns on the pull up resistor)
// lib_digitalio_setoutput(LEDOUTPUT, DIGITALLOW); // turn the output on by pulling it low

// #define PUSHBUTTON (DIGITALPORTB | 4)     // defines the reference for pin 4 on PORTB
// #define INTERRUPT6PORTANDPIN PUSHBUTTON
// lib_digitalio_initpin(PUSHBUTTON,DIGITALINPUT);  // set the pin as an input (also turns on the pull up resistor)
// if (lib_digitalio_getinput(PUSHBUTTON)) {}       // Check the input
// unimplemented below:
// lib_digitalio_setinterruptcallback(PUSHBUTTON,mypushbuttoncallback); // tell the interrupt
// void mypushbuttoncallback(char interruptnumber,char newstate) // call back will get called any time the pin changes
//      {
//      if (newstate==DIGITALON) {}
//      }

void lib_digitalio_initpin(unsigned char portandpinnumber, unsigned char output)
{
    uint8_t port = (portandpinnumber & 0xF0) >> 4;
    uint8_t pin = portandpinnumber & 0x0F;
    // Call drv_gpio here
    uint32_t mode = output == DIGITALINPUT ? GPIO_PMD_INPUT : GPIO_PMD_OUTPUT;
    GPIO_SetMode(((GPIO_T *) (P0_BASE + 0x40*port)), (1 << pin), mode);
}

unsigned char lib_digitalio_getinput(unsigned char portandpinnumber)
{
    uint8_t port = (portandpinnumber & 0xF0) >> 4;
    uint8_t pin = portandpinnumber & 0x0F;  
    return GPIO_PIN_ADDR(port, pin);
}

void lib_digitalio_setoutput(unsigned char portandpinnumber, unsigned char value)
{
    uint8_t port = (portandpinnumber & 0xF0) >> 4;
    uint8_t pin = portandpinnumber & 0x0F;
    GPIO_PIN_ADDR(port, pin) = value ? 1 : 0;
}

void lib_digitalio_setinterruptcallback(unsigned char pinnumber, digitalcallbackfunctptr callback)
{
    // Not implemented, no need on real hardware...
}
