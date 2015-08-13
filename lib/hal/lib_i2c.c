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
#include "lib_i2c.h"

#define SCL_H         GPIOB->BSRR = Pin_10 /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         GPIOB->BRR  = Pin_10 /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         GPIOB->BSRR = Pin_11 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         GPIOB->BRR  = Pin_11 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      (GPIOB->IDR & Pin_10) /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      (GPIOB->IDR & Pin_11) /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */

static void I2C_delay(void)
{
    volatile int i = 7;
    while (i)
        i--;
}

static uint8_t lib_i2c_receivebyte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

unsigned int lib_i2c_error_count = 0;

unsigned char lib_i2c_waitack(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if (SDA_read) {
        SCL_L;
        return 1;
    }
    SCL_L;

    return 0;
}

/*************************************************************************
Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void lib_i2c_init(void)
{
    gpio_config_t gpio;

    gpio.pin = Pin_10 | Pin_11;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_Out_OD;
    gpioInit(GPIOB, &gpio);
}

void lib_i2c_setclockspeed(unsigned char speed)
{

}

/*************************************************************************   
 Issues a start condition and sends address and transfer direction.
 return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char lib_i2c_start(unsigned char address)
{
    // send START condition
    SDA_H;
    SCL_H;
    I2C_delay();
    if (!SDA_read)
        return 1;
    SDA_L;
    I2C_delay();
    if (SDA_read)
        return 1;
    SDA_L;
    I2C_delay();

    // transmit address byte + direction
    if (lib_i2c_write(address))
        return 2;

    return 0;
}                               /* lib_i2c_start */

/*************************************************************************
Issues a repeated start condition and sends address and transfer direction 

Input:   address and transfer direction of I2C device

Return:  0 device accessible
         1 failed to access device
*************************************************************************/
unsigned char lib_i2c_rep_start(unsigned char address)
{
    return lib_i2c_start(address);

}


/*************************************************************************
Terminates the data transfer and releases the I2C bus
*************************************************************************/
void lib_i2c_stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}


/*************************************************************************
 Send one byte to I2C device
 
 Input:    byte to be transfered
 Return:   0 write successful 
           1 write failed
*************************************************************************/
unsigned char lib_i2c_write(unsigned char data)
{
    int i = 8;
    // transmit byte
    while (i--) {
        SCL_L;
        I2C_delay();
        if (data & 0x80)
            SDA_H;
        else
            SDA_L;
        data <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;

    if (lib_i2c_waitack()) {
        lib_i2c_stop();
        return 1;
    }

    return 0;
}


/*************************************************************************
Read one byte from the I2C device, request more data from device 

Return:  byte read from I2C device
*************************************************************************/
unsigned char lib_i2c_readack(void)
{
    uint8_t ch;
    ch = lib_i2c_receivebyte();
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();

    return ch;
}


/*************************************************************************
Read one byte from the I2C device, read is followed by a stop condition 

Return:  byte read from I2C device
*************************************************************************/
unsigned char lib_i2c_readnak(void)
{
    uint8_t ch;
    ch = lib_i2c_receivebyte();

    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();

    return ch;
}

void lib_i2c_writereg(unsigned char address, unsigned char reg, unsigned char value)
{
    lib_i2c_start((address << 1) + I2C_WRITE);
    lib_i2c_write(reg);
    lib_i2c_write(value);
    lib_i2c_stop();
}

unsigned char lib_i2c_readreg(unsigned char address, unsigned char reg)
{
    lib_i2c_start((address << 1) + I2C_WRITE);
    lib_i2c_write(reg);
    lib_i2c_rep_start((address << 1) + I2C_READ);

    unsigned char returnvalue = lib_i2c_readnak();

    lib_i2c_stop();

    return (returnvalue);
}

void lib_i2c_readdata(unsigned char address, unsigned char reg, unsigned char *data, unsigned char length)
{
    lib_i2c_start((address << 1) + I2C_WRITE);
    lib_i2c_write(reg);
    lib_i2c_rep_start((address << 1) + I2C_READ);

    while (--length) {
        *data++ = lib_i2c_readack();
    }
    *data = lib_i2c_readnak();

    lib_i2c_stop();
}
