/* 
Copyright 2014 Victor Joukov, Brad Quick

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

#include "Mini51Series.h"
#include "lib_i2c.h"

// referenced from app/serial.c
unsigned int lib_i2c_error_count = 0;

static inline int I2C_WAIT_READY_ERROR(I2C_T *i2c)
{
    int attempts = 1000;
    while(!(i2c->I2CON & I2C_I2CON_SI_Msk) && attempts--);
    if (!attempts) lib_i2c_error_count++;
    return attempts;
}


static uint8_t initialized = 0;
void lib_i2c_init(void)
{
    if (initialized) return;
    CLK_EnableModuleClock(I2C_MODULE);

    // Set P3.4 and P3.5 for I2C SDA and SCL
    SYS->P3_MFP = SYS_MFP_P34_SDA | SYS_MFP_P35_SCL;

    // SYS_ResetModule(I2C_RST);
    SYS->IPRSTC2 |=  SYS_IPRSTC2_I2C_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C_RST_Msk;

    /* Enable I2C Controller */
    I2C->I2CON |= I2C_I2CON_ENSI_Msk;

    // I2C clock divider, I2C Bus Clock = PCLK/4/(divider + 1)
    // PCLK 22.1184MHz, PCLK/4 = 5.5696MHz
    //I2C->I2CLK = 14-1; // ~395KHz
    //I2C->I2CLK = 20-1; // ~278KHz
    //I2C->I2CLK = 30-1; // ~184KHz
    I2C->I2CLK = 37-1; // ~151KHz
    //I2C->I2CLK = 55-1; // ~100KHz

    /* Enable I2C interrupt */
//    I2C->I2CON |= I2C_I2CON_EI_Msk;
//    NVIC_EnableIRQ(I2C_IRQn);
    initialized = 1;
}

void lib_i2c_setclockspeed(unsigned char speed){}


/*************************************************************************
 Send one byte to I2C device
 
 Input:    byte to be transfered
 Return:   0 write successful 
           1 write failed
*************************************************************************/
unsigned char lib_i2c_write(unsigned char data)
{
    I2C_SET_DATA(I2C, data);
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    if (!I2C_WAIT_READY_ERROR(I2C))
        return 2;
    // Check ACK
    uint8_t status = I2C_GET_STATUS(I2C);
    // Master TX/RX Address/Data ACKs
    if (status != 0x18 && status != 0x28 && status != 0x40 && status != 0x50)
        return 1;

    return 0;
}

/*************************************************************************   
 Issues a start condition and sends address and transfer direction.
 return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char lib_i2c_start(unsigned char address)
{
    // Send start
    I2C_START(I2C);
    if (!I2C_WAIT_READY_ERROR(I2C))
        return 3;

    // transmit address byte + direction
    if (lib_i2c_write(address)) // Master Transmit Address ACK
        return 2;

    return 0;
} /* lib_i2c_start */

/*************************************************************************
Issues a repeated start condition and sends address and transfer direction 

Input:   address and transfer direction of I2C device

Return:  0 device accessible
         1 failed to access device
*************************************************************************/
unsigned char lib_i2c_rep_start(unsigned char address)
{
    /* Send data */
    I2C_SET_CONTROL_REG(I2C, I2C_STA | I2C_SI);
    if (!I2C_WAIT_READY_ERROR(I2C))
        return 4;
    if(I2C_GET_STATUS(I2C) != 0x10) // Master repeat start
        return 3;
    // transmit address byte + direction
    if (lib_i2c_write(address))
        return 2;
    return 0;
}

/*************************************************************************
Terminates the data transfer and releases the I2C bus
*************************************************************************/
void lib_i2c_stop(void)
{
    // ? use I2C_STOP
    I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
}


void lib_i2c_writereg(unsigned char address, unsigned char reg, unsigned char value)
{
    lib_i2c_start((address << 1) + I2C_WRITE);
    lib_i2c_write(reg); // Master Transmit Data ACK
    lib_i2c_write(value);
    lib_i2c_stop();
}

/*************************************************************************
Read one byte from the I2C device, request more data from device 

Return:  byte read from I2C device
*************************************************************************/
unsigned char lib_i2c_readack(void)
{
    I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
    I2C_WAIT_READY_ERROR(I2C);
    return I2C_GET_DATA(I2C);
}


/*************************************************************************
Read one byte from the I2C device, read is followed by a stop condition 

Return:  byte read from I2C device
*************************************************************************/
unsigned char lib_i2c_readnak(void)
{
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY_ERROR(I2C);
    return I2C_GET_DATA(I2C);
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
