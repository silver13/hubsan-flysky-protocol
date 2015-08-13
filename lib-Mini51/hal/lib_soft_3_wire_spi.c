/* 
Copyright 2014 Goebish

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
#include "lib_soft_3_wire_spi.h"

static uint8_t pin_SDIO, pin_SCK, pin_SCS;
void lib_soft_3_wire_spi_setSCK(uint8_t state);
void lib_soft_3_wire_spi_setSDIO(uint8_t state);

void lib_soft_3_wire_spi_setCS(uint8_t state)
{
    lib_digitalio_setoutput( pin_SCS, state);
}

void lib_soft_3_wire_spi_setSCK(uint8_t state)
{
    lib_digitalio_setoutput( pin_SCK, state);
}

void lib_soft_3_wire_spi_setSDIO(uint8_t state)
{
    lib_digitalio_setoutput( pin_SDIO, state);
}

void lib_soft_3_wire_spi_init(uint8_t SDIO_portandpinnumber, uint8_t SCK_portandpinnumber, uint8_t SCS_portandpinnumber )
{
    pin_SDIO = SDIO_portandpinnumber;
    pin_SCK = SCK_portandpinnumber;
    pin_SCS = SCS_portandpinnumber;
    lib_digitalio_initpin(pin_SDIO, DIGITALOUTPUT);
    lib_digitalio_initpin(pin_SCK, DIGITALOUTPUT);
    lib_digitalio_initpin(pin_SCS, DIGITALOUTPUT);
    lib_soft_3_wire_spi_setCS(DIGITALOFF);
    lib_soft_3_wire_spi_setSDIO(DIGITALOFF);
    lib_soft_3_wire_spi_setSCK(DIGITALOFF);
}

void lib_soft_3_wire_spi_write(uint8_t data) 
{  
    uint8_t n=8; 
    lib_soft_3_wire_spi_setSCK(DIGITALOFF);
    lib_soft_3_wire_spi_setSDIO(DIGITALOFF);
    while(n--) {
        if(data&0x80) // MSB first
            lib_soft_3_wire_spi_setSDIO(DIGITALON);
        else 
            lib_soft_3_wire_spi_setSDIO(DIGITALOFF);
		lib_soft_3_wire_spi_setSCK(DIGITALON);
		lib_soft_3_wire_spi_setSCK(DIGITALOFF);
        data = data << 1;
    }
    lib_soft_3_wire_spi_setSDIO(DIGITALON);
}

uint8_t lib_soft_3_wire_spi_read(void) 
{
    uint8_t result=0;
    uint8_t i;
    lib_digitalio_initpin( pin_SDIO, DIGITALINPUT); 
    for(i=0;i<8;i++) {                    
        if( lib_digitalio_getinput(pin_SDIO)) 
            result=(result<<1)|0x01;
        else
            result=result<<1;
        lib_soft_3_wire_spi_setSCK(DIGITALON);
        lib_soft_3_wire_spi_setSCK(DIGITALOFF);
    }
    lib_digitalio_initpin( pin_SDIO, DIGITALOUTPUT);
    return result;
}
