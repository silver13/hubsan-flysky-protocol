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

#include "a7105.h"
#include "lib_soft_3_wire_spi.h"

void A7105_WriteID(uint32_t ida) 
{
    lib_soft_3_wire_spi_setCS(DIGITALOFF);
    lib_soft_3_wire_spi_write(A7105_06_ID_DATA);//ex id=0x5475c52a ;txid3txid2txid1txid0
    lib_soft_3_wire_spi_write((ida>>24)&0xff);//53 
    lib_soft_3_wire_spi_write((ida>>16)&0xff);//75
    lib_soft_3_wire_spi_write((ida>>8)&0xff);//c5
    lib_soft_3_wire_spi_write((ida>>0)&0xff);//2a
    lib_soft_3_wire_spi_setCS(DIGITALON);
}

// read 4 bytes ID
void A7105_ReadID(uint8_t *_aid)
{
    uint8_t i;
    lib_soft_3_wire_spi_setCS(DIGITALOFF);
    lib_soft_3_wire_spi_write(0x46);
    for(i=0;i<4;i++){
        _aid[i]=lib_soft_3_wire_spi_read();
    }
    lib_soft_3_wire_spi_setCS(DIGITALON);
}

void A7105_WritePayload(uint8_t *_packet, uint8_t len) 
{
    uint8_t i;
    lib_soft_3_wire_spi_setCS(DIGITALOFF);
    lib_soft_3_wire_spi_write(A7105_RST_WRPTR);
    lib_soft_3_wire_spi_write(0x05);
    for (i=0;i<len;i++) {
        lib_soft_3_wire_spi_write(_packet[i]);
    }
    lib_soft_3_wire_spi_setCS(DIGITALON);
}

void A7105_ReadPayload(uint8_t *_packet, uint8_t len) 
{
    uint8_t i;
    lib_soft_3_wire_spi_setCS(DIGITALOFF);
    lib_soft_3_wire_spi_write(0x45);
    for (i=0;i<len;i++) {
        _packet[i]=lib_soft_3_wire_spi_read();
    }
    lib_soft_3_wire_spi_setCS(DIGITALON);
}

void A7105_Reset(void) 
{
    A7105_WriteRegister(A7105_00_MODE,0x00); 
}

uint8_t A7105_ReadRegister(uint8_t address) 
{ 
    uint8_t result;
    lib_soft_3_wire_spi_setCS(DIGITALOFF);
    address |=0x40; 
    lib_soft_3_wire_spi_write(address);
    result = lib_soft_3_wire_spi_read();  
    lib_soft_3_wire_spi_setCS(DIGITALON);
    return(result); 
} 

void A7105_WriteRegister(uint8_t address, uint8_t data) 
{
    lib_soft_3_wire_spi_setCS(DIGITALOFF);
    lib_soft_3_wire_spi_write(address); 
    lib_soft_3_wire_spi_write(data);  
    lib_soft_3_wire_spi_setCS(DIGITALON);
} 

void A7105_Strobe(uint8_t command) 
{
    lib_soft_3_wire_spi_setCS(DIGITALOFF);
    lib_soft_3_wire_spi_write(command);
    lib_soft_3_wire_spi_setCS(DIGITALON);
}
