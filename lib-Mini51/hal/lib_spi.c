/* 
Copyright 2014 Victor Joukov

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

#pragma once

#include "hal.h"
#include "lib_spi.h"

void lib_spi_init(void)
{
    CLK_EnableModuleClock(SPI_MODULE);

    CLK->CLKSEL1 |= CLK_CLKSEL1_SPI_S_HCLK;

    // MFP pin configuration
    SYS->P0_MFP |= SYS_MFP_P01_SPISS | SYS_MFP_P05_MOSI | SYS_MFP_P06_MISO | SYS_MFP_P07_SPICLK;

    //SYS_ResetModule(SPI_RST);
    SYS->IPRSTC2 |=  SYS_IPRSTC2_SPI_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_SPI_RST_Msk;
    
    // Configure as a master, clock idle low,
    // falling clock edge Tx, rising edge Rx and 32-bit transaction
    CLK->APBCLK |= CLK_APBCLK_SPI_EN_Msk;
    SPI->CNTRL = SPI_MASTER | SPI_MODE_0;

    // SPI clock rate = 1843200Hz (PCLK/12)
    //SPI->DIVIDER = 5;
    SPI->DIVIDER = 10; // PCLK/22 ~1MHz

    // 8-bit per transfer
    SPI_SET_DATA_WIDTH(SPI, 8);

    // Select the SS pin and configure as low-active.
    SPI->SSR |= SPI_SS_ACTIVE_LOW;
}

void lib_spi_ss_on(void)
{
    SPI->SSR |= SPI_SS;
}

void lib_spi_ss_off(void)
{
    SPI->SSR &= ~SPI_SS;
}

uint8_t lib_spi_xfer(uint8_t data)
{
    SPI->TX = data;

    /* SPI Go */
    SPI->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    /* Wait SPI is free */
    while(SPI->CNTRL & SPI_CNTRL_GO_BUSY_Msk);

    /* Read Data */
    return SPI->RX;
}    
