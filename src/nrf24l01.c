/*
    This file is copied with modifications from project Deviation,
    see http://deviationtx.com

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Deviation is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "hal.h"
#include "nrf24l01.h"
#include "lib_spi.h"
#include "lib_timers.h"


/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF


static uint8_t rf_setup;

static void usleep(unsigned long delayus)
{
    unsigned long timercounts = lib_timers_starttimer();
    while (lib_timers_gettimermicroseconds(timercounts) < delayus) {
    }
}

void NRF24L01_Initialize()
{
    rf_setup = 0x0F;
}    

uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t data)
{
    lib_spi_ss_on();
    uint8_t res = lib_spi_xfer(W_REGISTER | (REGISTER_MASK & reg));
    lib_spi_xfer(data);
    lib_spi_ss_off();
    return res;
}

uint8_t NRF24L01_WriteRegisterMulti(uint8_t reg, const uint8_t data[], uint8_t length)
{
    lib_spi_ss_on();
    uint8_t res = lib_spi_xfer(W_REGISTER | ( REGISTER_MASK & reg));
    for (uint8_t i = 0; i < length; i++)
    {
        lib_spi_xfer(data[i]);
    }
    lib_spi_ss_off();
    return res;
}

uint8_t NRF24L01_WritePayload(uint8_t *data, uint8_t length)
{
    lib_spi_ss_on();
    uint8_t res = lib_spi_xfer(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++)
    {
        lib_spi_xfer(data[i]);
    }
    lib_spi_ss_off();
    return res;
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    lib_spi_ss_on();
    lib_spi_xfer(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t data = lib_spi_xfer(0xFF);
    lib_spi_ss_off();
    return data;
}

uint8_t NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t data[], uint8_t length)
{
    lib_spi_ss_on();
    uint8_t res = lib_spi_xfer(R_REGISTER | (REGISTER_MASK & reg));
    for(uint8_t i = 0; i < length; i++)
    {
        data[i] = lib_spi_xfer(0xFF);
    }
    lib_spi_ss_off();
    return res;
}

uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length)
{
    lib_spi_ss_on();
    uint8_t res = lib_spi_xfer(R_RX_PAYLOAD);
    for(uint8_t i = 0; i < length; i++)
    {
        data[i] = lib_spi_xfer(0xFF);
    }
    lib_spi_ss_off();
    return res;
}

static uint8_t Strobe(uint8_t state)
{
    lib_spi_ss_on();
    uint8_t res = lib_spi_xfer(state);
    lib_spi_ss_off();
    return res;
}

uint8_t NRF24L01_FlushTx()
{
    return Strobe(FLUSH_TX);
}

uint8_t NRF24L01_FlushRx()
{
    return Strobe(FLUSH_RX);
}

uint8_t NRF24L01_Activate(uint8_t code)
{
    lib_spi_ss_on();
    uint8_t res = lib_spi_xfer(ACTIVATE);
    lib_spi_xfer(code);
    lib_spi_ss_off();
    return res;
}

uint8_t NRF24L01_SetBitrate(uint8_t bitrate)
{
    // Note that bitrate 250kbps (and bit RF_DR_LOW) is valid only
    // for nRF24L01+. There is no way to programmatically tell it from
    // older version, nRF24L01, but the older is practically phased out
    // by Nordic, so we assume that we deal with with modern version.

    // Bit 0 goes to RF_DR_HIGH, bit 1 - to RF_DR_LOW
    rf_setup = (rf_setup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}

// Power setting is 0..3 for nRF24L01
// Claimed power amp for nRF24L01 from eBay is 20dBm. 
//      Raw            w 20dBm PA
// 0 : -18dBm  (16uW)   2dBm (1.6mW)
// 1 : -12dBm  (60uW)   8dBm   (6mW)
// 2 :  -6dBm (250uW)  14dBm  (25mW)
// 3 :   0dBm   (1mW)  20dBm (100mW)
// So it maps to Deviation as follows
/*
TXPOWER_100uW  = -10dBm
TXPOWER_300uW  = -5dBm
TXPOWER_1mW    = 0dBm
TXPOWER_3mW    = 5dBm
TXPOWER_10mW   = 10dBm
TXPOWER_30mW   = 15dBm
TXPOWER_100mW  = 20dBm
TXPOWER_150mW  = 22dBm
*/
uint8_t NRF24L01_SetPower(uint8_t power)
{
    uint8_t nrf_power = 0;
    switch(power) {
        case TXPOWER_100uW: nrf_power = 0; break;
        case TXPOWER_300uW: nrf_power = 0; break;
        case TXPOWER_1mW:   nrf_power = 0; break;
        case TXPOWER_3mW:   nrf_power = 1; break;
        case TXPOWER_10mW:  nrf_power = 1; break;
        case TXPOWER_30mW:  nrf_power = 2; break;
        case TXPOWER_100mW: nrf_power = 3; break;
        case TXPOWER_150mW: nrf_power = 3; break;
        default:            nrf_power = 0; break;
    };
    // Power is in range 0..3 for nRF24L01
    rf_setup = (rf_setup & 0xF9) | ((nrf_power & 0x03) << 1);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}
static void CE_lo()
{
#if HAS_MULTIMOD_SUPPORT
    SPI_ConfigSwitch(0x0f, 0x0b);
#endif
}
static void CE_hi()
{
#if HAS_MULTIMOD_SUPPORT
    SPI_ConfigSwitch(0x1f, 0x1b);
#endif
}

void NRF24L01_SetTxRxMode(enum TXRX_State mode)
{
    if(mode == TX_EN) {
        CE_lo();
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP));
        usleep(130);
        CE_hi();
    } else if (mode == RX_EN) {
        CE_lo();
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);        // switch to RX mode
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP)
                                            | (1 << NRF24L01_00_PRIM_RX));
        usleep(130);
        CE_hi();
    } else {
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); //PowerDown
        CE_lo();
    }
}

int NRF24L01_Reset()
{
    NRF24L01_SetTxRxMode(TXRX_OFF);
    return NRF24L01_ReadReg(0x07) == 0x0E;
}
