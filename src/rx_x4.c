/* 
Copyright 2014 Goebish

Some of this code is based on Hubsan RX by Midelic on rcgroups

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
#include "config_X4.h"

#ifndef FLYSKY_RX

#include "bradwii.h"
#include "rx.h"
#include "lib_soft_3_wire_spi.h"
#include "lib_timers.h"
#include "a7105.h"


#define A7105_SCS   (DIGITALPORT1 | 4)
#define A7105_SCK   (DIGITALPORT1 | 3)
#define A7105_SDIO  (DIGITALPORT1 | 2)

#define AUX1_FLAG   0x04 
#define AUX2_FLAG   0x08 

static const uint8_t allowed_ch[] = {0x14, 0x1E, 0x28, 0x32, 0x3C, 0x46, 0x50, 0x5A, 0x64, 0x6E, 0x78, 0x82};
static uint8_t packet[16], channel, counter;
static uint8_t txid[4];
static unsigned long timeout_timer;
void init_a7105(void);
bool hubsan_check_integrity(void);
void update_crc(void);

extern globalstruct global;

void update_crc(void)
{
    int sum = 0;
    for(int i = 0; i < 15; i++)
        sum += packet[i];
    packet[15] = (256 - (sum % 256)) & 0xff;
}

bool hubsan_check_integrity(void) 
{
    int sum = 0;
    for(int i = 0; i < 15; i++)
        sum += packet[i];
    return packet[15] == ((256 - (sum % 256)) & 0xff);
}

void hubsan_build_bind_packet(uint8_t bindstate)
{
    packet[0] = bindstate;
    packet[1] = (bindstate!=0x0a)? channel : counter;
    packet[6] = 0x08;
    packet[7] = 0xe4;
    packet[8] = 0xea;	
    packet[9] = 0x9e;
    packet[10] = 0x50;
    update_crc();
}

void init_a7105(void)
{
    A7105_Reset();
    A7105_WriteID(0x55201041); 
    A7105_WriteRegister(A7105_01_MODE_CONTROL, 0x63);
    A7105_WriteRegister(A7105_03_FIFOI, 0x0f);
    A7105_WriteRegister(A7105_0D_CLOCK, 0x05);
    A7105_WriteRegister(A7105_0E_DATA_RATE, 0x04);
    A7105_WriteRegister(A7105_15_TX_II, 0x2b);
    A7105_WriteRegister(A7105_18_RX, 0x62);
    A7105_WriteRegister(A7105_19_RX_GAIN_I, 0x80);
    A7105_WriteRegister(A7105_1C_RX_GAIN_IV, 0x0A);
    A7105_WriteRegister(A7105_1F_CODE_I, 0x07);
    A7105_WriteRegister(A7105_20_CODE_II, 0x17);
    A7105_WriteRegister(A7105_29_RX_DEM_TEST_I, 0x47);
    A7105_Strobe(A7105_STANDBY);
    A7105_WriteRegister(A7105_02_CALC,0x01);
    A7105_WriteRegister(A7105_0F_PLL_I,0x00);
    A7105_WriteRegister(A7105_02_CALC,0x02);
    A7105_WriteRegister(A7105_0F_PLL_I,0xA0);
    A7105_WriteRegister(A7105_02_CALC,0x02);
    A7105_Strobe(A7105_STANDBY);
}

void waitTRXCompletion(void)
{
    while(( A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK)) 
        ;
}

void strobeTXRX(void)
{
    A7105_WriteRegister(A7105_0F_PLL_I, channel);
    A7105_Strobe(A7105_TX);
    waitTRXCompletion();
    A7105_Strobe(A7105_RX);
    waitTRXCompletion();
    A7105_Strobe(A7105_RST_RDPTR);
}

void bind() 
{
    uint8_t chan=0;
	
    while(1){
        if( lib_timers_gettimermicroseconds(0) % 500000 > 250000)
            x4_set_leds(X4_LED_FR | X4_LED_RL);
        else
            x4_set_leds(X4_LED_FL | X4_LED_RR);

        A7105_Strobe(A7105_STANDBY);
        channel=allowed_ch[chan];
        if(chan==11)
            chan=0;
        A7105_WriteRegister(A7105_0F_PLL_I, channel);
        A7105_Strobe(A7105_RX);
        unsigned long timer=lib_timers_starttimer();
        while(1){
            if(lib_timers_gettimermicroseconds(timer) > 8000) {
                chan++;
                break;
            }
            if(A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK){
                continue;
            }else{
                A7105_ReadPayload((uint8_t*)&packet, sizeof(packet)); 
                A7105_Strobe(A7105_RST_RDPTR);
                if (packet[0]==1){
                    break;
                }	
            }
        }	
        if (packet[0]==1){
            break;
        }
    }
    channel = packet[1];
	
    while(1) {
        hubsan_build_bind_packet(2);
        A7105_Strobe(A7105_STANDBY);
        A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
        strobeTXRX();
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (packet[0]==3){
            break;
        }
    }
	
    hubsan_build_bind_packet(4);
    A7105_Strobe(A7105_STANDBY);
    A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
    A7105_WriteRegister(A7105_0F_PLL_I, channel);
    A7105_Strobe(A7105_TX);
    waitTRXCompletion();

    A7105_WriteID(((uint32_t)packet[2] << 24) | ((uint32_t)packet[3] << 16) | ((uint32_t)packet[4] << 8) | packet[5]);
	
    while(1) { // useless block ?
        A7105_Strobe(A7105_RX);
        waitTRXCompletion();
        A7105_Strobe(A7105_RST_RDPTR);
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (packet[0]==1){
            break;
        }
    }
	
    while(1){
        hubsan_build_bind_packet(2);
        A7105_Strobe(A7105_STANDBY);
        A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
        strobeTXRX();
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (packet[0]==9){
            break;
        }
    }
	
    while(1){
        counter++;
        if(counter==10)
        counter=0;
        hubsan_build_bind_packet(0x0A);
        A7105_Strobe(A7105_STANDBY);
        A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
        strobeTXRX();
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (counter==9){
            break;
        }
    }
	
    A7105_WriteRegister(A7105_1F_CODE_I,0x0F); //CRC option CRC enabled adress 0x1f data 1111(CRCS=1,IDL=4bytes,PML[1:1]=4 bytes)
    //A7105_WriteRegister(0x28, 0x1F);//set Power to "1" dbm max value.
    A7105_Strobe(A7105_STANDBY);
    for(int i=0;i<4;i++){
        txid[i]=packet[i+11];
    }
}

void initrx(void)
{
    lib_soft_3_wire_spi_init(A7105_SDIO, A7105_SCK, A7105_SCS);
    lib_timers_delaymilliseconds(10);
    init_a7105();
    bind();
    A7105_Strobe(A7105_RX);
}

void decodepacket()
{
    if(packet[0]==0x20) {
        // converts [0;255] to [-1;1] fixed point num
        lib_fp_lowpassfilter(&global.rxvalues[THROTTLEINDEX], ((fixedpointnum) packet[2] - 0x80) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
        lib_fp_lowpassfilter(&global.rxvalues[YAWINDEX], ((fixedpointnum) packet[4] - 0x80) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
        lib_fp_lowpassfilter(&global.rxvalues[PITCHINDEX], ((fixedpointnum) 0x80 - packet[6]) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
        lib_fp_lowpassfilter(&global.rxvalues[ROLLINDEX], ((fixedpointnum) 0x80 - packet[8]) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
        // "LEDs" channel, AUX1 (only on H107L, H107C, H107D and Deviation TXs, high by default)
        lib_fp_lowpassfilter(&global.rxvalues[AUX1INDEX], ((fixedpointnum) (packet[9] & AUX1_FLAG ? 0x7F : -0x7F)) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
        // "Flip" channel, AUX2 (only on H107L, H107C, H107D and Deviation TXs, high by default)
        lib_fp_lowpassfilter(&global.rxvalues[AUX2INDEX], ((fixedpointnum) (packet[9] & AUX2_FLAG ? 0x7F : -0x7F)) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
    }
}

void readrx(void) // todo : telemetry
{
    if( lib_timers_gettimermicroseconds(timeout_timer) > 14000) {
        timeout_timer = lib_timers_starttimer();
        A7105_Strobe(A7105_RX);
    }
    if(A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK)
        return; // nothing received
    A7105_ReadPayload((uint8_t*)&packet, sizeof(packet)); 
    if(!((packet[11]==txid[0])&&(packet[12]==txid[1])&&(packet[13]==txid[2])&&(packet[14]==txid[3])))
        return; // not our TX !
    if(!hubsan_check_integrity())
        return; // bad checksum
    timeout_timer = lib_timers_starttimer();
    A7105_Strobe(A7105_RST_RDPTR);
    A7105_Strobe(A7105_RX);
    decodepacket();
    // reset the failsafe timer
    global.failsafetimer = lib_timers_starttimer();
}

#endif