/* 
flysky protocol for hubsan x4
by silverx 2015

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


#include "bradwii.h"
#include "rx.h"
#include "lib_soft_3_wire_spi.h"
#include "lib_timers.h"
#include "a7105.h"
#include "config_X4.h"

#ifdef FLYSKY_RX

#define A7105_SCS   (DIGITALPORT1 | 4)
#define A7105_SCK   (DIGITALPORT1 | 3)
#define A7105_SDIO  (DIGITALPORT1 | 2)

// channel gain for first 4 channels - throttle excepted
// imho it's better to change LEVEL_MODE_MAX_TILT since it is multiplied with the rx value
// that way it will still indicate the actual angle
#define CHANNEL_GAIN 131 

// a separate throttle gain for compatibility
// slightly higher to make sure we don't lose throttle at max
#define THROTTLE_GAIN 133

// a separate switch gain for the aux channels
#define SWITCH_GAIN 131

// offset to substract from ppm value so that it centers at zero
#define PPM_OFFSET 1500



// channel hopping time in the flysky / turnigy protocol ( in uS )
#define HOP_TIME 1450

// delay induced by spi when a packet is received ( in uS )
// set to measured value or a bit higher ( actual is 2200 )
#define SPI_DELAY 2600


// if ANY_TX is defined and id = 0 zero it will lock on *any* transmitter (without bind)
// it will forget the tx at *rx* poweroff
// use with caution
//#define ANY_TX

// the saving of tx id is not implemented right now
// you could hardwire it here
static uint32_t id = 0;

// Swap yaw and roll
// this might be needed by someone
//#define SWAP_YAW_AND_ROLL

#ifdef ANY_TX
#warning ANY_TX is on
#endif

// channel hopping table
// made smaller to save ROM, some is done in software
// based on the fact that every second row was the previous back to front
// even columns are the previous column value + 0x50
// may help save some (more) memory sometime
static const uint8_t tx_channels[8][16] = {
  {0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
  {0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
  {0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
  {0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
  {0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
  {0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
  {0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
  {0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
};

static uint8_t chanrow;
static uint8_t chancol;
static int chanoffset;
static int chandirection;


static uint8_t packet[21];
static unsigned long timeout_timer;

void init_a7105(void);
int checkpacket( void);
extern globalstruct global;
void nextchannel( void);
void sethopping ( uint32_t );
void bind( void);

void init_a7105(void)
{
  A7105_Reset();
	// the id could maybe be included in the register/command array too
	A7105_WriteID(0x5475c52A);//A7105 id

const int data[32] = { 
								A7105_01_MODE_CONTROL, 0x42 , 
								A7105_0D_CLOCK , 0x05,
								A7105_18_RX, 0x62,
								A7105_19_RX_GAIN_I, 0x80,
								A7105_1C_RX_GAIN_IV, 0x0A,
								A7105_1F_CODE_I, 0x0f,
								A7105_20_CODE_II, 0x1E, // 16h recommended
								A7105_29_RX_DEM_TEST_I, 0x47 ,
							A7105_PLL ,	// strobe command
								0x02 , 0x01,
								A7105_03_FIFOI, 0x14 ,
								0x24 , 0x13 ,
								0x26 , 0x3b	,
								0x02 , 0x02	,
								0x0F , 0xA0 , 
								0x02 , 0x02 ,
							A7105_STANDBY }; // strobe command

// set registers and calibration ( all in one)
	lib_soft_3_wire_spi_setCS(DIGITALOFF);
	for (int i = 0 ; i <= 31 ; i = i+1)
	{
    lib_soft_3_wire_spi_write(data[i]);
	}	
	lib_soft_3_wire_spi_setCS(DIGITALON);
	
}


void initrx(void)
{
  lib_soft_3_wire_spi_init(A7105_SDIO, A7105_SCK, A7105_SCS);
  lib_timers_delaymilliseconds(10);
  init_a7105();
//bind only id anytx if off
#ifndef ANY_TX
	bind();
	// save id here 
	// ( not implemented)
	// savetxid(id);
#endif
	
	sethopping(id);
	chancol=0;
	nextchannel();
}


void bind()
{
	// set channel 0;
	A7105_Strobe(0xA0);
	A7105_WriteRegister(A7105_0F_PLL_I,00);
	A7105_Strobe(A7105_RX);
	while(1)
	{
	if( lib_timers_gettimermicroseconds(0) % 524288 > 262144)
            x4_set_leds(X4_LED_FR | X4_LED_RL);
        else
            x4_set_leds(X4_LED_FL | X4_LED_RR);
				
	char mode = A7105_ReadRegister(A7105_00_MODE);
//	if(mode & A7105_MODE_TRER_MASK  )	
	//	{
//		continue;
//		}
	if(mode & A7105_MODE_TRER_MASK || mode & (1<<6) ||  mode & ( 1<<5) )	
		{
		A7105_Strobe(A7105_RST_RDPTR); 
		A7105_Strobe(A7105_RX);
		continue;
		}
	A7105_ReadPayload((uint8_t*)&packet, sizeof(packet)); 
  A7105_Strobe(A7105_RST_RDPTR);
	if ( packet[0] == 170 )  // 170
			{
			int i;
			for ( i = 5 ; i < 21; i++)
					{
						if ( packet[i]!= 0xFF) 
						{
							break;
						}
					}
			if ( i== 21) 
				{
				//set found tx		
			  id = ( ( packet[1] << 0 |  packet[2] <<8 |  packet[3] << 16 | packet[4]<<24 ) );
				break;
				}
			}
	 A7105_Strobe(A7105_RX);		
	}
	
}

void decodepacket()
{
	// converts [0;XXXX] to [-1;1] fixed point num
	// throttle multiplier slightly higher so it reaches 65535 at default 100% rates
	lib_fp_lowpassfilter(&global.rxvalues[THROTTLEINDEX], ( ((uint32_t) (packet[9]+256*packet[10])) - PPM_OFFSET ) * THROTTLE_GAIN , global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
	lib_fp_lowpassfilter(&global.rxvalues[PITCHINDEX], ( ((uint32_t) (packet[7]+256*packet[8])) - PPM_OFFSET ) * CHANNEL_GAIN , global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);

#ifdef SWAP_YAW_AND_ROLL
		lib_fp_lowpassfilter(&global.rxvalues[YAWINDEX], ( ((uint32_t) (packet[5]+256*packet[6])) - PPM_OFFSET ) * CHANNEL_GAIN, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
		lib_fp_lowpassfilter(&global.rxvalues[ROLLINDEX], ( ((uint32_t) (packet[11]+256*packet[12])) - PPM_OFFSET ) * CHANNEL_GAIN , global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
#else
		lib_fp_lowpassfilter(&global.rxvalues[ROLLINDEX], ( ((uint32_t) (packet[5]+256*packet[6])) - PPM_OFFSET ) * CHANNEL_GAIN, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
		lib_fp_lowpassfilter(&global.rxvalues[YAWINDEX], ( ((uint32_t) (packet[11]+256*packet[12])) - PPM_OFFSET ) * CHANNEL_GAIN , global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
#endif
	
	// AUX1 == CH5
	lib_fp_lowpassfilter(&global.rxvalues[AUX1INDEX], ( ((uint32_t) (packet[13]+256*packet[14])) - PPM_OFFSET ) * SWITCH_GAIN , global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
	// AUX2 == CH6
	lib_fp_lowpassfilter(&global.rxvalues[AUX2INDEX], ( ((uint32_t) (packet[15]+256*packet[16])) - PPM_OFFSET ) * SWITCH_GAIN , global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);

#if (RXNUMCHANNELS>6)  
	lib_fp_lowpassfilter(&global.rxvalues[AUX3INDEX], ( ((uint32_t) (packet[17]+256*packet[18])) - PPM_OFFSET ) * SWITCH_GAIN , global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
#endif
#if (RXNUMCHANNELS>7)
	lib_fp_lowpassfilter(&global.rxvalues[AUX4INDEX], ( ((uint32_t) (packet[19]+256*packet[20])) - PPM_OFFSET ) * SWITCH_GAIN , global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
#endif
// this is done in other places too, but better safe then sorry
  lib_fp_constrain(&global.rxvalues[THROTTLEINDEX], -FIXEDPOINTONE, FIXEDPOINTONE);
	//  lib_fp_constrain(&global.rxvalues[ROLLINDEX], -FIXEDPOINTONE, FIXEDPOINTONE);
	//  lib_fp_constrain(&global.rxvalues[PITCHINDEX], -FIXEDPOINTONE, FIXEDPOINTONE);
	//	lib_fp_constrain(&global.rxvalues[YAWINDEX], -FIXEDPOINTONE, FIXEDPOINTONE);
}

void sethopping( uint32_t tx_id)
{
	chanrow= tx_id % 16;
	if (chanrow % 2) 	chandirection = -1;	// reverse direction
		else chandirection = 1;		// forward direction
	
	chanrow = chanrow>>1;				
	chanoffset = (tx_id & 0xff) / 16;				
	if(chanoffset > 9) chanoffset = 9;	
	return;
}

void nextchannel( )
{	
	  int8_t channel;
	  chancol = (chancol + chandirection) % 16;
		channel=tx_channels[chanrow][chancol]-chanoffset;
		channel-=1;
		A7105_Strobe(0xA0);
		A7105_WriteRegister(A7105_0F_PLL_I,channel);
		A7105_Strobe(A7105_RX);
}

// debug
//static unsigned long packet_timer;
//static unsigned long packettime ;

//static unsigned long test1 ;
//static unsigned long test3 ;



// this is designed to work if loop time is higher then the channel hopping time
// it skips several channels ahead to account for the delay

void readrx(void)
{
	char mode = A7105_ReadRegister(A7105_00_MODE);
	if(mode & A7105_MODE_TRER_MASK)
		{// nothing received
		if( lib_timers_gettimermicroseconds(timeout_timer) >28000) 
		{// change channel in case there is no reception in it
			#ifdef ANY_TX
			if ( id == 0) 
			{ // we have no tx id
				// skip thru channels looking for packets
				sethopping( rand() % 256 );
			}	
			#endif
		nextchannel();
	  timeout_timer = lib_timers_starttimer();
	  //return;
    }
		return;
		}
  if( mode & (1<<6) || ( mode & ( 1<<5) ) )
		{// fec and crc check
		// i think fec is not used by flysky
		// bad packet received ( or background noise)
  	A7105_Strobe(A7105_RST_RDPTR); 
		A7105_Strobe(A7105_RX);
		//A7105_WriteRegister( A7105_RST_RDPTR , A7105_RX );
    return;
		}
  A7105_ReadPayload((uint8_t*)&packet, sizeof(packet)); 
  A7105_Strobe(A7105_RST_RDPTR);
		
	if (!checkpacket() )
		{
		// invalid packet which passed crc or bind packet
	  A7105_Strobe(A7105_RX);
		return;
		}
		
	uint32_t idreceived;
	idreceived = ( ( packet[1] << 0 |  packet[2] <<8 |  packet[3] << 16 | packet[4]<<24 ) );
#ifdef ANY_TX		
 if ( id == 0 ) 
		{// if we have no id save the found id here
			id = idreceived;
			sethopping(id);
		}
#endif
 if( id != idreceived  )
	 {// different tx
	  A7105_Strobe(A7105_RX);
    return; 
	 }
 decodepacket();

// packettime = lib_timers_gettimermicroseconds(packet_timer);
// packet_timer = lib_timers_starttimer();
	 
// 2200 uS reading packet by softspi and other delays in this routine
// 1700uS loop time is adjusted dynamically 
// global.timesliver >> 4 == loop time in microseconds

 unsigned long offset = (global.timesliver >> 4) + SPI_DELAY;
 int skippackets = 0;
	 
 // this is basically division by HOP_TIME
 for ( int i = 0 ; i  < 9; i++)
 {
	if ( offset < HOP_TIME) break;
	offset = offset - HOP_TIME;
	skippackets++;
 }
// test1 = skippackets; // for debug
 //skip ahead this many channels
 chancol = (chancol +  skippackets ) % 16;
 // +1 channel in nextchannel
 nextchannel();
 
 timeout_timer = lib_timers_starttimer();
 // reset the failsafe timer
 global.failsafetimer = lib_timers_starttimer();
}

int checkpacket()
{
	if( packet[0] == 0x55 )
	 {// corrupt packet probably ( or bind packet)
    for ( int i = 6 ; i <= 16; i=i+2)
			{
			// upper bits of the channel msb's should be all 0 
			if( packet[i] & 0xF0)
				{// corrupt packet ( or strange transmitter protocol)
				return 0; 
				}
			}
		return 1;
	 }

	return 0;
}






#endif
