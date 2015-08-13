/* 
Copyright 2014 Victor Joukov

Some of this code is based on Multiwii code by Alexandre Dubus (www.multiwii.com)

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
#include "bradwii.h"
#include "rx.h"
#include "defs.h"
#include "lib_timers.h"
#include "nrf24l01.h"
//#include "lib_digitalio.h"
//#include "lib_serial.h"

// when adding new receivers, the following functions must be included:
// initrx()   // initializes the r/c receiver
// readrx()   // loads global.rxvalues with r/c values as fixedpointnum's from -1 to 1 (0 is the center).

unsigned char v2x2_channelindex[] = { THROTTLEINDEX,YAWINDEX,PITCHINDEX,ROLLINDEX,AUX1INDEX,AUX2INDEX,AUX3INDEX,AUX4INDEX,8,9,10,11 };

extern globalstruct global;
extern usersettingsstruct usersettings;        // user editable variables


#define BV(x) (1 << (x))

///////////////////////////////////////////////////////////////////////
// Definitions for a group of nRF24L01 based protocols
enum Proto {
    PROTO_NONE = 0,
    PROTO_V2X2,
    PROTO_HISKY,
    PROTO_SLT,
};


///////////////////////////////////////////////////////////////////////
// V2X2 protocol

#define V2X2_PAYLOAD_SIZE 16
#define V2X2_NFREQCHANNELS 16

enum {
    V2X2_FLAG_CAMERA = 0x01, // also automatic Missile Launcher and Hoist in one direction
    V2X2_FLAG_VIDEO  = 0x02, // also Sprayer, Bubbler, Missile Launcher(1), and Hoist in the other dir.
    V2X2_FLAG_FLIP   = 0x04,
    V2X2_FLAG_UNK9   = 0x08,
    V2X2_FLAG_LED    = 0x10,
    V2X2_FLAG_UNK10  = 0x20,
    V2X2_FLAG_BIND   = 0xC0
};

// This is frequency hopping table for V202 protocol
// The table is the first 4 rows of 32 frequency hopping
// patterns, all other rows are derived from the first 4.
// For some reason the protocol avoids channels, dividing
// by 16 and replaces them by subtracting 3 from the channel
// number in this case.
// The pattern is defined by 5 least significant bits of
// sum of 3 bytes comprising TX id
static const uint8_t v2x2_freq_hopping[][V2X2_NFREQCHANNELS] = {
 { 0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36,
   0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18 }, //  00
 { 0x2E, 0x33, 0x25, 0x38, 0x19, 0x12, 0x18, 0x16,
   0x2A, 0x1C, 0x1F, 0x37, 0x2F, 0x23, 0x34, 0x10 }, //  01
 { 0x11, 0x1A, 0x35, 0x24, 0x28, 0x18, 0x25, 0x2A,
   0x32, 0x2C, 0x14, 0x27, 0x36, 0x34, 0x1C, 0x17 }, //  02
 { 0x22, 0x27, 0x17, 0x39, 0x34, 0x28, 0x2B, 0x1D,
   0x18, 0x2A, 0x21, 0x38, 0x10, 0x26, 0x20, 0x1F }  //  03
};

static uint8_t rf_channels[MAXFHSIZE];
static uint8_t packet[V2X2_PAYLOAD_SIZE];
static uint8_t rf_ch_num;
static uint8_t nfreqchannels;
static uint8_t bind_phase;
static uint8_t boundprotocol;
static uint8_t tryprotocol;
static uint32_t packet_timer;
//static uint32_t rx_timeout;
//static uint32_t valid_packets;
//static uint32_t missed_packets;
//static uint32_t bad_packets;
#define valid_packets (global.debugvalue[0])
#define missed_packets (global.debugvalue[1])
#define bad_packets (global.debugvalue[2])
#define rx_timeout (global.debugvalue[3])

enum {
    PHASE_NOT_BOUND = 0,
    PHASE_JUST_BOUND,
    PHASE_LOST_BINDING,
    PHASE_BOUND
};

static void v2x2_set_tx_id(uint8_t *id)
{
    uint8_t sum;
    boundprotocol = usersettings.boundprotocol = PROTO_V2X2;
    usersettings.txidsize = 3;
    usersettings.txid[0] = id[0];
    usersettings.txid[1] = id[1];
    usersettings.txid[2] = id[2];
    sum = id[0] + id[1] + id[2];
    usersettings.fhsize = V2X2_NFREQCHANNELS;
    
    // Base row is defined by lowest 2 bits
    const uint8_t *fh_row = v2x2_freq_hopping[sum & 0x03];
    // Higher 3 bits define increment to corresponding row
    uint8_t increment = (sum & 0x1e) >> 2;
    for (int i = 0; i < V2X2_NFREQCHANNELS; ++i) {
        uint8_t val = fh_row[i] + increment;
        // Strange avoidance of channels divisible by 16
        usersettings.freqhopping[i] = (val & 0x0f) ? val : val - 3;
    }
}

static void set_bound()
{
    boundprotocol = usersettings.boundprotocol;
    for (int i = 0; i < usersettings.fhsize; ++i) {
        rf_channels[i] = usersettings.freqhopping[i];
    }
    nfreqchannels = usersettings.fhsize;
    rx_timeout = 1000L; // find the channel as fast as possible
}

static void prepare_to_bind(void)
    {
    packet_timer = lib_timers_starttimer();
    tryprotocol = PROTO_V2X2;
    for (int i = 0; i < V2X2_NFREQCHANNELS; ++i) {
        rf_channels[i] = v2x2_freq_hopping[0][i];
    }
    nfreqchannels = V2X2_NFREQCHANNELS;
    rx_timeout = 1000L;
    boundprotocol = PROTO_NONE;
}


static void switch_channel(void)
{
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_channels[rf_ch_num]);
    if (++rf_ch_num >= nfreqchannels) rf_ch_num = 0;
}

// The Beken radio chip can be improperly reset
// We try to set it into Bank 0, before that we try
// to verify that it is there at all
static void reset_beken(void)
{
    NRF24L01_Activate(0x53); // magic for BK2421/BK2423 bank switch
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x80) {
        NRF24L01_Activate(0x53); // switch to register set 0
    }
}

// Check for Beken BK2421/BK2423 chip
// It is done by using Beken specific activate code, 0x53
// and checking that status register changed appropriately
// There is no harm to run it on nRF24L01 because following
// closing activate command changes state back even if it
// does something on nRF24L01
// For detailed description of what's happening here see 
//   http://www.inhaos.com/uploadfile/otherpic/BK2423%20Datasheet%20v2.0.pdf
//   http://www.inhaos.com/uploadfile/otherpic/AN0008-BK2423%20Communication%20In%20250Kbps%20Air%20Rate.pdf
static void initialize_beken(void)
{
    NRF24L01_Activate(0x53); // magic for BK2421/BK2423 bank switch
    printf("Trying to switch banks\n");
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x80) {
        printf("BK2421 detected\n");
        // Beken registers don't have such nice names, so we just mention
        // them by their numbers
        // It's all magic, eavesdropped from real transfer and not even from the
        // data sheet - it has slightly different values
        NRF24L01_WriteRegisterMulti(0x00, (uint8_t *) "\x40\x4B\x01\xE2", 4);
        NRF24L01_WriteRegisterMulti(0x01, (uint8_t *) "\xC0\x4B\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x02, (uint8_t *) "\xD0\xFC\x8C\x02", 4);
        NRF24L01_WriteRegisterMulti(0x03, (uint8_t *) "\xF9\x00\x39\x21", 4); // V202
//        NRF24L01_WriteRegisterMulti(0x03, (uint8_t *) "\x99\x00\x39\x41", 4); // Beken datasheet
        NRF24L01_WriteRegisterMulti(0x04, (uint8_t *) "\xC1\x96\x9A\x1B", 4); // V202

        NRF24L01_WriteRegisterMulti(0x05, (uint8_t *) "\x24\x06\x7F\xA6", 4); // Disable RSSI
//        NRF24L01_WriteRegisterMulti(0x05, (uint8_t *) "\x3C\x02\x7F\xA6", 4); // Enable RSSI

//        NRF24L01_WriteRegisterMulti(0x0C, (uint8_t *) "\x00\x12\x73\x00", 4); // PLL locking time 120us, like BK2421
        NRF24L01_WriteRegisterMulti(0x0C, (uint8_t *) "\x00\x12\x73\x05", 4); // PLL locking time 130us, like nRF24L01
//        NRF24L01_WriteRegisterMulti(0x0D, (uint8_t *) "\x46\xB4\x80\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0D, (uint8_t *) "\x36\xB4\x80\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0E, (uint8_t *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);

        NRF24L01_WriteRegisterMulti(0x04, (uint8_t *) "\xC7\x96\x9A\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x04, (uint8_t *) "\xC1\x96\x9A\x1B", 4);
    } else {
        printf("nRF24L01 detected\n");
    }
    NRF24L01_Activate(0x53); // switch bank back
}

void initrx(void)
{
    NRF24L01_Initialize();
    
    reset_beken();

    // 2-bytes CRC, radio off
    uint8_t config = BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PRIM_RX);

    NRF24L01_WriteReg(NRF24L01_00_CONFIG, config); 
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xFF); // 4ms retransmit t/o, 15 tries
//    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);      // Channel 8 - bind
    NRF24L01_SetBitrate(NRF24L01_BR_1M);                          // 1Mbps
    NRF24L01_SetPower(TXPOWER_100mW);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, V2X2_PAYLOAD_SIZE);  // bytes of data payload for pipe 0
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here
    uint8_t rx_tx_addr[] = {0x66, 0x88, 0x68, 0x68, 0x68};
//    uint8_t rx_p1_addr[] = {0x88, 0x66, 0x86, 0x86, 0x86};
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, 5);
//    NRF24L01_WriteRegisterMulti(NRF24L01_0B_RX_ADDR_P1, rx_p1_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);
    
    initialize_beken();

    lib_timers_delaymilliseconds(50);

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();

    rf_ch_num = 0;

    // Turn radio power on
    config |= BV(NRF24L01_00_PWR_UP);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, config);
    // delayMicroseconds(150);
    lib_timers_delaymilliseconds(1); // 6 times more than needed
    
    valid_packets = missed_packets = bad_packets = 0;
    
    if (usersettings.boundprotocol == PROTO_NONE) {
        bind_phase = PHASE_NOT_BOUND;
        prepare_to_bind();
    } else {
        // Prepare to listen to bound protocol, if fails
        // try to bind
        bind_phase = PHASE_JUST_BOUND;
        set_bound();
    }
    switch_channel();
}

static void decode_bind_packet(uint8_t *packet)
{
    switch (tryprotocol) {
    case PROTO_V2X2:
        if ((packet[14] & V2X2_FLAG_BIND) == V2X2_FLAG_BIND) {
            // Fill out usersettings with bound protocol parameters
            v2x2_set_tx_id(&packet[7]);
            // Read usersettings into current values
            bind_phase = PHASE_BOUND;
            set_bound();
        }
        break;
    }
}

// Returns whether the data was successfully decoded
static bool decode_packet(uint8_t *packet, uint16_t *data)
{
    switch (boundprotocol) {
    case PROTO_NONE:
        decode_bind_packet(packet);
        break;
    case PROTO_V2X2:
        // Decode packet
        if ((packet[14] & V2X2_FLAG_BIND) == V2X2_FLAG_BIND) {
            return false;
        }
        if (packet[7] != usersettings.txid[0] ||
            packet[8] != usersettings.txid[1] ||
            packet[9] != usersettings.txid[2])
        {
            bad_packets++;
            return false;
        }
        // Restore regular interval
        rx_timeout = 10000L; // 4ms interval, duplicate packets, (8ms unique) + 25%
        // TREA order in packet to MultiWii order is handled by
        // correct assignment to channelindex
        // Throttle 0..255 to 1000..2000
        data[v2x2_channelindex[0]] = ((uint16_t)packet[0]) * 1000 / 255 + 1000;
        for (int i = 1; i < 4; ++i) {
            uint8_t a = packet[i];
            data[v2x2_channelindex[i]] = ((uint16_t)(a < 0x80 ? 0x7f - a : a)) * 1000 / 255 + 1000;
        }
        uint8_t flags[] = {V2X2_FLAG_LED, V2X2_FLAG_FLIP, V2X2_FLAG_CAMERA, V2X2_FLAG_VIDEO}; // two more unknown bits
        for (int i = 4; i < 8; ++i) {
            data[v2x2_channelindex[i]] = 1000 + ((packet[14] & flags[i-4]) ? 1000 : 0);
        }
        packet_timer = lib_timers_starttimer();
        if (++valid_packets > 50) bind_phase = PHASE_BOUND;
        return true;
    }
    return false;
}

void readrx(void)
{
    int chan;
    uint16_t data[8];

    if (!(NRF24L01_ReadReg(NRF24L01_07_STATUS) & BV(NRF24L01_07_RX_DR))) {
        uint32_t t = lib_timers_gettimermicroseconds(packet_timer);
        if (t > rx_timeout) {
            if (boundprotocol != PROTO_NONE) {
                if (++missed_packets > 500 && bind_phase == PHASE_JUST_BOUND) {
                    valid_packets = missed_packets = bad_packets = 0;
                    bind_phase = PHASE_LOST_BINDING;
                    prepare_to_bind();
                }
            } else switch_channel();
            packet_timer = lib_timers_starttimer();
        }
        return;
    }
    packet_timer = lib_timers_starttimer();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BV(NRF24L01_07_RX_DR));
    NRF24L01_ReadPayload(packet, V2X2_PAYLOAD_SIZE);
    NRF24L01_FlushRx();
    switch_channel();
    if (!decode_packet(packet, data))
        return;
    
    for (chan = 0; chan < 8; ++chan) {
//        data = pwmRead(chan);
//    if (data < 750 || data > 2250)
//        data = 1500;

        // convert from 1000-2000 range to -1 to 1 fixedpointnum range and low pass filter to remove glitches
        lib_fp_lowpassfilter(&global.rxvalues[chan], ((fixedpointnum) data[chan] - 1500) * 131L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
    }
    // reset the failsafe timer
    global.failsafetimer = lib_timers_starttimer();
}
