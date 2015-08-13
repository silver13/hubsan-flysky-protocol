#include <stdint.h>

// strobe commands 
#define A7105_SLEEP      0x80
#define A7105_IDLE       0x90
#define A7105_STANDBY    0xA0
#define A7105_PLL        0xB0
#define A7105_RX         0xC0
#define A7105_TX         0xD0
#define A7105_RST_WRPTR  0xE0
#define A7105_RST_RDPTR  0xF0

// registers
#define A7105_00_MODE          0x00
#define A7105_01_MODE_CONTROL  0x01
#define A7105_02_CALC          0x02
#define A7105_03_FIFOI         0x03
#define A7105_04_FIFOII        0x04
#define A7105_05_FIFO_DATA     0x05
#define A7105_06_ID_DATA       0x06
#define A7105_07_RC_OSC_I      0x07
#define A7105_08_RC_OSC_II     0x08
#define A7105_09_RC_OSC_III    0x09
#define A7105_0A_CK0_PIN       0x0A
#define A7105_0B_GPIO1_PIN1    0x0B
#define A7105_0C_GPIO2_PIN_II  0x0C
#define A7105_0D_CLOCK         0x0D
#define A7105_0E_DATA_RATE     0x0E
#define A7105_0F_PLL_I         0x0F
#define A7105_10_PLL_II        0x10
#define A7105_11_PLL_III       0x11
#define A7105_12_PLL_IV        0x12
#define A7105_13_PLL_V         0x13
#define A7105_14_TX_I          0x14
#define A7105_15_TX_II         0x15
#define A7105_16_DELAY_I       0x16
#define A7105_17_DELAY_II      0x17
#define A7105_18_RX            0x18
#define A7105_19_RX_GAIN_I     0x19
#define A7105_1A_RX_GAIN_II    0x1A
#define A7105_1B_RX_GAIN_III   0x1B
#define A7105_1C_RX_GAIN_IV    0x1C
#define A7105_1D_RSSI_THOLD    0x1D
#define A7105_1E_ADC           0x1E
#define A7105_1F_CODE_I        0x1F
#define A7105_20_CODE_II       0x20
#define A7105_21_CODE_III      0x21
#define A7105_22_IF_CALIB_I    0x22
#define A7105_23_IF_CALIB_II   0x23
#define A7105_24_VCO_CURCAL    0x24
#define A7105_25_VCO_SBCAL_I   0x25
#define A7105_26_VCO_SBCAL_II  0x26
#define A7105_27_BATTERY_DET   0x27
#define A7105_28_TX_TEST       0x28
#define A7105_29_RX_DEM_TEST_I 0x29
#define A7105_2A_RX_DEM_TEST_II 0x2A
#define A7105_2B_CPC           0x2B
#define A7105_2C_XTAL_TEST     0x2C
#define A7105_2D_PLL_TEST      0x2D
#define A7105_2E_VCO_TEST_I    0x2E
#define A7105_2F_VCO_TEST_II   0x2F
#define A7105_30_IFAT          0x30
#define A7105_31_RSCALE        0x31
#define A7105_32_FILTER_TEST   0x32

#define A7105_MODE_TRER_MASK	(uint8_t)(1 << 0) // TRX is enabled

void A7105_WriteID(uint32_t ida);
void A7105_ReadID(uint8_t *_aid);
void A7105_WritePayload(uint8_t *_packet, uint8_t len);
void A7105_ReadPayload(uint8_t *_packet, uint8_t len);
void A7105_Reset(void);
uint8_t A7105_ReadRegister(uint8_t address);
void A7105_WriteRegister(uint8_t address, uint8_t data);
void A7105_Strobe(uint8_t command);
