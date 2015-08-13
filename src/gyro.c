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

#include "gyro.h"
#include "lib_i2c.h"
#include "lib_timers.h"
#include "rx.h"
#include "lib_fp.h"
#include "bradwii.h"

extern globalstruct global;

// when adding gyros, the following functions need to be included:
// initgyro() // initializes the gyro
// readgyro() // loads global.gyrorate with gyro readings in fixedpointnum degrees per second
#if (GYRO_TYPE==MPU3050)
#define MPU3050_ADDRESS     0x68        // address pin AD0 low (GND)
#if (GYRO_LOW_PASS_FILTER<=6)
#define MPU3050_DLPF_CFG GYRO_LOW_PASS_FILTER
#else
#define MPU3050_DLPF_CFG   6
#endif

void initgyro(void)
{
    lib_i2c_writereg(MPU3050_ADDRESS, 0x3E, 0x80);  //PWR_MGMT_1 -- DEVICE_RESET 1
    lib_timers_delaymilliseconds(5);
    lib_i2c_writereg(MPU3050_ADDRESS, 0x3E, 0x03); //PWR_MGMT_1 -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    lib_i2c_writereg(MPU3050_ADDRESS, 0x16, MPU3050_DLPF_CFG + 0x18); // Gyro CONFIG -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => GYRO bandwidth = 256Hz); -- FS_SEL = 3: Full scale set to 2000 deg/sec
}

void readgyro(void)
{
    unsigned char data[6];
    lib_i2c_readdata(MPU3050_ADDRESS, 0x1D, (unsigned char *) &data, 6);
	
    // convert to fixedpointnum, in degrees per second
    // the gyro puts out a 16 bit signed int where each count equals 0.0609756097561 degrees/second
    // So we have 15 bit fractional part, need to shift that to FIXEDPOINTSHIFT and
    // take 2000deg/s into account.
    // This only works if FIXEDPOINTSHIFT >= 15
    GYRO_ORIENTATION(global.gyrorate,
        ((int16_t) ((data[0] << 8) | data[1])) * (2000L << (FIXEDPOINTSHIFT - 15)),
        ((int16_t) ((data[2] << 8) | data[3])) * (2000L << (FIXEDPOINTSHIFT - 15)),
        ((int16_t) ((data[4] << 8) | data[5])) * (2000L << (FIXEDPOINTSHIFT - 15)));
}

#elif (GYRO_TYPE==ITG3200)
// ************************************************************************************************************
// I2C Gyroscope ITG3200 
// ************************************************************************************************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND)
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
// ************************************************************************************************************
#if !defined(ITG3200_ADDRESS)
#define ITG3200_ADDRESS 0X68
#endif
#if (GYRO_LOW_PASS_FILTER<=5)
#define ITG3200_DLPF_CFG GYRO_LOW_PASS_FILTER
#else
#define ITG3200_DLPF_CFG 5
#endif

void initgyro(void)
{
    lib_timers_delaymilliseconds(100);
    lib_i2c_writereg(ITG3200_ADDRESS, 0x3E, 0x80);      //register: Power Management  --  value: reset device
    lib_timers_delaymilliseconds(5);
    lib_i2c_writereg(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG);   //register: DLPF_CFG - low pass filter configuration
    lib_timers_delaymilliseconds(5);
    lib_i2c_writereg(ITG3200_ADDRESS, 0x3E, 0x03);      //register: Power Management  --  value: PLL with Z Gyro reference
    lib_timers_delaymilliseconds(100);
}

void readgyro(void)
{
    unsigned char data[6];
    lib_i2c_readdata(ITG3200_ADDRESS, 0X1D, (unsigned char *) &data, 6);

    // convert to fixedpointnum, in degrees per second
    // the gyro puts out an int where each count equals 0.0695652173913 degrees/second
    // we want fixedpointnums, so we multiply by 4559 (0.0695652173913 * (1<<FIXEDPOINTSHIFT))
    GYRO_ORIENTATION(global.gyrorate, ((data[0] << 8) | data[1]) * 4559L,       // range: +/- 8192; +/- 2000 deg/sec
                      ((data[2] << 8) | data[3]) * 4559L, ((data[4] << 8) | data[5]) * 4559L);
}

#elif (GYRO_TYPE==MPU6050) 
#define MPU6050_ADDRESS     0x68        // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
#if (GYRO_LOW_PASS_FILTER<=6)
#define MPU6050_DLPF_CFG GYRO_LOW_PASS_FILTER
#else
#define MPU6050_DLPF_CFG   6
#endif

void initgyro(void)
{
    // Resetting the MPU6050 does not work for some reason.
    // But it is already in default mode at power up, so there is no need to reset it anyways

    //lib_i2c_writereg(MPU6050_ADDRESS, 0x6B, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    //lib_timers_delaymilliseconds(10);
    lib_i2c_writereg(MPU6050_ADDRESS, 0x6B, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    lib_i2c_writereg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG);  //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    lib_i2c_writereg(MPU6050_ADDRESS, 0x1B, 0x18);      //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
}

void readgyro(void)
{
    unsigned char data[6];
    lib_i2c_readdata(MPU6050_ADDRESS, 0x43, (unsigned char *) &data, 6);
    // convert to fixedpointnum, in degrees per second
    // the gyro puts out an int where each count equals 0.0609756097561 degrees/second
    // we want fixedpointnums, so we multiply by 3996 (0.0609756097561 * (1<<FIXEDPOINTSHIFT))
    GYRO_ORIENTATION(global.gyrorate,
        ((int16_t) ((data[0] << 8) | data[1])) * 3996L,       // range: +/- 8192; +/- 2000 deg/sec
        ((int16_t) ((data[2] << 8) | data[3])) * 3996L,
        ((int16_t) ((data[4] << 8) | data[5])) * 3996L);
}
#endif
