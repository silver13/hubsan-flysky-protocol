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

#include "output.h"
#include "projectsettings.h"
#include "bradwii.h"
#include "lib_timers.h"
#include "drv_pwm.h"

extern globalstruct global;

#ifdef DC_MOTORS
   // for dc motors, we reduce the top so that we can switch at 8khz
#define TOPMOTORCOUNT16BIT 0x3FF
#define TOPMOTORCOUNT11BIT 0x3FF
#define PRESCALER11BIT PWM411BITPRESCALER1
#else
   // for speed controllers, we switch at about 490Hz
#define TOPMOTORCOUNT16BIT 0x3FFF
#define TOPMOTORCOUNT11BIT 0x7FF        // top is smaller, but we use a bigger prescaller so the cycle time is correct
#define PRESCALER11BIT PWM411BITPRESCALER16
#endif

void initoutputs(void)
{
    setallmotoroutputs(MIN_MOTOR_OUTPUT);
}

void setmotoroutput(unsigned char motornum, unsigned char motorchannel, fixedpointnum fpvalue)
{
    // set the output of a motor
    // convert from fixedpoint 0 to 1 into int 1000 to 2000
    int value = 1000 + ((fpvalue * 1000L) >> FIXEDPOINTSHIFT);

    if (value < ARMED_MIN_MOTOR_OUTPUT)
        value = ARMED_MIN_MOTOR_OUTPUT;
    if (value > MAX_MOTOR_OUTPUT)
        value = MAX_MOTOR_OUTPUT;
    setoutput(motorchannel, value);

    global.motoroutputvalue[motornum] = value;
}

void setallmotoroutputs(int value)
{
    int x;
    for (x = 0; x < NUMMOTORS; ++x) {
        global.motoroutputvalue[x] = value;
        setoutput(x, value);
    }
}

void setoutput(unsigned char outputchannel, unsigned int value)
{
    // value is from 1000 to 2000
    pwmWriteMotor(outputchannel, value);
}
