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

#include "hal.h"
#include "lib_timers.h"

// This code creates a timer to count microseconds and includes support for using this timer to run multiple
// software timers.  The timer can run on either TIMER0 or TIMER1.  When run on TIMER1, a 16 bit timer is used
// which gives us microsecond resolution and less interrupt overhead.  When run on TIMER0, we get 4 microsecond
// resolution and more overhead.  Unsigned longs are used to store microseconds, so the longest intervals that
// can be measured without extra code is about 70 minutes.

// Usage:

// half second delay done multiple ways

// inittimers();
// unsigned long mytimer=lib_timers_starttimer();
// while (lib_timers_gettimermicroseconds(mytimer)<500000L) {}

// inittimers();
//    lib_timers_delaymilliseconds(500) {}

// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
static uint32_t sysTickLimit;

// SysTick
void SysTick_Handler(void)
{
    sysTickUptime++;
}

// needs to be called once in the program before timers can be used
void lib_timers_init(void)
{                               
    // SysTick
    sysTickLimit = SystemCoreClock / 1000;
    SysTick_Config(sysTickLimit);
}

uint32_t lib_timers_getcurrentmicroseconds(void)
{
    // returns microseconds since startup.  This mainly used internally because it wraps around.
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    // goebish timer patch, modified by victzh for performance and clarity
    return (ms * 1000) + (sysTickLimit - cycle_cnt) / CyclesPerUs;
}

unsigned long lib_timers_gettimermicroseconds(unsigned long starttime)
{
    // returns microseconds since this timer was started
    unsigned long currenttime = lib_timers_getcurrentmicroseconds();
    if (starttime > currenttime)    // we have wrapped around
    {
        return (0xFFFFFFFF - starttime + currenttime);
    } else {
        return (currenttime - starttime);
    }
}

unsigned long lib_timers_gettimermicrosecondsandreset(unsigned long *starttime)
{
    // returns microseconds since this timer was started and then reset the start time
    // this allows us to keep checking the time without losing any time
    unsigned long currenttime = lib_timers_getcurrentmicroseconds();
    unsigned long returnvalue;

    if (*starttime > currenttime)       // we have wrapped around
    {
        returnvalue = (0xFFFFFFFF - *starttime + currenttime);
    } else {
        returnvalue = (currenttime - *starttime);
    }
    *starttime = currenttime;
    return (returnvalue);
}

unsigned long lib_timers_starttimer()
{                               // start a timer
    return (lib_timers_getcurrentmicroseconds());
}

void lib_timers_delaymilliseconds(unsigned long delaymilliseconds)
{
    unsigned long timercounts = lib_timers_starttimer();
    while (lib_timers_gettimermicroseconds(timercounts) < delaymilliseconds * 1000L) {
    }
}
