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

#include "lib_fp.h"
#include <stdint.h>

void lib_fp_constrain(fixedpointnum *lf, fixedpointnum low, fixedpointnum high)
{
    if (*lf < low)
        *lf = low;
    else if (*lf > high)
        *lf = high;
}

// returns the equivalent angle within the range -180 and 180 degrees
void lib_fp_constrain180(fixedpointnum *lf)
{
    while (*lf < -FIXEDPOINT180)
        *lf += FIXEDPOINT360;
    while (*lf > FIXEDPOINT180)
        *lf -= FIXEDPOINT360;
}

//             D1 C1 B1 A1 (x)
//           x D2 C2 B2 A2 (y)
// -----------------------
// z0 y0 D0 C0 B0 A0 x0 w0


// A2*A1 -> w0,x0 (skip)
// A2*B1 -> x0,A0 (unsigned, unsigned) 
// A2*C1 -> A0,B0 (unsigned, unsigned) // do first
// A2*D1 -> B0,C0 (unsigned, signed)
// B2*A1 -> x0,A0 (unsigned, unsigned)
// B2*B1 -> A0,B0 (unsigned, unsigned)
// B2*C1 -> B0,C0 (unsigned, unsigned) 
// B2*D1 -> C0,D0 (unsigned, signed) 
// C2*A1 -> A0,B0 (unsigned, unsigned)
// C2*B1   -> B0,C0 (unsigned, unsigned)
// C2*C1 -> C0,D0 (unsigned, unsigned) // do second
// C2*D1 -> D0,y0 (unsigned, signed)
// D2*A1 -> B0,C0 (signed, unsigned)
// D2*B1 -> C0,D0 (signed, unsigned)
// D2*C1 -> D0,y0 (signed, unsigned)
// D2*D1 -> y0,z0 (skip)


// A2*C1 -> A0,B0 (unsigned, unsigned) // do first
// C2*C1 -> C0,D0 (unsigned, unsigned)  // do second
// B2*C1 -> B0,C0 (unsigned, unsigned)
// D2*C1 -> D0,y0 (signed, unsigned)
// B2*A1 -> x0,A0 (unsigned, unsigned)
// C2*A1 -> A0,B0 (unsigned, unsigned)
// D2*A1 -> B0,C0 (signed, unsigned)
// A2*B1 -> x0,A0 (unsigned, unsigned) 
// B2*B1 -> A0,B0 (unsigned, unsigned)
// C2*B1   -> B0,C0 (unsigned, unsigned)
// D2*B1 -> C0,D0 (signed, unsigned)
// A2*D1 -> B0,C0 (unsigned, signed)
// B2*D1 -> C0,D0 (unsigned, signed) 
// C2*D1 -> D0,y0 (unsigned, signed)

fixedpointnum lib_fp_multiply(fixedpointnum x, fixedpointnum y)
{
    // multiplies two fixed point numbers without overflowing and returns the result
    int32_t xh = x >> FIXEDPOINTSHIFT;
    uint32_t xl = x & 0xffff;
    int32_t yh = y >> FIXEDPOINTSHIFT;
    uint32_t yl = y & 0xffff;
    return ((xh * yh) << FIXEDPOINTSHIFT) + xh * yl + yh * xl + ((xl * yl) >> FIXEDPOINTSHIFT);
}

void lib_fp_lowpassfilter(fixedpointnum *variable, fixedpointnum newvalue, fixedpointnum timesliver, fixedpointnum oneoverperiod, int timesliverextrashift)
{   // updates a low pass filter variable.  It uses timesliver.  oneoverperiod is one over the time period
    // over which the filter is averaged.  We use oneoverperiod to avoid having to use division.
    // this does the equivalent of:
    //    float fraction=timesliver/period;
    //    variable=fraction*newvalue+(1-fraction)*variable;
    // except it does it using fixed point arithmatic
    // If timesliver is very small, resolution can be gained by keeping timesliver shifted left by some extra bits.
    // Make sure variable can be also shifted left this same number of bits or else the following will overflow! 
    fixedpointnum fraction = lib_fp_multiply(timesliver, oneoverperiod);

    *variable = (lib_fp_multiply(fraction, newvalue) + lib_fp_multiply((FIXEDPOINTONE << timesliverextrashift) - fraction, *variable)) >> timesliverextrashift;

    // the adder combines adding .5 for rounding error and adding .5 in the direction the newvalue is trying to pull us
    // So we can zero in on the desired value.
    if (newvalue > *variable)
        ++ * variable;
}

fixedpointnum lib_fp_abs(fixedpointnum fp)
{
    if (fp < 0)
        return (-fp);
    else
        return (fp);
}

// we may be able to make these unsigned ints and save space
static const fixedpointnum biganglesinelookup[] = {  // every 8 degrees
    0,                          // sine(0)
    9121,                       // sine(8)
    18064,                      // sine(16)
    26656,                      // sine(24)
    34729,                      // sine(32)
    42126,
    48703,
    54332,
    58903,
    62328,
    64540,
    65496,
};

static const fixedpointnum biganglecosinelookup[] = {        // every 8 degrees
    65536,                      // cosine(0)
    64898,                      // cosine(8)
    62997,                      // cosine(16)
    59870,                      // cosine(32)
    55578,
    50203,
    43852,
    36647,
    28729,
    20252,
    11380,
    2287
};

fixedpointnum lib_fp_sine(fixedpointnum angle)
{   // returns sine of angle where angle is in degrees
    // manipulate so that we only have to work in a range from 0 to 90 degrees
    char negate = 0;

    while (angle > FIXEDPOINT180)
        angle -= FIXEDPOINT360;
    while (angle < -FIXEDPOINT180)
        angle += FIXEDPOINT360;

    if (angle < 0) {
        angle = -angle;
        negate = !negate;
    }
    if (angle > FIXEDPOINT90) {
        angle = FIXEDPOINT180 - angle;
    }
    // below works from 0 to 90 degrees
    int bigangle = angle >> (3 + FIXEDPOINTSHIFT);      // big angle is angle/8
    angle -= ((long) bigangle) << (3 + FIXEDPOINTSHIFT);        // the small angle remainder

    // sine(x+y)=cosine(x)*sine(y)+sine(x)*cosine(y);
    // for small angles, sine(x)=x where x is in radians
    // and cosine of x=1 for small angles
    fixedpointnum result = lib_fp_multiply(biganglecosinelookup[bigangle], (angle * 143) >> 13) + biganglesinelookup[bigangle];
    if (negate)
        result = -result;
    return (result);
}

fixedpointnum lib_fp_cosine(fixedpointnum angle)
{
    return (lib_fp_sine(angle + FIXEDPOINT90));
}

static const fixedpointnum atanlist[] = {
    2949120L,                   // atan(2^-i), in fixedpointnum
    1740967L,
    919879L,
    466945L,
    234379L,
    117304L,
    58666L,
    29335L,
    14668L,
    7334L,
    3667L,
    1833L,
    917L
};

// cordic arctan2 using no division!
// http://www.coranac.com/documents/arctangent/

fixedpointnum lib_fp_atan2(fixedpointnum y, fixedpointnum x)
{   // returns angle from -180 to 180 degrees
    if (y == 0)
        return (x >= 0 ? 0 : FIXEDPOINT180);

    fixedpointnum returnvalue = 0;
    fixedpointnum tmp;

    if (y < 0) {
        x = -x;
        y = -y;
        returnvalue -= FIXEDPOINT180;
    }
    if (x <= 0) {
        tmp = x;
        x = y;
        y = -tmp;
        returnvalue += FIXEDPOINT90;
    }
    if (x <= y) {
        tmp = y - x;
        x = x + y;
        y = tmp;
        returnvalue += FIXEDPOINT45;
    }
    // Scale up a bit for greater accuracy.
    if (x < 0x10000) {
        x *= 0x1000;
        y *= 0x1000;
    }

    for (int i = 1; i < 11; i++)        // increase the accuracy by increasing the number of cycles (was 12)
    {
        if (y >= 0) {
            tmp = x + (y >> i);
            y = y - (x >> i);
            x = tmp;
            returnvalue += atanlist[i];
        } else {
            tmp = x - (y >> i);
            y = y + (x >> i);
            x = tmp;
            returnvalue -= atanlist[i];
        }
    }
    return (returnvalue);
}

fixedpointnum lib_fp_sqrt(fixedpointnum x)
{
    return (lib_fp_multiply(x, lib_fp_invsqrt(x)));
}

int32_t lib_fp_stringtolong(char *string)
{   // converts a string to an integer.  Stops at any character other than 0-9 or '-'.  Stops at decimal places too.
    long value = 0;
    char negative = 0;
    if (*string == '-') {
        negative = 1;
        ++string;
    }
    while (*string >= '0' && *string <= '9') {
        value = value * 10 + *string++ - '0';
    }
    if (negative)
        return (-value);
    else
        return (value);
}

fixedpointnum lib_fp_stringtofixedpointnum(char *string)
{
    char negative = 0;
    if (*string == '-') {
        negative = 1;
        ++string;
    }

    fixedpointnum value = (long) lib_fp_stringtolong(string) << FIXEDPOINTSHIFT;

    // find the decimal place
    while (*string != '\0' && *string != '.')
        ++string;
    if (*string == '\0')
        return (value);

    // use six digits after the decimal
    ++string;
    int fractionallength = 0;
    while (string[fractionallength] >= '0' && string[fractionallength] <= '9')
        ++fractionallength;

    if (fractionallength > 6)
        string[6] = '\0';
    fixedpointnum fractional = lib_fp_stringtolong(string);
    while (fractionallength++ < 6) {
        fractional *= 10;
    }
    value = value + (lib_fp_multiply(fractional, 137439L) >> 5);        // 137439L is .000001 * 2^16 * 2^16 * 2^5

    if (negative)
        return (-value);
    else
        return (value);
}

static const fixedpointnum invsqrtstartingpoint[12] = {      // =(1/((x+4.5)/16)^0.5)
    // add 4 because range of xysquared starts at .25.
    // add .5 to get a value in the middle of the range.
    // 16 is the number of steps we break down into (hence the shift by 4 when calling)
    123575L,
    111778L,
    102821L,
    95721L,
    89914L,
    85050L,
    80899L,
    77302L,
    74145L,
    71346L,
    68842L,
    66584L
};

fixedpointnum lib_fp_invsqrt(fixedpointnum x)
{
    if (x <= 0)
        return (0);

    fixedpointnum y = FIXEDPOINTONE;
    fixedpointnum xysquared = x;

    // find the starting point
    while (xysquared < FIXEDPOINTONEOVERFOUR) {
        y = y << 1;
        xysquared = xysquared << 2;
    }
    while (xysquared >= FIXEDPOINTONE) {
        y = y >> 1;
        xysquared = xysquared >> 2;
    }

    // at this point .25 <= xysquared < 1
    // the idea is to get xysquared as close to 1 as possible.  Start by using
    // the lookup table.

    y = lib_fp_multiply(y, invsqrtstartingpoint[(xysquared >> (FIXEDPOINTSHIFT - 4)) - 4]);

    // now use newton's method to increase accuracy
    // increase iterations if you want more accuracy
    // this one iteration should get us to within .5% error
    xysquared = lib_fp_multiply(y, lib_fp_multiply(x, y));
    y = lib_fp_multiply(y, (FIXEDPOINTTHREE - xysquared)) >> 1;
//   xysquared=lib_fp_multiply(y,lib_fp_multiply(x,y));
//   y = lib_fp_multiply(y,(FIXEDPOINTTHREE - xysquared))>>1;

    return (y);
}
