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

#pragma once

#include "projectsettings.h"

#define DIGITALINPUT 0
#define DIGITALOUTPUT 1

#define DIGITALPORTA 0x10
#define DIGITALPORTB 0x20
#define DIGITALPORTC 0x30

#define DIGITALPORT0 0x00
#define DIGITALPORT1 0x10
#define DIGITALPORT2 0x20
#define DIGITALPORT3 0x30
#define DIGITALPORT4 0x40
#define DIGITALPORT5 0x50

#define DIGITALHIGH 0
#define DIGITALLOW 1

#define DIGITALON 1
#define DIGITALOFF 0

void lib_digitalio_initpin(unsigned char portandpinnumber, unsigned char output);
unsigned char lib_digitalio_getinput(unsigned char portandpinnumber);
void lib_digitalio_setoutput(unsigned char portandpinnumber,unsigned char value);

typedef void (* digitalcallbackfunctptr)(unsigned char interruptnumber, unsigned char newpinstate);
void lib_digitalio_setinterruptcallback(unsigned char portandpinnumber, digitalcallbackfunctptr callback);
