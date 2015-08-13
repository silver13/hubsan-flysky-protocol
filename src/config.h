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

// this is the file that is used to configure the software.  Uncomment the appropriate lines by removing the // in front of them.
// Configuration works with a lot of defaults.  The only thing you really need to choose is the control board.  After that,
// all defaults will be chosen for that board.  If you want to use other than defaults, then uncomment the things you want
// to change.  To see what the defaults are, look in defs.h
// The options are defined in options.h

#if defined(V202_BUILD)
#include "config_V202.h"
#elif defined(JD385_BUILD)
#include "config_JD385.h"
#elif defined(X4_BUILD)
#include "config_X4.h"
#else
#include "config_STM32.h"
#endif
