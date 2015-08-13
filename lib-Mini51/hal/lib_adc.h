/*
 * Copyright 2014 TheLastMutt
 * This file is free. It comes without any warranty, to the extent permitted by applicable law.
 * You can redistribute it and/or modify it under the terms of the
 * Do What The Fuck You Want To Public License, Version 2, as published by Sam Hocevar.
 * See http://www.wtfpl.net/ for more details.
*/

#ifndef LIB_ADC_H_
#define LIB_ADC_H_

// Parameter type for lib_adc_select_channel
typedef enum lib_adc_channel_tag
{
	LIB_ADC_CHAN0 = (1 << 0), // Pin AIN0
	LIB_ADC_CHAN1 = (1 << 1), // ..
	LIB_ADC_CHAN2 = (1 << 2),
	LIB_ADC_CHAN3 = (1 << 3),
	LIB_ADC_CHAN4 = (1 << 4),
	LIB_ADC_CHAN5 = (1 << 5),
	LIB_ADC_CHAN6 = (1 << 6),
	LIB_ADC_CHAN7 = (1 << 7), // Pin AIN7
	LIB_ADC_CHANREF = 0xFF    // Internal bandgap reference, needs ADC Clock < 300kHz
} lib_adc_channel_t;

void lib_adc_init(void);
void lib_adc_select_channel(lib_adc_channel_t channel);
bool lib_adc_is_busy(void);
void lib_adc_startconv(void);
// Returns absolute voltage
fixedpointnum lib_adc_read_volt(void);
// Returns 0..1
fixedpointnum lib_adc_read_raw(void);


#endif /* LIB_ADC_H_ */
