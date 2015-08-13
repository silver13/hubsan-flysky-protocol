/*
 * Copyright 2014 TheLastMutt
 * This file is free. It comes without any warranty, to the extent permitted by applicable law.
 * You can redistribute it and/or modify it under the terms of the
 * Do What The Fuck You Want To Public License, Version 2, as published by Sam Hocevar.
 * See http://www.wtfpl.net/ for more details.
*/

#include "hal.h"
#include "lib_fp.h"
#include "lib_adc.h"
#include "config.h"

// ADC reference voltage defined in config_*.h as fixedpointnum
#define FP_ADC_REF_VOLTAGE FIXEDPOINTCONSTANT(ADC_REF_VOLTAGE)

void lib_adc_init(void) {
    CLK_EnableModuleClock(ADC_MODULE);

    // 22MHz / 75 = 293kHz clock for ADC module
    // -> approx 137µs conversion time.
    CLK_SetModuleClock(ADC_MODULE,
                       CLK_CLKSEL1_ADC_S_IRC22M,
                       CLK_CLKDIV_ADC(50));
    ADC_POWER_ON(ADC);
} // lib_adc_init()

void lib_adc_select_channel(lib_adc_channel_t channel) {
    // While changing channel, ADST bit must be cleared
    ADC_STOP_CONV(ADC);

    if (channel == LIB_ADC_CHANREF) {
        // Needs special treatment, shared with input 7
        ADC_CONFIG_CH7(ADC, ADC_CH7_BGP);
        channel = LIB_ADC_CHAN7;
    } else if (channel == LIB_ADC_CHAN7) {
        ADC_CONFIG_CH7(ADC, ADC_CH7_EXT);
    }

    ADC_SET_INPUT_CHANNEL(ADC,channel);
} // lib_adc_select_channel()

bool lib_adc_is_busy(void) {
    return ADC_IS_BUSY(ADC);
}

void lib_adc_startconv(void) {
    ADC_START_CONV(ADC);
}

// Returns measured absolute voltage as fixedpointnum
fixedpointnum lib_adc_read_volt(void) {
    fixedpointnum voltage;
    // ADC_GET_CONVERSION_DATA() returns uint32, only lower 10 bits used
    voltage = (fixedpointnum) ADC_GET_CONVERSION_DATA(ADC, 0);
    // Now shift left to get a fixedpointnum, which then is a fraction
    // relative to ADC reference voltage
    voltage = voltage << (FIXEDPOINTSHIFT - 10);
    // Now multiply by supply voltage
    voltage = lib_fp_multiply(voltage, FP_ADC_REF_VOLTAGE);
    return voltage;
} // lib_adc_read_volt()

// Returns ADC result as fixedpointnum between 0..1
fixedpointnum lib_adc_read_raw(void) {
    fixedpointnum voltage;
    // ADC_GET_CONVERSION_DATA() returns uint32, only lower 10 bits used
    voltage = (fixedpointnum) ADC_GET_CONVERSION_DATA(ADC, 0);
    // Now shift left to get a fixedpointnum, which then is a fraction
    // relative to ADC reference voltage
    voltage = voltage << (FIXEDPOINTSHIFT - 10);
    return voltage;
} // lib_adc_read_volt()
