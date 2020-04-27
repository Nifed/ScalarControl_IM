/*
 * modulator.h
 *
 *  Created on: Oct 5, 2016
 *      Author: dauz
 */


#include "MLC_drv.h"
#include "DSP2833x_EPwm.h"

#ifndef INCLUDE_ADC_CONFIG_H_
#define INCLUDE_ADC_CONFIG_H_



void adc_config(void);
interrupt void adc_conversion_routine(void);
interrupt void dma_ch3_interrupt(void);
void adc_trigger_SOC(void);
float UdcCalculation(void);

#endif /* INCLUDE_MODULATOR_H_ */
