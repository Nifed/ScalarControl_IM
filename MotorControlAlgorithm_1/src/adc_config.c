/*
 * adc_config.c
 *
 *  Created on:
 *      Author:
 */

#include "MLC_drv.h"
#include "DSP2833x_ePWM.h"
#include "adc_config.h"
#include "MotorControlAlgorithm_0.h"

volatile int16_t* adc_conversion;
volatile float MeasuredUdc;
float fs = 0;
#define UDC_ADC_CONSTANT 2.95229e-3

void adc_config(void){
    // ADC reset
    MLC_ADC_reset();
    // Link to adc routine
    MLC_ADC_enable_isr(&adc_conversion_routine);
    // ADC setup
    MLC_ADC_setup(AD_OS0, AD_OS0, AD_OS0, AD_RNG_5V, AD_RNG_5V, AD_RNG_5V);
    // SOC setup
    MLC_ADC_setup_SOC(AD_SOC_ALL_SOCA);
    // DMA eneabling and link with routine
    MLC_DMA_enable_isr(AD_CH3, dma_ch3_interrupt);
    // Enabling of DMA
    MLC_DMA_activate();
    // Address of ADC conversion
    adc_conversion = MLC_ADC3_get_res_ptr();
}


void adc_trigger_SOC(void){
    // Trigger SOC on top and bottom of value
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = 1;
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;
}

volatile int16_t * ADC_conv;

float UdcCalculation(void)
{
    ADC_conv = MLC_ADC1_get_res_ptr();
    MeasuredUdc = UDC_ADC_CONSTANT*abs(ADC_conv[4]); //[8]
    return MeasuredUdc;
}


interrupt void adc_conversion_routine(void){

    // Do something BUT remember, this function is called when ADC finished to convert data but data have to be transfered
    // (by DMA) from the exrernal ADC to the MCU's SRAM.
    // Remeber that I have to ack this IT (this is an external IT provided by ADC to one pin of the MCU)
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


interrupt void dma_ch3_interrupt(void){
    // IT ack (REMEMBER THIS!!!)
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

    // Impementation of the control scheme
    MotorControlAlg_PID(fs);
}


