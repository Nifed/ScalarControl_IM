/*
 * modulator.h
 *
 *  Created on: Oct 5, 2016
 *      Author: dauz
 */


#include "MLC_drv.h"
#include "DSP2833x_EPwm.h"

#ifndef INCLUDE_MODULATOR_H_
#define INCLUDE_MODULATOR_H_

extern uint32_t f_pwm;

void init_modulator(uint32_t f_pwm, struct EPWM_REGS* x_ePWM);
interrupt void isr_epwm1(void);
interrupt void isr_epwm2(void);
interrupt void isr_epwm3(void);

#endif /* INCLUDE_MODULATOR_H_ */
