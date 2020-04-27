/*
 *  main.c
 *
 *  Created on: Sep 21, 2016
 *      Author: dauz
 */


#include <stdio.h>
#include "MLC_drv.h"
#include "modulator.h"

#include "adc_config.h"
#include "MotorControlAlgorithm_0.h"

float EncUnitTimerPeriod;


//uint32_t f_pwm = 10000; // eg f = 10kHz. (this is also the sampling frequency because the ADC is sync with PWM)

int main(){


	// Basic init of PLL etc.
	InitSysCtrl();

    // Initialization of evaluation board. All GPIO, peripheral, etc used are configurated by this function.
    MLC_init();
    MLC_global_enable();

    // Enabling of the writing of protected registers (this writing permission ends with EDIS)
    EALLOW;
    // This is for start of conv pin
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 3;

	// Disable CPU interrupts
	DINT;
	// Disable Real-Time interrupts
	DRTM;

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the DSP2833x_PieCtrl.c file.
	InitPieCtrl();

	// Clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
	// This function is found in DSP2833x_PieVect.c.
	InitPieVectTable();

    // Init of Encoder (EALLOW and EDIS in function)
    EncoderUnitTimerBaseSetup(ENC_UNIT_TIMER_PERIOD);

	// Enable the write access to protected registers (EDIS disable the write access)
	EALLOW;

	// Disable Interrupt timer
	EPwm1Regs.ETSEL.bit.INTEN = 0;

	// ADC configuration
	adc_config();
	// ADC - SOC trigger
	adc_trigger_SOC();

	// Call Init routine
	init_modulator(F_PWM, &EPwm1Regs);
	init_modulator(F_PWM, &EPwm2Regs);
	init_modulator(F_PWM, &EPwm3Regs);

	// Enable generation of interrupt
	EPwm1Regs.ETSEL.bit.INTEN = 1;
	EPwm2Regs.ETSEL.bit.INTEN = 1;
	EPwm3Regs.ETSEL.bit.INTEN = 1;
	// Selecting source of interrupt. In this case the source is the CTR = Zero (ref. pag 31 of slideshow)
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
	EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
	EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
	// Selecting period of interrupt. In this case the interrupt is generated at every period (so, ad every reaching of zero) (ref. page 33 of slideshow)
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;
	EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;
	EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;

	// Enabling of IER according with interrupt-crossbars (ref. pag 19) and routing-table of interrupt (ref. pag 20).
	// In this case I'm using the interrupt of ePWM1 (ePWM1_INT). This Interrupt is on the 3th row (-> INT3)  and 8th column (-> INTx.1)
	IER |= M_INT3;                      // --> this is the enabling of the 3th row
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // --> this is the enabling of the 8th column
	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  // --> this is the enabling of the 7th column
	PieCtrlRegs.PIEIER3.bit.INTx3 = 1;  // --> this is the enabling of the 6th column
	// NOTE: IER |= somethig and PieCtrlRegs.PIEIERsomething are always together!!!

	// Enable global interrupt
	//EINT;

	EALLOW;
	// This is the assignment of the my interrupt routine to the PIE Vector Table. It's a linking between the ePWM1_Interrupt and my routine.
	PieVectTable.EPWM1_INT = &isr_epwm1;
	PieVectTable.EPWM2_INT = &isr_epwm2;
	PieVectTable.EPWM3_INT = &isr_epwm3;

	// Lock the writing access to protected registers
	EDIS;

	// Enabling Global interrupt INTM (EINT) and global real-time interrupt (ERTM)
	EINT;
	ERTM;

	MLC_DAC_init();
	MLC_DAC_reset();

	while(1){

	}
}

