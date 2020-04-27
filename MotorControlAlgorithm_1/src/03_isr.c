/*
 * 03_isr.c
 *
 *  Created on: Oct 5, 2016
 *      Author: dauz
 */

#include <modulator.h>
#include "MLC_drv.h"

interrupt void isr_fce(void)
{
	 CpuTimer0.InterruptCount++;
	 //GpioDataRegs.GPATOGGLE.bit.GPIO26 = 1; // Toggle GPIO32 once per 500 milliseconds
	 // Acknowledge this interrupt to receive more interrupts from group 1
	 PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



interrupt void myisr_tim2_routine(void){
    CpuTimer2.InterruptCount++;
    //GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
