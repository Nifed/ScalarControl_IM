/*
 * modulator.c
 *
 *  Created on:
 *      Author:
 */

#include "MLC_drv.h"
#include "DSP2833x_ePWM.h"
uint32_t period;

//uint32_t f_pwm = 10000;

void init_modulator(uint32_t f_pwm, volatile struct EPWM_REGS* x_ePWM){
    EALLOW;
    // Set of HighSpeed clk divisor to zero.
    x_ePWM->TBCTL.bit.HSPCLKDIV = 0;
    // Set of clk divisor to zero.
    x_ePWM->TBCTL.bit.CLKDIV = 0;

    // Overflow avoiding
    period = (150e6/f_pwm)/4; //divided by 4 because it's setted up and down and there is a toggling of led
    if(period >= 65535){
        period = 65535;
    }

    // Set of the period
    x_ePWM->TBPRD = period;
    // Set of the counting mode.
    // In this case UP-DOWN means that:
        // 1) The counter starts to count UP
        // 2) When the counter reaches the value stored in TBPRD, it change the counting direction and start to decrease the counter.
        // 3) When the counter reaches zero an interrupt is generated (according with the setup) and the direction in toggled again.
        // 4) Point 1) again
    x_ePWM->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    // To be sure the counting starts from zero (so, about the first period) it's good to clear the counter register (TBCTR)
    x_ePWM->TBCTR = 0x0;

    // Enable shadow mode
    x_ePWM->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    // Set update compare values on top and bottom
    x_ePWM->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    // Enable shadow mode
    x_ePWM->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    // Set update compare values on top and bottom
    x_ePWM->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
    // Set startup value for compare regs to achieve 50/50 dc
    x_ePWM->CMPA.half.CMPA = period/2;
    x_ePWM->CMPB = period/2;

    // Init of AQ submodule
    // Set out mode of AQ
    x_ePWM->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    // Set polarity for A and B (complementary)
    x_ePWM->DBCTL.bit.POLSEL = DB_ACTV_HIC;
    // Set input mode of AQ for A and B
    x_ePWM->DBCTL.bit.IN_MODE = DBA_ALL;
    x_ePWM->DBCTL.bit.IN_MODE = DBB_ALL;
    // Set delay (clk = 150 MHz --> T = 6.67e-9 --> 1 us = 149.92 counts --> 150 counts)
    x_ePWM->DBFED = 150;
    x_ePWM->DBRED = 150;
    // Set A and B
    x_ePWM->AQCTLA.bit.CAU = AQ_SET;
    x_ePWM->AQCTLA.bit.CAD = AQ_CLEAR;
    x_ePWM->AQCTLB.bit.CAU = AQ_CLEAR;
    x_ePWM->AQCTLB.bit.CAD = AQ_SET;

    EDIS;
}

uint16_t global_compare_A;
uint16_t global_compare_B;


interrupt void isr_epwm1(void){
    //GpioDataRegs.GPATOGGLE.bit.GPIO26 = 1;
    // Crear flag about interrupt of Group 3 (it means all the 3th column of the table in page 20)
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    // Clear flag about trigger ePWM1
    EPwm1Regs.ETCLR.bit.INT = 1;
    // Update cmoparators
    EPwm1Regs.CMPA.half.CMPA = global_compare_A;
    EPwm1Regs.CMPB = global_compare_B;

}

interrupt void isr_epwm2(void){
    //GpioDataRegs.GPATOGGLE.bit.GPIO26 = 1;
    // Crear flag about interrupt of Group 3 (it means all the 3th column of the table in page 20)
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    // Clear flag about trigger ePWM1
    EPwm2Regs.ETCLR.bit.INT = 1;
    // Update cmoparators
    EPwm2Regs.CMPA.half.CMPA = global_compare_A;
    EPwm2Regs.CMPB = global_compare_B;
}

interrupt void isr_epwm3(void){
    //GpioDataRegs.GPATOGGLE.bit.GPIO26 = 1;
    // Crear flag about interrupt of Group 3 (it means all the 3th column of the table in page 20)
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    // Clear flag about trigger ePWM1
    EPwm3Regs.ETCLR.bit.INT = 1;
    // Update cmoparators
    EPwm3Regs.CMPA.half.CMPA = global_compare_A;
    EPwm3Regs.CMPB = global_compare_B;
}

