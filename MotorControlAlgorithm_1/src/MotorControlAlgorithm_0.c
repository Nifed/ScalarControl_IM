/*
 * modulator.c
 *
 *  Created on:
 *      Author:
 */

#include "MLC_drv.h"
#include "DSP2833x_ePWM.h"
#include "DSP2833x_EQep.h"
#include "MotorControlAlgorithm_0.h"
#include "qmath.h"
#include "modulator.h"
#include "math.h"
#include "adc_config.h"


volatile int16_t Ua = 0;

#define TBPeriod_a EPwm1Regs.TBPRD
#define TBPeriod_b EPwm2Regs.TBPRD
#define TBPeriod_c EPwm3Regs.TBPRD


#define BETA_REF M_PI
#define F_REF 100
#define F_SAMP 20e3
#define DELTA_T_SAMP 1/F_SAMP
#define K_BETA  ((2*M_PI*F_REF*DELTA_T_SAMP)/(BETA_REF))
#define Q015_RESOLUTION 3.051757e-5
#define I_NOMINAL 3.65
#define R_PHASE 1.95//3.68
#define U_MIN (I_NOMINAL*R_PHASE*sqrt(2))
#define FR_MAX 5
#define UM_REF 60
#define U_MAX 48


#define IRC 1024

#define PERIOD_SYSCLKOUT 6.67e-9

volatile int32_t a = 0;
volatile int32_t b = 0;
volatile int32_t c = 0;
volatile uint16_t comp_value_a, comp_value_b, comp_value_c;
volatile float MeasuredFmotor = 0;
static int32_t DeltaCNT;
volatile uint16_t MotorDir;

volatile int16_t Ua_fp = 0;
volatile int16_t Ub_fp = 0;
volatile int16_t Uc_fp = 0;

int16_t fs_target_q015, f_q015, fr_error_q015;
float f_err_perc;

float Udc = 20;
//float fref = 100;
float BETA;
float PIDdt = 1.0/(2*F_PWM);
float DeltaU = 0;
float Uint = 0;
float Umodulator = 0;
static float fr_pi;
int16_t udc_q015;


static uint16_t toggle = 0;
interrupt void EncoderInterruptRoutine(void)
{
    // Clear interrupt flags
    EQep1Regs.QCLR.all = 0xffff;
    // Clear PIE flags
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1;
    MotorDir = (uint16_t)EQep1Regs.QEPSTS.bit.QDF;
    // Calculating of real motor speed fmot = DCNT/(4*IRC*Period)
    DeltaCNT = (0xFFFFFFFF - EQep1Regs.QPOSLAT);
    MeasuredFmotor = -DeltaCNT/(4*IRC*ENC_UNIT_TIMER_PERIOD);

        if(toggle == 0)
        {
            toggle = 1;
            dac_values[2] = 4095;
        }else
        {
            toggle = 0;
            dac_values[2] = 0;
        }

}





void EncoderUnitTimerBaseSetup(float period) // Use period in ns
{
    float EQEPTimer_Period = 0;
    EQEPTimer_Period = period/PERIOD_SYSCLKOUT;
    EALLOW;
    // Enable PIE
    IER |= M_INT5;
    PieCtrlRegs.PIEIER5.bit.INTx1 = 1;
    // Enable Interrupt
    EQep1Regs.QEINT.bit.UTO = 1;
    // Link the interrupt routine
    PieVectTable.EQEP1_INT = (PINT)EncoderInterruptRoutine;
    // Reset of the time base for the eQEP unit timer
    EQep1Regs.QUTMR = 0;
    // Set of the period of eQEP unit timer (interrupt when QUTMR matches QUPRD)
    EQep1Regs.QUPRD = (uint32_t)EQEPTimer_Period;
    // Setting of the mode (Position Counter Reset on Unit Time out Event)
    EQep1Regs.QEPCTL.bit.PCRM = 0b11;
    // Enabling of QCLM to store position counter in QPOSLAT
    EQep1Regs.QEPCTL.bit.QCLM = 1;
    //
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;
    EQep1Regs.QEPCTL.bit.QPEN = 1;
    // Enable eQEP unit timer
    EQep1Regs.QEPCTL.bit.UTE = 1;
    EDIS;
}

#define N_POLES_PAIRS 2
float DeltaU;
uint16_t FrtoU_Method = 0;
float Fr_to_DeltaU(float fr)
{
    float DeltaU = 0;
    //F_rotor = f_stator - (Npolespairs*Fmechanical)
    DeltaU = fabsf(fr/FR_MAX)*U_MIN;


  /*
    // Method 1 - Shifting of the courve Um/fr up of DeltaU
    if(FrtoU_Method == 0)
    {
        Um = abs(U_MAX*f/FR_MAX) + DeltaU;
    }
    // Method 2 - Limitation of the courve Um/fr using dinamic Umin
    else
    {
        Um = abs(U_MAX*(f)/FR_MAX);
        float Umin = 0;
        Umin = U_MIN - DeltaU;
        if(Um < Umin)
        {
            Um = Umin;
        }
    }*/

    return DeltaU;
}


float Fs_to_Uind(float fs) // (referring to Kfr)
{
    float Uind = 0;
    Uind = fabsf(U_MAX*fs/F_MAX);
    return Uind*sqrt(2);
}

int16 MotorControlAlgorithm_0_q015(float Um, float fs, float Udc)
{

    volatile static int16_t beta_q015 = 0;
    int16_t ua_q015, ub_q015, uc_q015, um_q015, phase_shift_q015;
    int16_t FS;
    //int16_t kbeta = K_BETA*(1U<<15);


    // Check of the maximum value of the stator frequency
    if(fs > F_REF)
    {
        fs = F_REF; // The resolution of Q0.15 is 3.0517578125e-5
    }
    // Normalization in Q0.15
    FS = (fs/F_REF)*(1U<<15);
    // Beta implementation in Q0.15 (either K_BETA or FS are now in Q0.15)
    //beta_q015 = beta_q015 + (((int32_t)(kbeta*FS))>>15); // Remember to force by casting the 32bts usage and after that convert it back to Q0.15 (look at theory of fixed number arithmetic)

    // I think it's in this way:
    //beta_q015 = beta_q015 + ((int32_t)(K_BETA*fs/F_REF))*(1U<<15); // ------------->>> I THINK SHOULD BE IN THIS WAY. ASK THIS!!
    // I have: a + b where  a = q15
    //                      b = q15 because of the shifting with (1U<<15)
    // Before shifting I have in b: int32 * int32 / int32 --> int32
    a = K_BETA*fs*(1U<<15);
    b = a/F_REF;
    c = b;
    beta_q015 += c;

    // phase shift = 2pi/3 and normalization constant = PI --> 2/3 and shifting -> Q0.15
    phase_shift_q015 = (2.0/3.0)*(1<<15); // writing 2.0 the calculation is made in float

    // Normalization and shift -> Q0.15
    um_q015 = (Um/UM_REF)*(1U<<15);

    // Calculating U phase A in Q0.15
    ua_q015 = ((int32_t)um_q015*qsin(beta_q015))>>15;
    // Calculating U phase B in Q0.15
    ub_q015 = ((int32_t)um_q015*qsin(beta_q015 + phase_shift_q015))>>15;
    // Calculating U phase C in Q0.15
    uc_q015 = ((int32_t)um_q015*qsin(beta_q015 - phase_shift_q015))>>15;

    // Writing ua in DAC ch 2
    //dac_values[2] = 2048+(ua_q015>>4);// ua_q015>>5;
    // Writing beta in Q0.15 to DAC
    //dac_values[1] = beta_q015>>4;


    // Normalization and shift of Udc --> Q015
    udc_q015 = (Udc/UM_REF)*(1U<<15);

    // ---------------------------- A phase check --------------------------------
    if(ua_q015 > udc_q015/2)
    {
        comp_value_a = 0xFFFF;
    }
    else if(ua_q015 < -udc_q015/2)
    {
        comp_value_a = 0x0000;
    }               //                      |----------- Q15.0 ------------------------|
    else            //                      |----------- Q15.15 -------------------|
    {               // Q15.0                Q15.0              |------ Q0.15-------|
        comp_value_a = TBPeriod_a/2 + ((int32_t)TBPeriod_a*(((int32_t)ua_q015<<15)/udc_q015)>>15);   // Remember to shift the numerator BEFORE the division otherwise the result could be zero
    }
    // Writing comp value for PWM
    EPwm1Regs.CMPA.half.CMPA = (uint16_t)comp_value_a;
    // Writing on DAC
    MLC_DAC_write(8);
    dac_values[3] = 2048 + ((comp_value_a - TBPeriod_a/2)>>4);

    // ---------------------------- B phase check --------------------------------
    if(ub_q015 > udc_q015/2)
    {
        comp_value_b = 0xFFFF;
    }
    else if(ub_q015 < -udc_q015/2)
    {
        comp_value_b = 0x0000;
    }               //                      |----------- Q15.0 ------------------------|
    else            //                      |----------- Q15.15 -------------------|
    {               // Q15.0                Q15.0              |------ Q0.15-------|
        comp_value_b = TBPeriod_b/2 + ((int32_t)TBPeriod_b*(((int32_t)ub_q015<<15)/udc_q015)>>15);   // Remember to shift the numerator BEFORE the division otherwise the result could be zero
    }
    // Writing comp value for PWM
    EPwm2Regs.CMPA.half.CMPA = (uint16_t)comp_value_b;

    // ---------------------------- C phase check --------------------------------
    if(uc_q015 > udc_q015/2)
    {
        comp_value_c = 0xFFFF;
    }
    else if(uc_q015 < -udc_q015/2)
    {
        comp_value_c = 0x0000;
    }               //                      |----------- Q15.0 ------------------------|
    else            //                      |----------- Q15.15 -------------------|
    {               // Q15.0                Q15.0              |------ Q0.15-------|
        comp_value_c = TBPeriod_c/2 + ((int32_t)TBPeriod_c*(((int32_t)uc_q015<<15)/udc_q015)>>15);   // Remember to shift the numerator BEFORE the division otherwise the result could be zero
    }
    // Writing comp value for PWM
    EPwm3Regs.CMPA.half.CMPA = (uint16_t)comp_value_c;



    return beta_q015; // return beta in Q0.15

    // It is possiible to implement this algo. in 32 bits to improve the precision. At the end of the algo I can shift (on right) the 32bit
    // beta to 15bits to convert back in Q0.15. TRY TO IMPLEMENT IT
}




//#define PID_e (fs_setpoint - MeasuredFmotor) // MeasuredFmotor = rotor freq from encoder

float KP = 0.8;
float KI = 0.15;
float KD = 0;
float tau = 0.02;
static float SumOfIPart = 0;
float PID_error;
uint16_t PID_Flag = 0;

float PIDController(float fs_setpoint, float dt)
{
    float fr_pid;
    // err = fs_setpoint - fsmechanical(electrical)
    PID_error = (fs_setpoint - MeasuredFmotor*N_POLES_PAIRS);
    //fr_pid = KP*PID_error + (KP/tau)*PID_error*dt + KD*PID_error/dt;
    fr_pid = KP*PID_error + SumOfIPart + KD*PID_error/dt;

    // Limitation of Fr
    if(fr_pid < -FR_MAX)
    {
        fr_pid = -FR_MAX;
        PID_Flag = 1;
    }
    else if(fr_pid > FR_MAX)
    {
        fr_pid = FR_MAX;
        PID_Flag = 1;
    }
    else
    {
        PID_Flag = 0;
    }

    if(PID_Flag == 0)
    {
        SumOfIPart += (KP/tau)*PID_error*dt;
        //SumOfIPart += KI*PID_error*dt;
    }
    return fr_pid;
}


float inline UswLimiter(float U)
{
    if(U > UM_REF)
        U = UM_REF;
    if(U < -UM_REF)
        U = -UM_REF;
    return U;
}


void MotorControlAlg_PID(float fs_target)
{
    // PID controller
    fr_pi = PIDController(fs_target, PIDdt);
    // Calculation of Um according to fs (referring to Ku)
    //Um = Fs_to_U(fr_pi + MeasuredFmotor*N_POLES_PAIRS);
    DeltaU = Fr_to_DeltaU(fr_pi);
    // Calculation of Uind (referring to Kfr)
    Uint = Fs_to_Uind(fr_pi + N_POLES_PAIRS*MeasuredFmotor);
    // Calculation of Udc
    Udc = UdcCalculation();
    // Sum of DeltaU and Uint and limitation of this value
    Umodulator = UswLimiter(DeltaU + Uint);
    // Using Motor_Control_Algorithm_0 in fixed point
    BETA = MotorControlAlgorithm_0_q015(Umodulator, fr_pi + N_POLES_PAIRS*MeasuredFmotor, Udc);
    // Writing fs_target and fs_current on DAC (of values in q015 format)

    // NOTE: F_rotor = f_stator - (Npolespairs*Fmechanical)
    // Conversion of speed data for display them on the oscilloscope
    fs_target_q015 = (fs_target/F_REF)*(1U<<15);
    f_q015 = ((N_POLES_PAIRS*MeasuredFmotor)/F_REF)*(1U<<15);
    dac_values[0] = ((fs_target_q015)>>4) + 2048;
    dac_values[1] = ((f_q015)>>4) + 2048;
    MLC_DAC_write(2);

}



