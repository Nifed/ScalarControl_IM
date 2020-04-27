/*
 * modulator.h
 *
 *  Created on: Oct 5, 2016
 *      Author: dauz
 */


#include "MLC_drv.h"
#include "DSP2833x_EPwm.h"

#ifndef INCLUDE_MOTORCONTROLALGOTITHM_H_
#define INCLUDE_MOTORCONTROLALGOTITHM_H_
#define F_MAX 50
#define ENC_UNIT_TIMER_PERIOD 0.3e-2
#define F_PWM 10000

typedef struct PID
{
    float Kp;
    float Ki;
    float Kd;
}PIDCoeff_t;

extern PIDCoeff_t PIDCoeff;



int16_t MotorControlAlgorithm_0_q015(float Um_fp, float fs, float Udc_fp);
int16_t MotorControlAlgorithm_0_fp(float Um_fp, float fs_fp, float Udc_fp, float fsampling_fp);
float PIDController(float fs_setpoint, float dt);
//float Fs_to_U(float fr);
float Fr_to_DeltaU(float fr);
float Fs_to_Uind(float fr);
float inline UswLimiter(float U);
void MotorControlAlg_PID(float fs_target);
float UdcCalculation(void);
void EncoderUnitTimerBaseSetup(float period);
#endif /* INCLUDE_MODULATOR_H_ */
