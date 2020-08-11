/*
 * File: LADRC_for_code_gen.c
 *
 * Code generated for Simulink model 'LADRC_for_code_gen'.
 *
 * Model version                  : 1.19
 * Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
 * C/C++ source code generated on : Wed Jan 15 11:31:44 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "LADRC_for_code_gen.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void LADRC_for_code_gen_step(void)
{
  real_T rtb_u0k;
  real_T rtb_Sum2_o;
  real_T rtb_Sum3;
  real_T rtb_ek;
  real_T rtb_Sum5;
  real_T rtb_Sum8;
  real_T rtb_Saturation;
  real_T rtb_Sum8_tmp;

  /* Delay: '<S2>/Delay' incorporates:
   *  Constant: '<S2>/Constant'
   *  Gain: '<S2>/Gain1'
   *  Inport: '<Root>/u0'
   *  Sum: '<S2>/Sum'
   */
  if (rtDW.icLoad != 0) {
    /* Sum: '<S2>/Sum' incorporates:
     *  Constant: '<S2>/Constant'
     *  Gain: '<S2>/Gain1'
     */
    rtb_Sum3 = T * rtP.wc_bar;
    rtDW.Delay_DSTATE = (rtb_Sum3 + 1.0) * rtP.y_init - rtb_Sum3 * rtU.u0;
  }

  /* Sum: '<S2>/Sum3' incorporates:
   *  Delay: '<S2>/Delay'
   *  Gain: '<S2>/Gain3'
   *  Gain: '<S2>/Gain4'
   *  Inport: '<Root>/u0'
   */
  rtb_u0k = T * rtP.wc_bar / (T * rtP.wc_bar + 1.0) * rtU.u0 + 1.0 / (T *
    rtP.wc_bar + 1.0) * rtDW.Delay_DSTATE;

  /* Outport: '<Root>/Output2' */
  rtY.uk_m = rtb_u0k;

  /* Sum: '<S1>/Sum2' incorporates:
   *  Delay: '<S1>/Delay1'
   *  Delay: '<S1>/Delay2'
   *  Delay: '<S1>/Delay3'
   *  Gain: '<S1>/H_12_1'
   *  Gain: '<S1>/H_13_1'
   *  Inport: '<Root>/y'
   *  Sum: '<S1>/Sum1'
   */
  rtb_Sum2_o = rtU.yk - ((0.0 * rtDW.Delay2_DSTATE + rtDW.Delay1_DSTATE) + 0.0 *
    rtDW.Delay3_DSTATE);

  /* Sum: '<S1>/Sum3' incorporates:
   *  Delay: '<S1>/Delay1'
   *  Gain: '<S1>/Lc_11'
   */
  rtb_Sum3 = (1.0 - rtP.Beta * rtP.Beta * rtP.Beta) * rtb_Sum2_o +
    rtDW.Delay1_DSTATE;

  /* Sum: '<Root>/Sum1' */
  rtb_ek = rtb_u0k - rtb_Sum3;

  /* Gain: '<S1>/Lc_21' incorporates:
   *  Gain: '<S1>/Lc_31'
   */
  rtb_Sum8 = (1.0 - rtP.Beta) * (1.0 - rtP.Beta);

  /* Sum: '<S1>/Sum5' incorporates:
   *  Delay: '<S1>/Delay2'
   *  Gain: '<S1>/Lc_21'
   */
  rtb_Sum5 = rtb_Sum8 * (1.0 + rtP.Beta) * 3.0 / (2.0 * T) * rtb_Sum2_o +
    rtDW.Delay2_DSTATE;

  /* Gain: '<S1>/Lc_31' incorporates:
   *  Gain: '<S1>/Phi_13'
   */
  rtb_Sum8_tmp = T * T;

  /* Sum: '<S1>/Sum8' incorporates:
   *  Delay: '<S1>/Delay3'
   *  Gain: '<S1>/Lc_31'
   */
  rtb_Sum8 = rtb_Sum8 * (1.0 - rtP.Beta) / rtb_Sum8_tmp * rtb_Sum2_o +
    rtDW.Delay3_DSTATE;

  /* Gain: '<Root>/Gain' incorporates:
   *  Gain: '<Root>/Kd'
   *  Gain: '<Root>/Kp'
   *  Sum: '<Root>/Sum'
   *  Sum: '<Root>/Sum2'
   */
  rtb_Saturation = (((-2.0 * rtP.wc + 1.0) * rtb_Sum5 + rtP.wc * rtP.wc * rtb_ek)
                    - rtb_Sum8) * (1.0 / rtP.b0);

  /* Saturate: '<S1>/Saturation' */
  if (rtb_Saturation > 1.6) {
    rtb_Saturation = 1.6;
  } else {
    if (rtb_Saturation < -1.6) {
      rtb_Saturation = -1.6;
    }
  }
	/* End of Saturate: '<S1>/Saturation' */
	
	//手动加死区
	if(rtb_ek<=0.03 && rtb_ek>=-0.03)
	{
		rtY.uk=0;
	}
	else
	{
		/* Outport: '<Root>/Output' */
		rtY.uk = rtb_Saturation;
	}
	
  /* Outport: '<Root>/Output3' */
  rtY.ek = rtb_ek;

  /* Outport: '<Root>/Output1' */
  rtY.yhat = rtb_Sum3;

  /* Outport: '<Root>/Output4' */
  rtY.y_yhat = rtb_Sum2_o;

  /* Update for Delay: '<S2>/Delay' */
  rtDW.icLoad = 0U;
  rtDW.Delay_DSTATE = rtb_u0k;

  /* Gain: '<S1>/Gamma_11' incorporates:
   *  Gain: '<S1>/Gamma_12'
   */
  rtb_u0k = rtP.b0 * T;

  /* Update for Delay: '<S1>/Delay1' incorporates:
   *  Gain: '<S1>/Gamma_11'
   *  Gain: '<S1>/Phi_12'
   *  Gain: '<S1>/Phi_13'
   *  Sum: '<S1>/Sum'
   *  Sum: '<S1>/Sum9'
   */
  rtDW.Delay1_DSTATE = rtb_u0k * T / 2.0 * rtb_Saturation + (rtb_Sum8_tmp / 2.0 *
    rtb_Sum8 + (T * rtb_Sum5 + rtb_Sum3));

  /* Update for Delay: '<S1>/Delay2' incorporates:
   *  Gain: '<S1>/Gamma_12'
   *  Gain: '<S1>/Phi_21'
   *  Gain: '<S1>/Phi_23'
   *  Sum: '<S1>/Sum11'
   *  Sum: '<S1>/Sum12'
   */
  rtDW.Delay2_DSTATE = ((0.0 * rtb_Sum3 + rtb_Sum5) + T * rtb_Sum8) + rtb_u0k *
    rtb_Saturation;

  /* Update for Delay: '<S1>/Delay3' incorporates:
   *  Gain: '<S1>/Gamma_13'
   *  Gain: '<S1>/Phi_31'
   *  Gain: '<S1>/Phi_32'
   *  Sum: '<S1>/Sum10'
   *  Sum: '<S1>/Sum13'
   */
  rtDW.Delay3_DSTATE = ((0.0 * rtb_Sum3 + 0.0 * rtb_Sum5) + rtb_Sum8) + 0.0 *
    rtb_Saturation;
}

/* Model initialize function */
void LADRC_for_code_gen_initialize(void)
{
  /* InitializeConditions for Delay: '<S2>/Delay' */
  rtDW.icLoad = 1U;

  /* InitializeConditions for Delay: '<S1>/Delay1' */
  rtDW.Delay1_DSTATE = rtP.y_init;
}

void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;

  /* Disable interrupts here */

  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }

  OverrunFlag = true;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /* Set model inputs here */

  /* Step the model */
  LADRC_for_code_gen_step();

  /* Get model outputs here */

  /* Indicate task complete */
  OverrunFlag = false;

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
