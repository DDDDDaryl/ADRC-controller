/*
 * File: LADRC_for_code_gen_data.c
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

/* Block parameters (default storage) */
P rtP = {
  /* Variable: Beta
   * Referenced by:
   *   '<S1>/Lc_11'
   *   '<S1>/Lc_21'
   *   '<S1>/Lc_31'
   */
  0.30119421191220208,

  /* Variable: b0
   * Referenced by:
   *   '<Root>/Gain'
   *   '<S1>/Gamma_11'
   *   '<S1>/Gamma_12'
   */
  60.0,

  /* Variable: wc
   * Referenced by:
   *   '<Root>/Kd'
   *   '<Root>/Kp'
   */
  20.0,

  /* Variable: wc_bar
   * Referenced by:
   *   '<S2>/Constant'
   *   '<S2>/Gain1'
   *   '<S2>/Gain3'
   *   '<S2>/Gain4'
   */
  8.0,

  /* Variable: y_init
   * Referenced by:
   *   '<S1>/Delay1'
   *   '<S2>/Constant'
   */
  1.0
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
