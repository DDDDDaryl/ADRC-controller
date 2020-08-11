/*
 * File: LADRC_for_code_gen.h
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

#ifndef RTW_HEADER_LADRC_for_code_gen_h_
#define RTW_HEADER_LADRC_for_code_gen_h_
#include "rtwtypes.h"
#ifndef LADRC_for_code_gen_COMMON_INCLUDES_
# define LADRC_for_code_gen_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* LADRC_for_code_gen_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE;                 /* '<S2>/Delay' */
  real_T Delay1_DSTATE;                /* '<S1>/Delay1' */
  real_T Delay2_DSTATE;                /* '<S1>/Delay2' */
  real_T Delay3_DSTATE;                /* '<S1>/Delay3' */
  uint8_T icLoad;                      /* '<S2>/Delay' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T u0;                           /* '<Root>/u0' */
  real_T yk;                           /* '<Root>/y' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T uk;                           /* '<Root>/Output' */
  real_T yhat;                         /* '<Root>/Output1' */
  real_T uk_m;                         /* '<Root>/Output2' */
  real_T ek;                           /* '<Root>/Output3' */
  real_T y_yhat;                       /* '<Root>/Output4' */
} ExtY;

/* Parameters (default storage) */
struct P_ {
  real_T Beta;                         /* Variable: Beta
                                        * Referenced by:
                                        *   '<S1>/Lc_11'
                                        *   '<S1>/Lc_21'
                                        *   '<S1>/Lc_31'
                                        */
  real_T b0;                           /* Variable: b0
                                        * Referenced by:
                                        *   '<Root>/Gain'
                                        *   '<S1>/Gamma_11'
                                        *   '<S1>/Gamma_12'
                                        */
  real_T wc;                           /* Variable: wc
                                        * Referenced by:
                                        *   '<Root>/Kd'
                                        *   '<Root>/Kp'
                                        */
  real_T wc_bar;                       /* Variable: wc_bar
                                        * Referenced by:
                                        *   '<S2>/Constant'
                                        *   '<S2>/Gain1'
                                        *   '<S2>/Gain3'
                                        *   '<S2>/Gain4'
                                        */
  real_T y_init;                       /* Variable: y_init
                                        * Referenced by:
                                        *   '<S1>/Delay1'
                                        *   '<S2>/Constant'
                                        */
};

/* Parameters (default storage) */
typedef struct P_ P;

/* Imported (extern) block parameters */
extern real_T T;                       /* Variable: T
                                        * Referenced by:
                                        *   '<S1>/Gamma_11'
                                        *   '<S1>/Gamma_12'
                                        *   '<S1>/Lc_21'
                                        *   '<S1>/Lc_31'
                                        *   '<S1>/Phi_12'
                                        *   '<S1>/Phi_13'
                                        *   '<S1>/Phi_23'
                                        *   '<S2>/Constant'
                                        *   '<S2>/Gain1'
                                        *   '<S2>/Gain3'
                                        *   '<S2>/Gain4'
                                        */

/* Block parameters (default storage) */
extern P rtP;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
void LADRC_for_code_gen_initialize(void);
void LADRC_for_code_gen_step(void);
extern void rt_OneStep(void);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Gain' : Eliminated nontunable gain of 1
 * Block '<S1>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S1>/H_11_1' : Eliminated nontunable gain of 1
 * Block '<S1>/Phi_11' : Eliminated nontunable gain of 1
 * Block '<S1>/Phi_22' : Eliminated nontunable gain of 1
 * Block '<S1>/Phi_33' : Eliminated nontunable gain of 1
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LADRC_for_code_gen'
 * '<S1>'   : 'LADRC_for_code_gen/ESO'
 * '<S2>'   : 'LADRC_for_code_gen/Subsystem1'
 */
#endif                                 /* RTW_HEADER_LADRC_for_code_gen_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
