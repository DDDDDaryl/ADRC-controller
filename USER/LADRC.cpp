#include <stddef.h>
#include <stdio.h>              /* This ert_main.c example uses printf/fflush */
#include "LADRC_for_code_gen.h"        /* Model's header file */
#include "rtwtypes.h"
#include "LADRC.h"


///*--�����ɲ���--*/
//static float Ts 		= 0.01f;
//static float b0 		= 50.0f;
//static float Omega = 8.0f;
//static float Beta	= 0.5f;		//�۲�������
//static float Alpha = 50.0f;	//����������
//static float Kp		= 0.0f;
//static float Kd 		= 0.0f;

///*--��ɢϵͳ״̬�ռ�ϵ��--*/
//static float Phi[3][3];
//static float Gamma[3][1];
//static float Lc[3][1];
//static float H[1][3];
//static float J=0;

///*--��ɢ״̬�۲���ϵ��--*/
//static float A[3][3];
//static float B[3][2];
//static float C[3][3];
//static float D[3][2];
	
/* Attach rt_OneStep to a timer or interrupt service routine with
 * period 0.01 seconds (the model's base sample time) here.  The
 * call syntax for rt_OneStep is
 *
 *  rt_OneStep();
 */
//void rt_OneStep(void);
//void rt_OneStep(void)
//{
//  static boolean_T OverrunFlag = false;

//  /* Disable interrupts here */

//  /* Check for overrun */
//  if (OverrunFlag) {
//    return;
//  }

//  OverrunFlag = true;

//  /* Save FPU context here (if necessary) */
//  /* Re-enable timer or interrupt here */
//  /* Set model inputs here */
//	
//  /* Step the model */
//  LADRC_for_code_gen_step();

//  /* Get model outputs here */
//	
//  /* Indicate task complete */
//  OverrunFlag = false;

//  /* Disable interrupts here */
//  /* Restore FPU context here (if necessary) */
//  /* Enable interrupts here */
//}


