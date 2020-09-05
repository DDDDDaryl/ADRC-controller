#ifndef __DAC_H
#define __DAC_H	 
#ifdef __cplusplus
 extern "C"
 {
#endif
#include "sys.h"	     	
#include "stm32f4xx.h"
//////////////////////////////////////////////////////////////////////////////////	 

//STM32F407������
//DAC ��������	   

////////////////////////////////////////////////////////////////////////////////// 	

void Dac1_Init(void);		//DACͨ��1��ʼ��	 	 
void Dac1_Set_Vol(float vol);
void Dac2_Init(void);		//DACͨ��2��ʼ��	 	 
void Dac2_Set_Vol(float vol);	//����ͨ�������ѹ
void DAC_TIM_Config(void);
void set_output(float vol);
#ifdef __cplusplus
 }
 #endif
#endif

















