#ifndef __DAC_H
#define __DAC_H	 
#ifdef __cplusplus
 extern "C"
 {
#endif
#include "sys.h"	     	
#include "stm32f4xx.h"
//////////////////////////////////////////////////////////////////////////////////	 

//STM32F407开发板
//DAC 驱动代码	   

////////////////////////////////////////////////////////////////////////////////// 	

void Dac1_Init(void);		//DAC通道1初始化	 	 
void Dac1_Set_Vol(float vol);
void Dac2_Init(void);		//DAC通道2初始化	 	 
void Dac2_Set_Vol(float vol);	//设置通道输出电压
void DAC_TIM_Config(void);
void set_output(float vol);
#ifdef __cplusplus
 }
 #endif
#endif

















