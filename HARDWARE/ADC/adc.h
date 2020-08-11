#ifndef __ADC_H
#define __ADC_H	
#ifdef __cplusplus
 extern "C"
 {
#endif
#include "sys.h" 
#include "dma.h"
//////////////////////////////////////////////////////////////////////////////////	 

// STM32F407开发板
//ADC 驱动代码	   
								  
////////////////////////////////////////////////////////////////////////////////// 	 
#define SEND_BUF_SIZE 2	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.	   
void Adc_Init(void); 				//ADC通道初始化
u16  Get_Adc(u8 ch); 				//获得某个通道值 
u16 Get_Adc_Average(u8 ch,u16 times);//得到某个通道给定次数采样的平均值  
     
#ifdef __cplusplus
 }
 #endif
#endif 















