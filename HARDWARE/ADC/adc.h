#ifndef __ADC_H
#define __ADC_H	
#ifdef __cplusplus
 extern "C"
 {
#endif
#include "sys.h" 
#include "dma.h"
//////////////////////////////////////////////////////////////////////////////////	 

// STM32F407������
//ADC ��������	   
								  
////////////////////////////////////////////////////////////////////////////////// 	 
#define SEND_BUF_SIZE 2	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.	   
void Adc_Init(void); 				//ADCͨ����ʼ��
u16  Get_Adc(u8 ch); 				//���ĳ��ͨ��ֵ 
u16 Get_Adc_Average(u8 ch,u16 times);//�õ�ĳ��ͨ����������������ƽ��ֵ  
     
#ifdef __cplusplus
 }
 #endif
#endif 















