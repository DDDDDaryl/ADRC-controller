#ifndef __LED_H
#define __LED_H
#ifdef __cplusplus
 extern "C"
 {
#endif
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 

//STM32F407开发板
//LED驱动代码	   
						  
////////////////////////////////////////////////////////////////////////////////// 	


//LED端口定义
#define LED2 PAout(1)	// D2


void LED_Init(void);//初始化		
#ifdef __cplusplus
 }
 #endif     
#endif
