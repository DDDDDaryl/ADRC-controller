#ifndef __DELAY_H
#define __DELAY_H 		
#ifdef __cplusplus
 extern "C"
 {
#endif
#include <sys.h>	  
//////////////////////////////////////////////////////////////////////////////////  

// STM32F407开发板
//使用SysTick的普通计数模式对延迟进行管理(支持ucosii)
//包括delay_us,delay_ms

////////////////////////////////////////////////////////////////////////////////// 	 
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
#ifdef __cplusplus
 }
 #endif
#endif





























