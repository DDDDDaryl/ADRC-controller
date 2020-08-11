#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 

// STM32F407开发板
//系统时钟初始化	
//包括时钟设置/中断管理/GPIO设置等

//////////////////////////////////////////////////////////////////////////////////  


//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}
//关闭所有中断(但是不包括fault和NMI中断)
void INTX_DISABLE(void)
{
	__ASM volatile("cpsid i");	  
}
//开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");
}
//设置栈顶地址
//addr:栈顶地址
void MSR_MSP(u32 addr) 
{
    __ASM volatile
        (
    "MSR MSP, r0\n\t"
    "BX r14"
    );
}
















