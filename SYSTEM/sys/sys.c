#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 

// STM32F407������
//ϵͳʱ�ӳ�ʼ��	
//����ʱ������/�жϹ���/GPIO���õ�

//////////////////////////////////////////////////////////////////////////////////  


//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}
//�ر������ж�(���ǲ�����fault��NMI�ж�)
void INTX_DISABLE(void)
{
	__ASM volatile("cpsid i");	  
}
//���������ж�
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");
}
//����ջ����ַ
//addr:ջ����ַ
void MSR_MSP(u32 addr) 
{
    __ASM volatile
        (
    "MSR MSP, r0\n\t"
    "BX r14"
    );
}
















