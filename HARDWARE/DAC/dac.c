#include "dac.h"
//////////////////////////////////////////////////////////////////////////////////	 

//STM32F407������
//DAC ��������	   

////////////////////////////////////////////////////////////////////////////////// 	
extern float deadzone_compensation_dac1;
extern float deadzone_compensation_dac2;

u8 Tim2Flag=1;

uint32_t DualSine12bit[32];


//DACͨ��1�����ʼ��
void Dac1_Init(void)
{  
  GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//ʹ��DACʱ��
	   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//��ʹ�ô������� TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//��ʹ�ò��η���
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//���Ρ���ֵ����
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1�������ر� BOFF1=1
  DAC_Init(DAC_Channel_1,&DAC_InitType);	 //��ʼ��DACͨ��1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //ʹ��DACͨ��1
  
  DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12λ�Ҷ������ݸ�ʽ����DACֵ
}
//����ͨ��1�����ѹ
//vol:0~3300,����0~3.3V
void Dac1_Set_Vol(float vol)
{
	float temp=vol;
	temp/=1000;
	temp=temp*4095/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12λ�Ҷ������ݸ�ʽ����DACֵ
}

//DACͨ��2�����ʼ��
void Dac2_Init(void)
{  
  GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//ʹ��DACʱ��
	   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//��ʹ�ô������� TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//��ʹ�ò��η���
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//���Ρ���ֵ����
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC2�������ر� BOFF1=1
  DAC_Init(DAC_Channel_2,&DAC_InitType);	 //��ʼ��DACͨ��2

	DAC_Cmd(DAC_Channel_2, ENABLE);  //ʹ��DACͨ��2
  
  DAC_SetChannel2Data(DAC_Align_12b_R, 0);  //12λ�Ҷ������ݸ�ʽ����DACֵ
}
//����ͨ��1�����ѹ
//vol:0~3300,����0~3.3V
void Dac2_Set_Vol(float vol)
{
	float temp=vol;
	temp/=1000;
	temp=temp*4095/3.3;
	DAC_SetChannel2Data(DAC_Align_12b_R,temp);//12λ�Ҷ������ݸ�ʽ����DACֵ
}


/**
  * @brief  ����TIM
  * @param  ��
  * @retval ��
  */
void DAC_TIM_Config(void)
{
	
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* ʹ��TIM2ʱ�ӣ�TIM2CLK Ϊ168M */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
  /* TIM2������ʱ������ */
 // TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 10-1;       									//1kHz  
  TIM_TimeBaseStructure.TIM_Prescaler = (8400-1);       							//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  84M / 8400 = 10Khz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    						//ʱ�ӷ�Ƶϵ��
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//���ϼ���ģʽ
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(         //ʹ�ܻ���ʧ��ָ����TIM�ж�
        TIM2, //TIM2
        TIM_IT_Update  |  //TIM �ж�Դ
        TIM_IT_Trigger,   //TIM �����ж�Դ 
        ENABLE            //ʹ��
        );
     
   NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;            //TIM3�ж�
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //�����ȼ�3��
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ����ʹ��
   NVIC_Init(&NVIC_InitStructure);                            //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	/* ʹ��TIM2 */
	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)   //TIM2�ж�
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
        {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
       	Tim2Flag=1;
				}
}

void set_output(float vol) {
	if (vol < 0) {
		Dac1_Set_Vol(0);
		Dac2_Set_Vol(fabs(vol) * 1000);		
	} else {
		Dac2_Set_Vol(0);
		Dac1_Set_Vol(vol * 1000);
	}	
}




