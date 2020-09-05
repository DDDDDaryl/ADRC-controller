#include "dac.h"
//////////////////////////////////////////////////////////////////////////////////	 

//STM32F407开发板
//DAC 驱动代码	   

////////////////////////////////////////////////////////////////////////////////// 	
extern float deadzone_compensation_dac1;
extern float deadzone_compensation_dac2;

u8 Tim2Flag=1;

uint32_t DualSine12bit[32];


//DAC通道1输出初始化
void Dac1_Init(void)
{  
  GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//使能DAC时钟
	   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//不使用触发功能 TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1输出缓存关闭 BOFF1=1
  DAC_Init(DAC_Channel_1,&DAC_InitType);	 //初始化DAC通道1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //使能DAC通道1
  
  DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
}
//设置通道1输出电压
//vol:0~3300,代表0~3.3V
void Dac1_Set_Vol(float vol)
{
	float temp=vol;
	temp/=1000;
	temp=temp*4095/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
}

//DAC通道2输出初始化
void Dac2_Init(void)
{  
  GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//使能DAC时钟
	   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//不使用触发功能 TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC2输出缓存关闭 BOFF1=1
  DAC_Init(DAC_Channel_2,&DAC_InitType);	 //初始化DAC通道2

	DAC_Cmd(DAC_Channel_2, ENABLE);  //使能DAC通道2
  
  DAC_SetChannel2Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
}
//设置通道1输出电压
//vol:0~3300,代表0~3.3V
void Dac2_Set_Vol(float vol)
{
	float temp=vol;
	temp/=1000;
	temp=temp*4095/3.3;
	DAC_SetChannel2Data(DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
}


/**
  * @brief  配置TIM
  * @param  无
  * @retval 无
  */
void DAC_TIM_Config(void)
{
	
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 使能TIM2时钟，TIM2CLK 为168M */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
  /* TIM2基本定时器配置 */
 // TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 10-1;       									//1kHz  
  TIM_TimeBaseStructure.TIM_Prescaler = (8400-1);       							//设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  84M / 8400 = 10Khz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    						//时钟分频系数
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//向上计数模式
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(         //使能或者失能指定的TIM中断
        TIM2, //TIM2
        TIM_IT_Update  |  //TIM 中断源
        TIM_IT_Trigger,   //TIM 触发中断源 
        ENABLE            //使能
        );
     
   NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;            //TIM3中断
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //从优先级3级
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道被使能
   NVIC_Init(&NVIC_InitStructure);                            //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	/* 使能TIM2 */
	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)   //TIM2中断
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
        {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
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




