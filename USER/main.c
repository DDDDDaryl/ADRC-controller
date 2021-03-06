#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "adc.h"
#include "dac.h"
#include "pid.h"
#include "signal_generator.h"
#include "LADRC_for_code_gen.h"
#include "kalman.h"
#include "stdio.h"


//STM32F407开发板

//extern __IO uint16_t SendBuff[SEND_BUF_SIZE];
//#define  	Sampling_frequency 	(u16)		1000														//采样频率1kHz
//#define  	Totol_time 					(float)	0.01														//采样频率10s
//#define 	Sample 							(int)		Totol_time * Sampling_frequency;

/*------------------------运行模式宏定义------------------------*/
//低四位选择模式，高四位选择信号或控制算法
#define		DAC_EMERGENCY_SHUTDOWN			0x00

#define		DAC_CONTROL_OUTPUT					0x01
#define		DAC_MANUAL_OUTPUT						0x02

#define		SINE_SIGNAL									0x10
#define		RAMP_SIGNAL									0x20
#define		SQUARE_WAVE									0x40
#define		ID_SIGNAL										0x80

#define		PID_CONTROLLER							0x10
#define		LADRC												0x20	
/*-------------------------------------------------------------*/
u16 	adcx;
u16		global_period_point = 0;
float temp;
float dacvol1 = 0;																		//输出电压范围0-3300（对应0-3.3V）
float dacvol2 = 0;
float upper_bound = 100;															//mm,传感器有效量程上界
float	lower_bound = 0;																//mm,传感器有效量程下届
float Measure[9];
float relative_max_range = 0;

extern u8 		Tim2Flag;
extern u8 		parse_flag;
extern u8 		USART_RX_BUF[USART_REC_LEN];
extern u16 		USART_RX_STA;       											//接收状态标记	
extern float 	sine_freq;
extern float 	amp;
extern float sine_output_data[10000];
extern float timeline[10000];
extern float Indent_signal[10000];
extern KFP KFP_height;


u8 					busy_flag 								= 0;						//勿扰标志位，DAC输出信号时串口不接收数据
u32 				cycle 											= 0;
uint8_t 		up_2_ten										=	0;

/*----------------数据帧中包含的可调参数---------------*/
float 			ramp_start_vol							= 0;
float 			ramp_end_vol								= 0;
float 			dac_sine_amp								= 0;
float 			dac_sine_freq								= 0;
u8					run_time										= 10;
u16					sample_point								= 10000;
float				deadzone_compensation_dac1	=	0;								//死区补偿电压，V
float				deadzone_compensation_dac2	=	0;								//死区补偿电压，V
float				ID_Signal_amp								= 0;								//系统辨识信号幅值
float 			current_output 							= 0;
float 			current_control 						= 0;
float				SetPoint										= 0;								//控制目标(V)
float				Sample_time									=	50;								//控制器采样周期(ms)
real_T 			T														= 0;
//PID控制器参数
float				PID_Kp											= 0;
float				PID_Ki											= 0;
float				PID_Kd											= 0;
//LADRC参数
float				ADRC_b0											= 0;
float				ADRC_Beta										= 0;
float				ADRC_wo											= 0;
float				ADRC_wc											= 0;
float				ADRC_wc_bar									= 0;
//卡尔曼滤波参数
float				Kalman_Q										= 0.1;
float				Kalman_R										= 0.543;


u32 				count												= 0;											//测试用


//u8 Tim3Flag=1;                                   		//定时标志位

union result                                     			//测量发送数据         
{
float d;
u8 data[4];    
}u_bound,l_bound,Disp,dac_ch1,dac_ch2,Rec,filtered_data,ladrc_output,Disp_est,Ref;

struct cmd_flags
{
	//首先是一个字节8位用于标志DAC1和DAC2的状态
	//0x00，均无输出
	u8 dac_output_none_and_none;
	//0x10，DAC1输出信号
	u8 dac_output_positive_and_none;
	//0x01，DAC2输出信号
	u8 dac_output_none_and_positive;
	//0x11，非法
	u8 illegal;
	u8 other_options;
	u8 mode_byte;																//存储运行模式
} cmd;

void Usart_Measure(void);
//void Timer3_Init();
void gpio_test_init(void);
u8 frame_parse(void);
void Boundary_detection(void);

int main(void)
{ 
	u8 	i							= 0;
	u8 	key							= 0;           //保存键值
	u8 	Sta							=	0;
	//u16 times						= 0;  
	u16 dacval					    = 0;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	uart_init(115200);	//串口初始化波特率为115200
	USART2_Init(115200);
	MCU1UART_DMA_cinfig();                         //串口DMA通信初始化
	
	LED_Init();		  		//初始化与LED连接的硬件接口  
	KEY_Init();       //初始化与按键连接的硬件接口
	
	Adc_Init(); 				//adc初始化
	Dac1_Init();		 		//DAC通道1初始化	
	Dac2_Init();		 		//DAC通道1初始化	
	DAC_TIM_Config();
	
	DAC_SetChannel1Data(DAC_Align_12b_R,dacval);//初始值为0	
	DAC_SetChannel2Data(DAC_Align_12b_R,dacval);//初始值为0	
	
	Boundary_detection();
	
	Dac1_Set_Vol(dacvol1);
	Dac2_Set_Vol(dacvol2);
	
	gpio_test_init();
	
	memset(&cmd, 0, sizeof(cmd));//结构体置空
	cmd.dac_output_none_and_none=1;//初始DAC无动作
	while(1)
	{   
		key=KEY_Scan(0);		//得到键值
		if(key)
		{
			Sta=1;
		}
/*--------------------------------------串口接收中断-------------------------------------*/
		if(parse_flag==1)
		{
			//标志位复位
			
			parse_flag=0;
			USART_RX_STA=0;

			if( frame_parse() )
			{
				COMMUNICATION_Cmd_WriteParam_ACK(0x80);	//接收到错误信息
			}
			
			/*-------------------------参数初始化区域------------------------*/
			if(cmd.mode_byte == (DAC_MANUAL_OUTPUT|RAMP_SIGNAL) )//0x22
			{	
				if(cmd.dac_output_none_and_positive)
				{
					dacvol2 = (float)ramp_start_vol;
				}
				if(cmd.dac_output_positive_and_none)
				{
					dacvol1 = (float)ramp_start_vol;
				}
			}
			if(cmd.mode_byte == (DAC_CONTROL_OUTPUT|PID_CONTROLLER) )//0x11
			{
				IncPIDInit();
				sptr->SetPoint	= SetPoint;		//设定值
				sptr->Proportion = PID_Kp;
				sptr->Integral = PID_Ki;
				sptr->Derivative = PID_Kd;
			}
			if(cmd.mode_byte == (DAC_CONTROL_OUTPUT|LADRC) )//0x11
			{
				relative_max_range = 3.3f/(upper_bound/125.0f);
				current_output 							= ((float)Get_Adc(3)/(4096-1)) * relative_max_range;
				rtU.yk											= current_output;
				rtU.u0 											= SetPoint;
				T 													= (double)Sample_time/1000;
				rtP.b0											= ADRC_b0;
				rtP.wc											= ADRC_wc;
				rtP.wc_bar									= ADRC_wc_bar;
				ADRC_Beta										= exp(-ADRC_wo*T);
				rtP.Beta										= ADRC_Beta;
				rtP.y_init									= current_output;
				KFP_height.Q								= Kalman_Q;
				KFP_height.R								=	Kalman_R;
			}
			
			/*--------------------------------------------------------------*/
			memset( USART_RX_BUF ,0 ,( USART_REC_LEN * sizeof(u8)));//清空缓存区

			
			if(cmd.dac_output_none_and_none)
			{
				LED2=!LED2;
				for(i=0;i<1;i++)
				{
					delay_ms(100);
					LED2=!LED2;
				}  
			}
			if(cmd.dac_output_positive_and_none)
			{
				LED2=!LED2;
				for(i=0;i<3;i++)
				{
					delay_ms(100);
					LED2=!LED2;
				}
			}
			if(cmd.dac_output_none_and_positive)
			{
				LED2=!LED2;
				for(i=0;i<5;i++)
				{
					delay_ms(100);
					LED2=!LED2;
				}
			}
			if(cmd.other_options)
			{
				LED2=!LED2;
				for(i=0;i<7;i++)
				{
					delay_ms(100);
					LED2=!LED2;
				}
			}
			if(cmd.illegal)
			{
				LED2=!LED2;
				for(i=0;i<9;i++)
				{
					delay_ms(100);
					LED2=!LED2;
				}
			}
			
		}
/*-------------------------------------根据接收信号选择运行方式----------------------------------*/   
/*-------------------------------------正常运行模式----------------------------------*/  		
		while(!cmd.dac_output_none_and_none)							//任一DAC动作时
		{
			/*-------------------------手动输出，斜坡信号----------------------------------*/
			if(cmd.mode_byte == (DAC_MANUAL_OUTPUT|RAMP_SIGNAL) )//0x22
			{
				USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//失能相关中断，避免在运行过程中接收其他信号
	//--------------------------------------产生10s斜坡信号，采样率1kHz，0-1s为0V，9-10s为3.3V----------------------------------------
				if(cmd.dac_output_none_and_positive)
				{
					if(Tim2Flag==1)
					{
						up_2_ten++;
						Tim2Flag=0;
						if((cycle<=1000||cycle>(sample_point-1000))&&cycle<=sample_point)
						{
							cycle++;
							Dac1_Set_Vol(0);//确保DAC1无动作
							Dac2_Set_Vol(dacvol2+1000*deadzone_compensation_dac2);
						}
						if(cycle>1000&&cycle<=(sample_point-1000))
						{
							cycle++;
							dacvol2+=(ramp_end_vol-ramp_start_vol)/(float)(sample_point-2000);
							Dac1_Set_Vol(0);//确保DAC1无动作
							Dac2_Set_Vol(dacvol2+1000*deadzone_compensation_dac2);
						}
						if(cycle>sample_point)
						{
							Usart_Measure(); //数据测量与发送
							cycle=0;
							Sta=0;
							dacvol2=0;
							deadzone_compensation_dac1=0;
							deadzone_compensation_dac2=0;
							Dac2_Set_Vol(dacvol2);
							cmd.dac_output_none_and_none=1;
							cmd.dac_output_none_and_positive=0;
							up_2_ten=0;
							USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
						}						
						//记够十次
						if(up_2_ten==10)
						{
							//GPIO_ToggleBits(GPIOB,GPIO_Pin_13);
							up_2_ten=0;
							Usart_Measure(); //数据测量与发送
							//printf("Displacement:%f mm\r\n",Disp.d);
						}
					}
				}//DAC2动作
				
				if(cmd.dac_output_positive_and_none)
				{
					if(Tim2Flag==1)
					{
						up_2_ten++;
						Tim2Flag=0;
						if((cycle<=1000||cycle>(sample_point-1000))&&cycle<=sample_point)
						{
							cycle++;
							Dac1_Set_Vol(dacvol1+1000*deadzone_compensation_dac1);
							Dac2_Set_Vol(0);
						}
						if(cycle>1000&&cycle<=(sample_point-1000))
						{
							cycle++;
							dacvol1+=(ramp_end_vol-ramp_start_vol)/(float)0x1F40;
							Dac1_Set_Vol(dacvol1+1000*deadzone_compensation_dac1);
							Dac2_Set_Vol(0);
						}
						if(cycle>sample_point)
						{
							Usart_Measure(); //数据测量与发送
							cycle=0;
							Sta=0;
							dacvol1=0;
							deadzone_compensation_dac1=0;
							deadzone_compensation_dac2=0;
							Dac1_Set_Vol(dacvol1);
							cmd.dac_output_none_and_none=1;
							cmd.dac_output_positive_and_none=0;
							up_2_ten=0;
							USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
						}
						
						//记够十次
						if(up_2_ten==10)
						{
							//GPIO_ToggleBits(GPIOB,GPIO_Pin_13);
							up_2_ten=0;
							Usart_Measure(); //数据测量与发送
						}
					}
				}//DAC1动作
				if(cmd.illegal||cmd.other_options)
				{
					cmd.illegal=0;
					cmd.dac_output_none_and_none=1;
					up_2_ten=0;
					USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
				}
			}
			/*-------------------------手动输出，正弦信号----------------------------------*/
			if(cmd.mode_byte == (DAC_MANUAL_OUTPUT|SINE_SIGNAL) )//0x12
			{
				float err = 0.0f;
				u16 cycle_tmp=0;
				float relative_max_range=3.3f/(upper_bound/125.0f);
				
				USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//失能相关中断，避免在运行过程中接收其他信号
				if(Tim2Flag==1)
				{
					up_2_ten++;
					Tim2Flag=0;
					if(cycle<sample_point)
					{
						cycle_tmp=cycle%global_period_point;
						cycle++;
						err=sine_output_data[cycle_tmp]+1.65f-((float)Get_Adc(3)/(4096-1))*relative_max_range;
						if(err<0)
						{
							dacvol1 = 0;
							dacvol2 = fabs(sine_output_data[cycle_tmp]+1.65f)*1000;
							Dac1_Set_Vol(dacvol1);
							Dac2_Set_Vol(fabs(err)*1000+1000*deadzone_compensation_dac2);
						}
						else
						{
							dacvol2 = 0;
							dacvol1 = fabs(sine_output_data[cycle_tmp]+1.65f)*1000;
							Dac1_Set_Vol(fabs(err)*1000+1000*deadzone_compensation_dac1);
							Dac2_Set_Vol(dacvol2);
						}
						
					}
					else
					{
						cycle=0;
						cycle_tmp=0;
						cmd.dac_output_none_and_none=1;
						cmd.dac_output_none_and_positive=0;
						cmd.dac_output_positive_and_none=0;
						up_2_ten=0;
						dacvol1=0;
						dacvol2=0;
						dac_sine_amp=0;
						dac_sine_freq=0;
						clear_load();
						deadzone_compensation_dac1=0;
						deadzone_compensation_dac2=0;
						Dac1_Set_Vol(dacvol1);
						Dac2_Set_Vol(dacvol2);
						USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
					}
					if(up_2_ten==10)
					{
						up_2_ten=0;
						Usart_Measure(); //数据测量与发送
					}
				}
			}
			/*-------------------------手动输出，方波信号----------------------------------*/
			if(cmd.mode_byte == (DAC_MANUAL_OUTPUT|SQUARE_WAVE) )
			{
				
			}
			/*-------------------------手动输出，系统辨识用随机正弦信号----------------------------------*/
			if(cmd.mode_byte == (DAC_MANUAL_OUTPUT|ID_SIGNAL) )
			{
//				USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//失能相关中断，避免在运行过程中接收其他信号
//				if(Tim2Flag==1)
//				{
//					up_2_ten++;
//					Tim2Flag=0;
//					if(cycle<10000)
//					{
//						cycle++;
//						if(cmd.other_options)
//						{
//							if(Indent_signal[cycle]<0)
//							{
//								dacvol1 = 0;
//								dacvol2 = fabs(Indent_signal[cycle]);
//								Dac1_Set_Vol(dacvol1);
//								Dac2_Set_Vol(dacvol2+1000*deadzone_compensation_dac2);
//							}
//							else
//							{
//								dacvol2 = 0;
//								dacvol1 = fabs(Indent_signal[cycle]);
//								Dac1_Set_Vol(dacvol1+1000*deadzone_compensation_dac1);
//								Dac2_Set_Vol(dacvol2);
//							}
//						}
//						else if(cmd.dac_output_none_and_positive)
//						{
//							dacvol1 = 0;
//							dacvol2 = (ID_Signal_amp/1.5f)*Indent_signal[cycle]+ID_Signal_amp*1000;
//							Dac1_Set_Vol(dacvol1);
//							Dac2_Set_Vol(dacvol2+1000*deadzone_compensation_dac2);
//						}
//						else if(cmd.dac_output_positive_and_none)
//						{
//							dacvol2 = 0;
//							dacvol1 = (ID_Signal_amp/1.5f)*Indent_signal[cycle]+ID_Signal_amp*1000;
//							Dac2_Set_Vol(dacvol2);
//							Dac1_Set_Vol(dacvol1+1000*deadzone_compensation_dac1);
//						}
//						
//					}
//					else
//					{
//						cycle=0;
//						cmd.dac_output_none_and_none=1;
//						cmd.dac_output_none_and_positive=0;
//						cmd.dac_output_positive_and_none=0;
//						up_2_ten=0;
//						dacvol1=0;
//						dacvol2=0;
//						deadzone_compensation_dac1=0;
//						deadzone_compensation_dac2=0;
//						Dac1_Set_Vol(dacvol1);
//						Dac2_Set_Vol(dacvol2);
//						USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
//					}
//					if(up_2_ten==10)
//					{
//						up_2_ten=0;
//						Usart_Measure(); //数据测量与发送
//					}
//				}
			}
			/*-------------------------控制器输出，PID控制----------------------------------*/
			if(cmd.mode_byte == (DAC_CONTROL_OUTPUT|PID_CONTROLLER) )//0x11
			{
				USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//失能相关中断，避免在运行过程中接收其他信号
				if(Tim2Flag==1)
				{
					Tim2Flag=0;
					if(cycle<sample_point)
					{
						cycle++;
						up_2_ten++;
						if(up_2_ten==Sample_time)
						{
							up_2_ten=0;
							current_output = ((float)(Get_Adc(3))/4096)*125;
							current_control = IncPIDCalc(current_output);
							if(sptr->Is_not_negative)
							{
								dacvol1 = current_control;
								dacvol2 = 0;
								Dac1_Set_Vol(dacvol1+1000*deadzone_compensation_dac1);
								Dac2_Set_Vol(dacvol2);
							}
							else
							{
								dacvol2 = current_control;
								dacvol1 = 0;
								Dac2_Set_Vol(dacvol2+1000*deadzone_compensation_dac2);
								Dac1_Set_Vol(dacvol1);
							}
						}
					}
					else
					{
						cycle=0;
						cmd.dac_output_none_and_none=1;
						up_2_ten=0;
						dacvol1=0;
						dacvol2=0;
						deadzone_compensation_dac1=0;
						deadzone_compensation_dac2=0;
						Dac1_Set_Vol(dacvol1);
						Dac2_Set_Vol(dacvol2);
						USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
					}
					if(up_2_ten%10==0)
					{
						count++;
						Usart_Measure(); //数据测量与发送
					}
				}
				
				
			}
			/*-------------------------控制器输出，LADRC控制----------------------------------*/
			if(cmd.mode_byte == (DAC_CONTROL_OUTPUT|LADRC) )
			{
				USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//失能相关中断，避免在运行过程中接收其他信号
				if(Tim2Flag==1)
				{
					Tim2Flag=0;
					if(cycle<sample_point)
					{
						cycle++;
						up_2_ten++;
						if(up_2_ten==Sample_time)
						{
							up_2_ten=0;
							current_output = ((float)Get_Adc(3)/(4096-1)) * relative_max_range;
							//rtU.yk = (double)current_output;					//当前输出
							rtU.yk = kalmanFilter(&KFP_height, current_output);
							rtP.y_init=rtU.yk;
							rt_OneStep();
							current_control = (float)rtY.uk;
							if(current_control>=0)
							{
								dacvol1 = 1000*current_control;
								dacvol2 = 0;
								Dac1_Set_Vol(dacvol1+1000*deadzone_compensation_dac1);
								Dac2_Set_Vol(dacvol2);
							}
							else
							{
								dacvol2 = fabs(1000*current_control);
								dacvol1 = 0;
								Dac2_Set_Vol(dacvol2+1000*deadzone_compensation_dac2);
								Dac1_Set_Vol(dacvol1);
							}
						}
					}
					else
					{
						cycle=0;
						cmd.dac_output_none_and_none=1;
						up_2_ten=0;
						dacvol1=0;
						dacvol2=0;
						deadzone_compensation_dac1=0;
						deadzone_compensation_dac2=0;
						Dac1_Set_Vol(dacvol1);
						Dac2_Set_Vol(dacvol2);
						USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
					}
					if(up_2_ten%10==0)
					{
						
					}
					if(up_2_ten%((u16)Sample_time/5)==0)
					{
						Usart_Measure(); //数据测量与发送
					}
				}
				
			}
		}
	}
}

/*******数据测量与发送********/
void Usart_Measure(void)
{
//	u8 t;
//	u8 len=20;
	
	u_bound.d=0;
	l_bound.d=0;
	Disp.d=0;
	dac_ch1.d=0;
	dac_ch2.d=0;
	ladrc_output.d=0;
	filtered_data.d=0;
	Disp_est.d=0;
	
	//输出测量值
	Measure[0]=Get_Adc(3);   //Disp
	//位移最大值
	Measure[1]=upper_bound;   //mm
	//控制量理想值
	Measure[2]=(float)dacvol1;
	Measure[3]=(float)dacvol2;
	//位移最小值
	Measure[4]=lower_bound;   //mm
	//归一化的位移测量值
	Measure[5]=(float)current_output;
	//卡尔曼滤波后的位移测量值
	Measure[6]=(float)rtU.yk;
	//ESO的位移估计值
	Measure[7]=(float)rtY.yhat;
	Measure[8]=(float)rtY.uk_m;
											
	Disp.d=((float)Measure[0]/(4096-1))*125;           //125mm为位移传感器量程，Disp表示的为当前位移
	u_bound.d=(float)Measure[1];     //输出电压模拟值
	dac_ch1.d=(float)Measure[2]/1000;
	dac_ch2.d=(float)Measure[3]/1000;
	l_bound.d=(float)Measure[4];
	ladrc_output.d=Measure[5];
	filtered_data.d=Measure[6];
	Disp_est.d=Measure[7];
	Ref.d=Measure[8];
	//串口1数据发送
	COMMUNICATION_Cmd_WriteParam(0, 0);
}

void gpio_test_init(void){
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOA时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//LED2对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

  GPIO_SetBits(GPIOB,GPIO_Pin_13);//PA1  设置高，灯灭
}


u8 frame_parse(void)
{
	u8 i =0;
	u8 *p = USART_RX_BUF;
	u8 current_byte = 0;
	
	//检测帧头
	if( (*p) != 0x02)
	{
		if( (*p) == 0x04)
		{
			Usart_Measure();
			p=NULL;
			return 0;
		}
		p=NULL;
		return 1;
	}
	else
	{
		/*---------第二字节--------*/
		current_byte = *(++p);
		switch (current_byte)
		{
			case 0x00:
			{
				cmd.dac_output_none_and_none			=	1;
				cmd.dac_output_positive_and_none	=	0;
				cmd.dac_output_none_and_positive	=	0;
				cmd.illegal												=	0;
				cmd.other_options									= 0;
			}
			break;
			
			case 0x10:
			{
				cmd.dac_output_none_and_none			=	0;
				cmd.dac_output_positive_and_none	=	1;
				cmd.dac_output_none_and_positive	=	0;
				cmd.illegal												=	0;
				cmd.other_options									= 0;
			}
			break;
			
			case 0x01:
			{
				cmd.dac_output_none_and_none			=	0;
				cmd.dac_output_positive_and_none	=	0;
				cmd.dac_output_none_and_positive	=	1;
				cmd.illegal												=	0;
				cmd.other_options									= 0;
			}
			break;
			
			case 0xFF:
			{
				cmd.dac_output_none_and_none			=	0;
				cmd.dac_output_positive_and_none	=	0; 
				cmd.dac_output_none_and_positive	=	0;
				cmd.illegal												=	0;
				cmd.other_options									= 1;
			}
			break;
			
			default:
			{
				cmd.dac_output_none_and_none			=	0;
				cmd.dac_output_positive_and_none	=	0;
				cmd.dac_output_none_and_positive	=	0;
				cmd.illegal												=	1;
				cmd.other_options									= 0;
			}
		}
		/*---------第三字节--------*/
		current_byte = *(++p);
		cmd.mode_byte = current_byte;
		/*---------第4字节--------*/
		current_byte = *(++p);
		run_time = current_byte;
		sample_point = run_time * DAC_UPLOAD_FREQ;
		/*---------第5~12字节--------*/
		if(cmd.mode_byte == (DAC_MANUAL_OUTPUT|RAMP_SIGNAL) )//0x22
		{
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			ramp_start_vol = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			ramp_end_vol = Rec.d;
		}
		if(cmd.mode_byte == (DAC_MANUAL_OUTPUT|SINE_SIGNAL) )//0x12
		{
			//解析幅值频率
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			dac_sine_amp = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			dac_sine_freq = Rec.d;
			amp=dac_sine_amp;
			sine_freq=dac_sine_freq;
			//填充数据
			global_period_point=load_sine_output_data();
		}
		if(cmd.mode_byte == (DAC_CONTROL_OUTPUT|PID_CONTROLLER) 
			|| cmd.mode_byte == (DAC_CONTROL_OUTPUT|LADRC))//0x11
		{
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			SetPoint = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			Sample_time = Rec.d;
		}
		if(cmd.mode_byte == (DAC_MANUAL_OUTPUT|ID_SIGNAL) )
		{
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			ID_Signal_amp = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
		}
		/*---------第13~20字节--------*/
		for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			deadzone_compensation_dac1 = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			deadzone_compensation_dac2 = Rec.d;
		/*---------第21~36字节--------*/
		if(cmd.mode_byte == (DAC_CONTROL_OUTPUT|PID_CONTROLLER) )//0x11
		{
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			PID_Kp = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			PID_Ki = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			PID_Kd = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
		}
		if(cmd.mode_byte == (DAC_CONTROL_OUTPUT|LADRC) )//0x21
		{
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			ADRC_b0 = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			ADRC_wo = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			ADRC_wc = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			ADRC_wc_bar = Rec.d;
		}
	}
	/*---------第37~38字节--------*/
	if(cmd.mode_byte == (DAC_CONTROL_OUTPUT|LADRC) )//0x21
		{
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			Kalman_Q = Rec.d;
			for(i=3; (i>=0)&&(i!=0xFF); i--)
			{
				current_byte = *(++p);
				Rec.data[i] = current_byte;
			}
			Kalman_R = Rec.d;
		}
	p=NULL;
	return 0;
}

void Boundary_detection(void)
{
	//达到上界
	Dac1_Set_Vol(3300);
	Dac2_Set_Vol(0);
	delay_ms(1500);
	upper_bound=((float)Get_Adc_Average(3,20)/(4096-1))*125;
	//达到下界
	Dac2_Set_Vol(3300);
	Dac1_Set_Vol(0);
	delay_ms(1500);
	lower_bound=((float)Get_Adc_Average(3,20)/(4096-1))*125;
	//复位
	Dac2_Set_Vol(0);
	Dac1_Set_Vol(0);
}

 /*******通用定时器中断初始化********/
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
 /*******通用定时器中断初始化********/
//void Timer3_Init()
//{
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

//    TIM_TimeBaseStructure.TIM_Period = 100-1;                       //设置在下一个更新事件装入活动的自动重装载寄存器周期的值，计数到5000为500ms，计数到50为5ms
//    TIM_TimeBaseStructure.TIM_Prescaler =(8400-1);               //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  84M / 8400 = 10Khz
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;                 //设置时钟分割:TDTS = Tck_tim
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);              //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
// 
//     
//    TIM_ITConfig(         //使能或者失能指定的TIM中断
//        TIM3, //TIM2
//        TIM_IT_Update  |  //TIM 中断源
//        TIM_IT_Trigger,   //TIM 触发中断源 
//        ENABLE            //使能
//        );
//     
//    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;            //TIM3中断
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //从优先级3级
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道被使能
//    NVIC_Init(&NVIC_InitStructure);                            //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
// 
//    TIM_Cmd(TIM3, ENABLE);                                     //使能TIMx外设
//                             
//}
///*******通用定时器中断********/
//void TIM3_IRQHandler(void)   //TIM3中断
//{
//    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
//        {
//        TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
//       	Tim3Flag=1;
//				}
//}
