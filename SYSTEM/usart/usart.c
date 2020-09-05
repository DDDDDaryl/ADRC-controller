#include "sys.h"
#include "usart.h"	
#include "time.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//STM32F4开发板
//串口1初始化		   

//extern union result
//{
//float d;
//u8 data[4];    
//}u_bound,l_bound,Disp,dac_ch1,dac_ch2,filtered_data,ladrc_output,Disp_est,Ref;
extern u8 busy_flag;
////////////////////////////////////////////////////////////////////////////////// 	  
//#define  EN_USART2_RX 0  //是否使能RX中断

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
//#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
//struct __FILE 
//{ 
//	int handle; 
//}; 
typedef int FILEHANDLE;

// Disable semihosting 
// Note:
//   Use microlib will disable semihosting
//   If not, disable semihosting using folllow code
__asm(".global __use_no_semihosting\n\t");
//FILE __stdout;       

//定义_sys_exit()以避免使用半主机模式    

//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}

void _ttywrch(int ch)
{
ch = ch;
}

int ferror(FILE *f)
{
(void)f;
return EOF;
}

void _sys_exit(int return_code)
{
(void)return_code;
while (1) {
};
}

FILEHANDLE _sys_open(const char *name, int openmode)
{
return 1;
}

int _sys_close(FILEHANDLE fh)
{
return 0;
}

int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned len, int mode)
{
//your_device_write(buf, len);
return 0;
}

int _sys_read(FILEHANDLE fh, unsigned char *buf, unsigned len, int mode)
{
return -1;
}

int _sys_istty(FILEHANDLE fh)
{
return 0;
}

int _sys_seek(FILEHANDLE fh, long pos)
{
return -1;
}

long _sys_flen(FILEHANDLE fh)
{
return -1;
}
time_t time(time_t *t)  
{  
    return 0;  
}  
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	
u8 parse_flag=0;					//接收完毕，开始解析
u16 header_flag = 0;

#endif
//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	USART_Cmd(USART1, ENABLE);  //使能串口1 
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	uint8_t Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据

		//LED2=!LED2;
//		if (!(header_flag & 0x8000) || !(header_flag & 0x4000)) {
//			if (Res == 0xeb && (header_flag & 0x4000) == 0) {
//				header_flag |= 0x80;
//			}
//			else if (Res == 0x90) {
//				if ((header_flag & 0x8000) == 1) {
//					header_flag |= 0x4000;
//				} else {
//					header_flag = 0;
//				}						
//			}
//			return;
//		}
//		if (Res == 0xeb) header_flag = 1;
//		if (!header_flag) return;
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)
				{
					USART_RX_STA=0;
					//header_flag = 0;
				}//接收错误,重新开始
				else 
				{
					USART_RX_STA|=0x8000;
					//header_flag = 0;
                    C__set_PC_parse_flag(1);
                    //C__set_IC_parse_flag(1);
				}					//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)
					USART_RX_STA|=0x4000;
				else
				{
                    C__PC_rec_buf_write(Res, USART_RX_STA);
                    //C__IC_rec_buf_write(Res, USART_RX_STA);
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1)) {
						USART_RX_STA=0;//接收数据错误,重新开始接收	  
						//header_flag = 0;
					}
				}
			}
		}   		 
  } 

#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 



void USART2_Init(u32 bound)
{
   //GPIO端口设置
    GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);  //使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //使能USART2时钟
	
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIO D5 复用为USART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIO D6 复用为USART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource7,GPIO_AF_USART2); //GPIO D7 复用为USART2

	//USART2 端口配置 //GPIO D5 (TX)与 GPIO D6 (RX) GPIO D7  (SCK)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	     //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);          //初始化PA2，PA3

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate =   bound;     //波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;      //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;         //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
	//初始化同步通信 
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Enable; // 开启时钟
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low ; 
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge ;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable ;
	USART_ClockInit(USART2,&USART_ClockInitStructure);
#if EN_USART2_RX	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
    USART_Cmd(USART2, ENABLE);  //使能串口2 
	//USART_ClearFlag(USART2, USART_FLAG_TC);
}

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART2);//(USART2->DR);	//读取接收到的数据
        //printf("Res = %x\r\n", Res);
		//LED2=!LED2;
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)
				{
					USART_RX_STA=0;
				}//接收错误,重新开始
				else 
				{
					USART_RX_STA|=0x8000;
                    //C__set_PC_parse_flag(1);
                    C__set_IC_parse_flag(1);
				}					//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)
					USART_RX_STA|=0x4000;
				else
				{
                    //C__PC_rec_buf_write(Res, USART_RX_STA);
                    C__IC_rec_buf_write(Res, USART_RX_STA);
					USART_RX_STA++;
					if(USART_RX_STA>(USART2_REC_LEN-1))
						USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
  } 

#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 

/***************** 发送一个字节 **********************/
void Usart_SendByte(uint8_t data)
{
	/* 发送一个字节数据到USART2 */
	USART_SendData(USART2,data);
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

void Usart_SendByte_communication(uint8_t data)
{
	/* 发送一个字节数据到USART2 */
	USART_SendData(USART1,data);
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

/*****************************************************************************************
* Function Name: ADS_DMA_cinfig    DMA1_Stream0 Channel_4
* Description  : DMA数据传输初始化
* Arguments    : NONE
* Return Value : NONE
******************************************************************************************/
u8 MCU1UARTRecevie[100];
u8 COMMUNICATION_Tx_Buf[100];
void MCU1UART_DMA_cinfig(void)
{
	 DMA_InitTypeDef DMA_InitStructure;
	 NVIC_InitTypeDef   NVIC_InitStructure;  //中断

  /*开启DMA时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/*--- COMMUNICATION_UART_Tx_DMA_Channel DMA Config ---*/

	DMA_Cmd(COMMUNICATION_UART_Tx_DMA_Channel, DISABLE);                           // 关DMA通道

	DMA_DeInit(COMMUNICATION_UART_Tx_DMA_Channel);                                 // 恢复缺省值
	/* 确保DMA数据流复位完成 */
  while (DMA_GetCmdStatus(COMMUNICATION_UART_Tx_DMA_Channel) != DISABLE)  {}
	/*UART1 tx对应dma2，通道4，数据流7*/	
  DMA_InitStructure.DMA_Channel = COMMUNICATION_USART_DMA_CHANNEL;  

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&COMMUNICATION_UART->DR);// 设置串口发送数据寄存器

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)COMMUNICATION_Tx_Buf;        // 设置发送缓冲区首地址

	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                             // 设置外设位目标，内存缓冲区 ->外设寄存器

	DMA_InitStructure.DMA_BufferSize = COMMUNICATION_TX_BSIZE;                     // 需要发送的字节数，这里其实可以设置为0，因为在实际要发送的时候，会重新设置次值

	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;               // 外设地址不做增加调整，调整不调整是DMA自动实现的

	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                        // 内存缓冲区地址增加调整

	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;        // 外设数据宽度8位，1个字节

	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                // 内存数据宽度8位，1个字节

	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                  // 单次传输模式

	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                        // 优先级设置
	
	/*禁用FIFO*/
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                                   //？？
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;                            //？？ 
  /*存储器突发传输 16个节拍*/
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                             //？？？  
  /*外设突发传输 1个节拍*/
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;                             //？？

//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                 // 关闭内存到内存的DMA模式

	DMA_Init(COMMUNICATION_UART_Tx_DMA_Channel, &DMA_InitStructure);               // 写入配置

//	DMA_ClearFlag(COMMUNICATION_UART_Tx_DMA_FLAG);                               // 清除DMA所有标志
	/* Configure the NVIC Preemption Priority Bits */

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	/* Enable the DMA Interrupt */

	NVIC_InitStructure.NVIC_IRQChannel = COMMUNICATION_UART_Tx_DMA_IRQ;            // 发送DMA通道的中断配置

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;                      // 优先级设置

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(COMMUNICATION_UART_Tx_DMA_Channel, DISABLE);                           // 关闭DMA

	DMA_ITConfig(COMMUNICATION_UART_Tx_DMA_Channel, DMA_IT_TC, ENABLE);            // 开启发送DMA通道中断
  
  /* 复位初始化DMA数据流 */
  DMA_DeInit(DMA1_Stream7);                                                      //由DMA映射表选择数据流与通道

  /* 确保DMA数据流复位完成 */
  while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}

		
  /*usart1 rx对应dma2，通道4，数据流2*/	
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  /*设置DMA源：SPI1->DR*/
//  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint16_t)(UART5->DR & (uint16_t)0x01FF);    	//外设地址                //外设地址，(uint16_t)(UART5->DR & (uint16_t)0x01FF)
	DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&(USART1->DR) ;
  /*内存地址(要传输的变量的指针)*/
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&MCU1UARTRecevie;                            //内存地址，写变量          //内存，也就是变量数组地址
  /*方向：从外设到内存*/		
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;	                                //数据存储方向
  /*传输大小*/	
  DMA_InitStructure.DMA_BufferSize = 20; //接收数据量                                        //传输数据量，DMA利用这个产生中断信号
  /*外设地址不增*/	    
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;               
  /*内存地址自增*/
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	  
  /*外设数据单位*/	 
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;                    //8位
  /*内存数据单位 16bit*/
  DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;	
  /*DMA模式：不断循环*/
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	                                             //覆盖原来数据，重复存
  /*优先级*/	
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;      
  /*禁用FIFO*/
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                                   //？？
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;                            //？？ 
  /*存储器突发传输 16个节拍*/
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                              //？？？  
  /*外设突发传输 1个节拍*/
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;                      //？？
  /*配置DMA2的数据流2*/		   
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);                                    //使用中断进行接收，因此设置NVIC的优先级组，1表示1Bit抢占优先级
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                          //抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;                                 //子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);                                   //使用中断进行接收，因此设置NVIC的优先级组，1表示1Bit抢占优先级
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                         //抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;                                //子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
	USART_DMACmd(COMMUNICATION_UART, USART_DMAReq_Tx, ENABLE);                        // 开启串口DMA发送
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
  /*使能DMA*/
  DMA_Cmd(DMA2_Stream2, ENABLE);
	
  
  /* 等待DMA数据流有效*/
  while(DMA_GetCmdStatus(DMA2_Stream2) != ENABLE){}
}
 
/*****************************************************************************************
* Function Name: 
* Description  : 串口5DMA发送
* Arguments    : NONE
* Return Value : NONE
******************************************************************************************/

void COMMUNICATION_Uart_DAM_Tx_Over(void)
{
    DMA_ClearFlag(DMA2_Stream7, COMMUNICATION_UART_Tx_DMA_FLAG);              // 清除标志
    DMA_Cmd(COMMUNICATION_UART_Tx_DMA_Channel, DISABLE);                      // 关闭DMA通道
//    OSMboxPost(mbLumModule_Tx, (void*)1);                                   // 设置标志位，这里我用的是UCOSII ，可以根据自己的需求进行修改
}

void DMA2_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7))                           // DMA_GetFlagStatus
    {
        COMMUNICATION_Uart_DAM_Tx_Over();
    }
}

void COMMUNICATION_Uart_Start_DMA_Tx(uint16_t size)
{
    COMMUNICATION_UART_Tx_DMA_Channel->NDTR = (uint16_t)size;                  //设置要发送的字节数目
    DMA_Cmd(COMMUNICATION_UART_Tx_DMA_Channel, ENABLE);                        //开始DMA发送
}
/*
info_id_ref = 0x81,
	info_id_transient_profile,
	info_id_output_pos,
	info_id_err,
	info_id_control_signal,
	info_id_ESO_first_order_state,
	info_id_ESO_second_order_state,
	info_id_ESO_third_order_state,
	info_id_ESO_fourth_order_state

	float ref;
	float transient_profile;
	float sensor_pos;
	float err;
	float ESO_order1;
	float ESO_order2;
	float ESO_order3;
	float ctrl_sig;
*/


void my_usart_send_sys_state(struct info *info_) {
	static union float_transmit trans;
		
	int COMMUNICATION_Tx_Index = 0;
	
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = 0xeb;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = 0x90;
	
	u8 len = 4 + 1 + 5 * 8;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = len;
	
	trans.integer = info_id_ref;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	trans.data = info_->ref;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[1];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[2];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[3];
	
	trans.integer = info_id_transient_profile;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	trans.data = info_->transient_profile;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[1];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[2];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[3];
	
	
	trans.integer = info_id_output_pos;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	trans.data = info_->sensor_pos;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[1];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[2];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[3];
	
	trans.integer = info_id_err;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	trans.data = info_->err;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[1];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[2];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[3];
	
	trans.integer = info_id_ESO_first_order_state;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	trans.data = info_->ESO_order1;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[1];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[2];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[3];
	
	trans.integer = info_id_ESO_second_order_state;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	trans.data = info_->ESO_order2;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[1];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[2];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[3];
	
	trans.integer = info_id_ESO_third_order_state;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	trans.data = info_->ESO_order3;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[1];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[2];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[3];
	
	trans.integer = info_id_control_signal;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	trans.data = info_->ctrl_sig;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[0];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[1];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[2];
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = trans.d[3];
	
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = 0x0d;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++] = 0x0a;  
	
	COMMUNICATION_Uart_Start_DMA_Tx( COMMUNICATION_Tx_Index );
}

void COMMUNICATION_Cmd_WriteParam( u8 sample_num, u8 *psz_param )
{
//	u8 COMMUNICATION_Tx_Index ;
//	COMMUNICATION_Tx_Index = 0;
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=0xAB;                                    //电压，电流，频率，相位  每一个为32位
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=u_bound.data[0];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=u_bound.data[1];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=u_bound.data[2];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=u_bound.data[3];
//		
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Disp.data[0];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Disp.data[1];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Disp.data[2];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Disp.data[3];
//	
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=dac_ch1.data[0];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=dac_ch1.data[1];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=dac_ch1.data[2];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=dac_ch1.data[3];
//	
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=dac_ch2.data[0];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=dac_ch2.data[1];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=dac_ch2.data[2];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=dac_ch2.data[3];

//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=l_bound.data[0];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=l_bound.data[1];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=l_bound.data[2];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=l_bound.data[3];
//	
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=filtered_data.data[0];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=filtered_data.data[1];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=filtered_data.data[2];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=filtered_data.data[3];
//	
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=ladrc_output.data[0];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=ladrc_output.data[1];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=ladrc_output.data[2];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=ladrc_output.data[3];
//	
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Disp_est.data[0];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Disp_est.data[1];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Disp_est.data[2];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Disp_est.data[3];
//	
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Ref.data[0];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Ref.data[1];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Ref.data[2];
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=Ref.data[3];
//	
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=0xCD;
//	COMMUNICATION_Uart_Start_DMA_Tx( COMMUNICATION_Tx_Index );
	
}

void COMMUNICATION_Cmd_WriteParam_ACK( u8 sta )
{
//    u8 err;
	u8 COMMUNICATION_Tx_Index ;
	COMMUNICATION_Tx_Index = 0;
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=0x03;            //帧头	                         
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=sta;							//状态
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=0x0d;						//帧尾
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=0x0a;

	COMMUNICATION_Uart_Start_DMA_Tx( COMMUNICATION_Tx_Index );
}

/*****************************************************************************************
* Function Name: 
* Description  : 串口5DMA接收
* Arguments    : NONE
* Return Value : NONE
******************************************************************************************/
void COMMUNICATION_Uart_DMA_Rx_Data(void)
{
	DMA_Cmd(COMMUNICATION_UART_Rx_DMA_Channel, DISABLE);                                         // 关闭DMA ，防止干扰
	DMA_ClearFlag(DMA2_Stream2, COMMUNICATION_UART_Rx_DMA_FLAG );                                // 清DMA标志位
//	size = COMMUNICATION_RX_BSIZE - DMA_GetCurrDataCounter(COMMUNICATION_UART_Rx_DMA_Channel); //获得接收到的字节数
	COMMUNICATION_UART_Rx_DMA_Channel->NDTR = COMMUNICATION_RX_BSIZE;                             //重新赋值计数值，必须大于等于最大可能接收到的数据帧数目	
	
	DMA_Cmd(COMMUNICATION_UART_Rx_DMA_Channel, ENABLE);                                           // DMA 开启，等待数据。注意，如果中断发送数据帧的速率很快，MCU来不及处理此次接收到的数据，中断又发来数据的话，这里不能开启，否则数据会被覆盖。有2种方式解决。
}

u8 Res;
//void UART5_IRQHandler(void)                                                                   	//串口5中断服务程序（空闲中断，然后再开启DMMA，这样就不会有乱序了！！！）
//{
//	if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)//空闲中断
//	{
//		COMMUNICATION_Uart_DMA_Rx_Data();
//		Res = USART_ReceiveData( UART5 ); // Clear IDLE interrupt flag bit
//	}
//}


