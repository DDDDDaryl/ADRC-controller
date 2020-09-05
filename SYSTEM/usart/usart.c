#include "sys.h"
#include "usart.h"	
#include "time.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//STM32F4������
//����1��ʼ��		   

//extern union result
//{
//float d;
//u8 data[4];    
//}u_bound,l_bound,Disp,dac_ch1,dac_ch2,filtered_data,ladrc_output,Disp_est,Ref;
extern u8 busy_flag;
////////////////////////////////////////////////////////////////////////////////// 	  
//#define  EN_USART2_RX 0  //�Ƿ�ʹ��RX�ж�

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
//#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
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

//����_sys_exit()�Ա���ʹ�ð�����ģʽ    

//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
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
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	
u8 parse_flag=0;					//������ϣ���ʼ����
u16 header_flag = 0;

#endif
//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
}


void USART1_IRQHandler(void)                	//����1�жϷ������
{
	uint8_t Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������

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
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)
				{
					USART_RX_STA=0;
					//header_flag = 0;
				}//���մ���,���¿�ʼ
				else 
				{
					USART_RX_STA|=0x8000;
					//header_flag = 0;
                    C__set_PC_parse_flag(1);
                    //C__set_IC_parse_flag(1);
				}					//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)
					USART_RX_STA|=0x4000;
				else
				{
                    C__PC_rec_buf_write(Res, USART_RX_STA);
                    //C__IC_rec_buf_write(Res, USART_RX_STA);
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1)) {
						USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
						//header_flag = 0;
					}
				}
			}
		}   		 
  } 

#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 



void USART2_Init(u32 bound)
{
   //GPIO�˿�����
    GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);  //ʹ��GPIOAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //ʹ��USART2ʱ��
	
	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIO D5 ����ΪUSART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIO D6 ����ΪUSART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource7,GPIO_AF_USART2); //GPIO D7 ����ΪUSART2

	//USART2 �˿����� //GPIO D5 (TX)�� GPIO D6 (RX) GPIO D7  (SCK)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	     //�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //����
	GPIO_Init(GPIOD,&GPIO_InitStructure);          //��ʼ��PA2��PA3

   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate =   bound;     //����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;      //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;         //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	
	//��ʼ��ͬ��ͨ�� 
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Enable; // ����ʱ��
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low ; 
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge ;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable ;
	USART_ClockInit(USART2,&USART_ClockInitStructure);
#if EN_USART2_RX	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
    USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2 
	//USART_ClearFlag(USART2, USART_FLAG_TC);
}

void USART2_IRQHandler(void)                	//����2�жϷ������
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART2);//(USART2->DR);	//��ȡ���յ�������
        //printf("Res = %x\r\n", Res);
		//LED2=!LED2;
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)
				{
					USART_RX_STA=0;
				}//���մ���,���¿�ʼ
				else 
				{
					USART_RX_STA|=0x8000;
                    //C__set_PC_parse_flag(1);
                    C__set_IC_parse_flag(1);
				}					//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)
					USART_RX_STA|=0x4000;
				else
				{
                    //C__PC_rec_buf_write(Res, USART_RX_STA);
                    C__IC_rec_buf_write(Res, USART_RX_STA);
					USART_RX_STA++;
					if(USART_RX_STA>(USART2_REC_LEN-1))
						USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
  } 

#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 

/***************** ����һ���ֽ� **********************/
void Usart_SendByte(uint8_t data)
{
	/* ����һ���ֽ����ݵ�USART2 */
	USART_SendData(USART2,data);
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

void Usart_SendByte_communication(uint8_t data)
{
	/* ����һ���ֽ����ݵ�USART2 */
	USART_SendData(USART1,data);
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

/*****************************************************************************************
* Function Name: ADS_DMA_cinfig    DMA1_Stream0 Channel_4
* Description  : DMA���ݴ����ʼ��
* Arguments    : NONE
* Return Value : NONE
******************************************************************************************/
u8 MCU1UARTRecevie[100];
u8 COMMUNICATION_Tx_Buf[100];
void MCU1UART_DMA_cinfig(void)
{
	 DMA_InitTypeDef DMA_InitStructure;
	 NVIC_InitTypeDef   NVIC_InitStructure;  //�ж�

  /*����DMAʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/*--- COMMUNICATION_UART_Tx_DMA_Channel DMA Config ---*/

	DMA_Cmd(COMMUNICATION_UART_Tx_DMA_Channel, DISABLE);                           // ��DMAͨ��

	DMA_DeInit(COMMUNICATION_UART_Tx_DMA_Channel);                                 // �ָ�ȱʡֵ
	/* ȷ��DMA��������λ��� */
  while (DMA_GetCmdStatus(COMMUNICATION_UART_Tx_DMA_Channel) != DISABLE)  {}
	/*UART1 tx��Ӧdma2��ͨ��4��������7*/	
  DMA_InitStructure.DMA_Channel = COMMUNICATION_USART_DMA_CHANNEL;  

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&COMMUNICATION_UART->DR);// ���ô��ڷ������ݼĴ���

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)COMMUNICATION_Tx_Buf;        // ���÷��ͻ������׵�ַ

	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                             // ��������λĿ�꣬�ڴ滺���� ->����Ĵ���

	DMA_InitStructure.DMA_BufferSize = COMMUNICATION_TX_BSIZE;                     // ��Ҫ���͵��ֽ�����������ʵ��������Ϊ0����Ϊ��ʵ��Ҫ���͵�ʱ�򣬻��������ô�ֵ

	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;               // �����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�

	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                        // �ڴ滺������ַ���ӵ���

	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;        // �������ݿ��8λ��1���ֽ�

	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                // �ڴ����ݿ��8λ��1���ֽ�

	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                  // ���δ���ģʽ

	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                        // ���ȼ�����
	
	/*����FIFO*/
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                                   //����
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;                            //���� 
  /*�洢��ͻ������ 16������*/
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                             //������  
  /*����ͻ������ 1������*/
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;                             //����

//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                 // �ر��ڴ浽�ڴ��DMAģʽ

	DMA_Init(COMMUNICATION_UART_Tx_DMA_Channel, &DMA_InitStructure);               // д������

//	DMA_ClearFlag(COMMUNICATION_UART_Tx_DMA_FLAG);                               // ���DMA���б�־
	/* Configure the NVIC Preemption Priority Bits */

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	/* Enable the DMA Interrupt */

	NVIC_InitStructure.NVIC_IRQChannel = COMMUNICATION_UART_Tx_DMA_IRQ;            // ����DMAͨ�����ж�����

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;                      // ���ȼ�����

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(COMMUNICATION_UART_Tx_DMA_Channel, DISABLE);                           // �ر�DMA

	DMA_ITConfig(COMMUNICATION_UART_Tx_DMA_Channel, DMA_IT_TC, ENABLE);            // ��������DMAͨ���ж�
  
  /* ��λ��ʼ��DMA������ */
  DMA_DeInit(DMA1_Stream7);                                                      //��DMAӳ���ѡ����������ͨ��

  /* ȷ��DMA��������λ��� */
  while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}

		
  /*usart1 rx��Ӧdma2��ͨ��4��������2*/	
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  /*����DMAԴ��SPI1->DR*/
//  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint16_t)(UART5->DR & (uint16_t)0x01FF);    	//�����ַ                //�����ַ��(uint16_t)(UART5->DR & (uint16_t)0x01FF)
	DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&(USART1->DR) ;
  /*�ڴ��ַ(Ҫ����ı�����ָ��)*/
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&MCU1UARTRecevie;                            //�ڴ��ַ��д����          //�ڴ棬Ҳ���Ǳ��������ַ
  /*���򣺴����赽�ڴ�*/		
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;	                                //���ݴ洢����
  /*�����С*/	
  DMA_InitStructure.DMA_BufferSize = 20; //����������                                        //������������DMA������������ж��ź�
  /*�����ַ����*/	    
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;               
  /*�ڴ��ַ����*/
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	  
  /*�������ݵ�λ*/	 
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;                    //8λ
  /*�ڴ����ݵ�λ 16bit*/
  DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;	
  /*DMAģʽ������ѭ��*/
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	                                             //����ԭ�����ݣ��ظ���
  /*���ȼ�*/	
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;      
  /*����FIFO*/
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                                   //����
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;                            //���� 
  /*�洢��ͻ������ 16������*/
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                              //������  
  /*����ͻ������ 1������*/
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;                      //����
  /*����DMA2��������2*/		   
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);                                    //ʹ���жϽ��н��գ��������NVIC�����ȼ��飬1��ʾ1Bit��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                          //��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;                                 //�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);                                   //ʹ���жϽ��н��գ��������NVIC�����ȼ��飬1��ʾ1Bit��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                         //��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;                                //�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
	USART_DMACmd(COMMUNICATION_UART, USART_DMAReq_Tx, ENABLE);                        // ��������DMA����
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
  /*ʹ��DMA*/
  DMA_Cmd(DMA2_Stream2, ENABLE);
	
  
  /* �ȴ�DMA��������Ч*/
  while(DMA_GetCmdStatus(DMA2_Stream2) != ENABLE){}
}
 
/*****************************************************************************************
* Function Name: 
* Description  : ����5DMA����
* Arguments    : NONE
* Return Value : NONE
******************************************************************************************/

void COMMUNICATION_Uart_DAM_Tx_Over(void)
{
    DMA_ClearFlag(DMA2_Stream7, COMMUNICATION_UART_Tx_DMA_FLAG);              // �����־
    DMA_Cmd(COMMUNICATION_UART_Tx_DMA_Channel, DISABLE);                      // �ر�DMAͨ��
//    OSMboxPost(mbLumModule_Tx, (void*)1);                                   // ���ñ�־λ���������õ���UCOSII �����Ը����Լ�����������޸�
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
    COMMUNICATION_UART_Tx_DMA_Channel->NDTR = (uint16_t)size;                  //����Ҫ���͵��ֽ���Ŀ
    DMA_Cmd(COMMUNICATION_UART_Tx_DMA_Channel, ENABLE);                        //��ʼDMA����
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
//	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=0xAB;                                    //��ѹ��������Ƶ�ʣ���λ  ÿһ��Ϊ32λ
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
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=0x03;            //֡ͷ	                         
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=sta;							//״̬
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=0x0d;						//֡β
	COMMUNICATION_Tx_Buf[COMMUNICATION_Tx_Index++]=0x0a;

	COMMUNICATION_Uart_Start_DMA_Tx( COMMUNICATION_Tx_Index );
}

/*****************************************************************************************
* Function Name: 
* Description  : ����5DMA����
* Arguments    : NONE
* Return Value : NONE
******************************************************************************************/
void COMMUNICATION_Uart_DMA_Rx_Data(void)
{
	DMA_Cmd(COMMUNICATION_UART_Rx_DMA_Channel, DISABLE);                                         // �ر�DMA ����ֹ����
	DMA_ClearFlag(DMA2_Stream2, COMMUNICATION_UART_Rx_DMA_FLAG );                                // ��DMA��־λ
//	size = COMMUNICATION_RX_BSIZE - DMA_GetCurrDataCounter(COMMUNICATION_UART_Rx_DMA_Channel); //��ý��յ����ֽ���
	COMMUNICATION_UART_Rx_DMA_Channel->NDTR = COMMUNICATION_RX_BSIZE;                             //���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ	
	
	DMA_Cmd(COMMUNICATION_UART_Rx_DMA_Channel, ENABLE);                                           // DMA �������ȴ����ݡ�ע�⣬����жϷ�������֡�����ʺܿ죬MCU����������˴ν��յ������ݣ��ж��ַ������ݵĻ������ﲻ�ܿ������������ݻᱻ���ǡ���2�ַ�ʽ�����
}

u8 Res;
//void UART5_IRQHandler(void)                                                                   	//����5�жϷ�����򣨿����жϣ�Ȼ���ٿ���DMMA�������Ͳ����������ˣ�������
//{
//	if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)//�����ж�
//	{
//		COMMUNICATION_Uart_DMA_Rx_Data();
//		Res = USART_ReceiveData( UART5 ); // Clear IDLE interrupt flag bit
//	}
//}


