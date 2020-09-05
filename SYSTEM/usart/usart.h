#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
#include "protocol.h"
 extern "C"
 {
#endif

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "led.h"
#include "info_to_send.h"
//////////////////////////////////////////////////////////////////////////////////	 

//STM32������
//����1��ʼ��		   

////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
     
#define USART2_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART2_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

#define USART3_REC_LEN  			200  	//�����������ֽ��� 10000
#define EN_USART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

#define COMMUNICATION_USART_DMA_CHANNEL         DMA_Channel_4
#define COMMUNICATION_UART_Tx_DMA_FLAG          DMA_IT_TCIF7
#define COMMUNICATION_UART_Tx_DMA_IRQ           DMA2_Stream7_IRQn
#define COMMUNICATION_UART_Tx_DMA_Channel       DMA2_Stream7
#define COMMUNICATION_UART_Rx_DMA_FLAG          DMA_IT_TCIF0
#define COMMUNICATION_UART_Rx_DMA_IRQ           DMA2_Channel4_IRQn
#define COMMUNICATION_UART_Rx_DMA_Channel       DMA2_Stream2
#define COMMUNICATION_UART                      USART1                  
#define DMA_DIR_PeripheralDST                   DMA_DIR_MemoryToPeripheral
#define COMMUNICATION_TX_BSIZE                  0
#define COMMUNICATION_RX_BSIZE                  20

extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	

extern u16  USART3_RX_BUF[USART3_REC_LEN]; //���ջ���,���USART3_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART3_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��

//����֡�����ٶ�ʵ����������޸ģ�
typedef struct
{
	//���֡����
	#define MAX_FRAME_LENGTH (256+6)
	//��С֡����
	#define MIN_FRAME_LENGTH  5	
	//�豸��ַ
	uint8_t Device_Address;
	//֡����
	uint8_t Function_Type;
	//֡����
	uint8_t Sequence;
	//��Ч���ݳ���
	uint8_t Data_Length;
	//����
	uint8_t *Data;
	//У��ֵ
	uint16_t Checksum;

}TransportUARTProtocol_Typedef;



//������
typedef enum
{
	//֡��ʽ����
	FRAME_FORMAT_ERR = 1,		
	//У��ֵ��ʽ����
	CHECK_FORMAR_ERR = 2,
	//У��ֵ��λ
	CHECK_ERR = 3,

}TransportUARTProtocol_Result;

extern u8 MCU1UARTRecevie[100];



//void MCU1_UART_Init(u32 bound);
void MCU1UART_DMA_cinfig(void);
void COMMUNICATION_Cmd_WriteParam( u8 sample_num, u8 *psz_param );
void COMMUNICATION_Cmd_WriteParam_ACK( u8 sta );
void uart_init(u32 bound);
void usart3_init(u32 bound);
void USART2_Init(u32 bound);
void Usart_SendByte(uint8_t data);
void Usart_SendByte_communication(uint8_t data);

//my
void my_usart_send_sys_state(struct info *info_);

#ifdef __cplusplus
 }
 #endif
#endif


