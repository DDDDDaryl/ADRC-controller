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

//STM32开发板
//串口1初始化		   

////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
     
#define USART2_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART2_RX 			1		//使能（1）/禁止（0）串口1接收

#define USART3_REC_LEN  			200  	//定义最大接收字节数 10000
#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口1接收

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

extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	

extern u16  USART3_RX_BUF[USART3_REC_LEN]; //接收缓冲,最大USART3_REC_LEN个字节.末字节为换行符 
extern u16 USART3_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义

//传输帧对象（再对实际情况进行修改）
typedef struct
{
	//最大帧长度
	#define MAX_FRAME_LENGTH (256+6)
	//最小帧长度
	#define MIN_FRAME_LENGTH  5	
	//设备地址
	uint8_t Device_Address;
	//帧功能
	uint8_t Function_Type;
	//帧序列
	uint8_t Sequence;
	//有效数据长度
	uint8_t Data_Length;
	//数据
	uint8_t *Data;
	//校验值
	uint16_t Checksum;

}TransportUARTProtocol_Typedef;



//传输结果
typedef enum
{
	//帧格式错误
	FRAME_FORMAT_ERR = 1,		
	//校验值格式错误
	CHECK_FORMAR_ERR = 2,
	//校验值错位
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


