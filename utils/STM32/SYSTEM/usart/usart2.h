#ifndef __USART2_H_
#define __USART2_H_

#include "stm32f10x.h"
#include "stdio.h"

#define     URX_BUFFSIZE     9000   //接收数组的大小

extern char URX_RecFlag;    					//串口接收数据成功标志，0：接收完成，1：接收未完成
extern char URX_RECEIVE;        					//接收变量
extern char URX_RECE[URX_BUFFSIZE];      //接收数组
extern int rece_count;                //接收数据量


void Usart2_Init(uint32_t brr);
//void Usart_Init(uint32_t brr);
void USART2_SendByte(USART_TypeDef* USARTx, uint8_t Data);
void USART2_Send2Byte(USART_TypeDef* USARTx, uint16_t Data);
void USART2_SendArray(USART_TypeDef* USARTx, uint8_t *array);
void USART2_SendStr(USART_TypeDef* USARTx, uint8_t *str);

#endif
