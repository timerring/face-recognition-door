#ifndef __USART2_H_
#define __USART2_H_

#include "stm32f10x.h"
#include "stdio.h"

#define     URX_BUFFSIZE     9000   //��������Ĵ�С

extern char URX_RecFlag;    					//���ڽ������ݳɹ���־��0��������ɣ�1������δ���
extern char URX_RECEIVE;        					//���ձ���
extern char URX_RECE[URX_BUFFSIZE];      //��������
extern int rece_count;                //����������


void Usart2_Init(uint32_t brr);
//void Usart_Init(uint32_t brr);
void USART2_SendByte(USART_TypeDef* USARTx, uint8_t Data);
void USART2_Send2Byte(USART_TypeDef* USARTx, uint16_t Data);
void USART2_SendArray(USART_TypeDef* USARTx, uint8_t *array);
void USART2_SendStr(USART_TypeDef* USARTx, uint8_t *str);

#endif
