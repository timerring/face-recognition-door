#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"

void uart_init(u32 bound);
void UART1_SendCode(u8 *DATA,u8 len);
#endif


