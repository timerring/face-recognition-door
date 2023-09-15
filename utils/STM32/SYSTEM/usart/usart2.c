#include "usart2.h"



char URX_RecFlag = 1;    					//串口接收数据成功标志，0：接收完成，1：接收未完成
char URX_RECEIVE;        					//接收变量
char URX_RECE[URX_BUFFSIZE];      //接收数组
int rece_count=0;                //接收数据量


void Usart2_Init(uint32_t brr)
{
	USART_InitTypeDef USART_InitStruct;
//	USART_ClockInitTypeDef USART_ClockInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	//打开串口GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	//打开串口外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	//配置USART TX 的GPIO为推挽复用
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	//配置USART RX 的GPIO为浮空输入
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	//配置串口的工作参数
	//配置波特率（传入参数）
	USART_InitStruct.USART_BaudRate = brr;
	//配置硬件流控制
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//配置工作模式，发送和接收
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//配置校验位
	USART_InitStruct.USART_Parity = USART_Parity_No;  //无奇偶校验
	//配置停止位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	//配置数据位
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  //完成串口的初始化配置
	USART_Init(USART2,&USART_InitStruct);
	
	USART_ClearFlag(USART2, USART_FLAG_RXNE);	            //清除接收标志位
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);        //开启接收中断
	
	USART_Cmd(USART2,ENABLE);
	
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
}

/*   库函数发送一个字节数据   */
void USART2_SendByte(USART_TypeDef* USARTx, uint8_t Data)
{
	USART_SendData(USARTx,Data);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == 0 );
}

/*   库函数发送两个字节数据   */
void USART2_Send2Byte(USART_TypeDef* USARTx, uint16_t Data)
{
	uint8_t temp_h,temp_l;
	temp_h = (Data & 0xff00) >> 8;
	temp_l = Data & 0xff;
	USART_SendData(USARTx,temp_h);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == 0 );
	USART_SendData(USARTx,temp_l);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == 0 );
}

/*   库函数发送一个数组数据   */
void USART2_SendArray(USART_TypeDef* USARTx, uint8_t *array)
{
	uint8_t i=0;
	while( array[i] != '\0' )
	{
		USART2_SendByte(USARTx, array[i++]);
	}
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 0 );
}

/*   库函数发送一个字符串   */
void USART2_SendStr(USART_TypeDef* USARTx, uint8_t *str)
{
	do
	{
		USART2_SendByte(USARTx, *str);
	}while( *(++str) != '\0' );
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 0 );
}

/**
	*串口2中断函数
**/
void USART2_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE))
	{
		if(URX_RecFlag)							//未接受完成
		{
			URX_RECEIVE = USART2->DR;    //接收数据存放到变量URX_RECEIVE
			if(URX_RECEIVE != '}')
			{
				URX_RECE[rece_count++] = URX_RECEIVE;
			}
			else
			{
				URX_RECE[rece_count++] = URX_RECEIVE;
				URX_RecFlag = 0;       //接收完成
			}
		}
		USART_ClearFlag(USART2,USART_FLAG_RXNE);
	}
}

