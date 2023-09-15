#include "usart2.h"



char URX_RecFlag = 1;    					//���ڽ������ݳɹ���־��0��������ɣ�1������δ���
char URX_RECEIVE;        					//���ձ���
char URX_RECE[URX_BUFFSIZE];      //��������
int rece_count=0;                //����������


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
	
	//�򿪴���GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	//�򿪴�������ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	//����USART TX ��GPIOΪ���츴��
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	//����USART RX ��GPIOΪ��������
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	//���ô��ڵĹ�������
	//���ò����ʣ����������
	USART_InitStruct.USART_BaudRate = brr;
	//����Ӳ��������
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//���ù���ģʽ�����ͺͽ���
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//����У��λ
	USART_InitStruct.USART_Parity = USART_Parity_No;  //����żУ��
	//����ֹͣλ
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	//��������λ
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  //��ɴ��ڵĳ�ʼ������
	USART_Init(USART2,&USART_InitStruct);
	
	USART_ClearFlag(USART2, USART_FLAG_RXNE);	            //������ձ�־λ
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);        //���������ж�
	
	USART_Cmd(USART2,ENABLE);
	
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
}

/*   �⺯������һ���ֽ�����   */
void USART2_SendByte(USART_TypeDef* USARTx, uint8_t Data)
{
	USART_SendData(USARTx,Data);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == 0 );
}

/*   �⺯�����������ֽ�����   */
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

/*   �⺯������һ����������   */
void USART2_SendArray(USART_TypeDef* USARTx, uint8_t *array)
{
	uint8_t i=0;
	while( array[i] != '\0' )
	{
		USART2_SendByte(USARTx, array[i++]);
	}
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 0 );
}

/*   �⺯������һ���ַ���   */
void USART2_SendStr(USART_TypeDef* USARTx, uint8_t *str)
{
	do
	{
		USART2_SendByte(USARTx, *str);
	}while( *(++str) != '\0' );
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 0 );
}

/**
	*����2�жϺ���
**/
void USART2_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE))
	{
		if(URX_RecFlag)							//δ�������
		{
			URX_RECEIVE = USART2->DR;    //�������ݴ�ŵ�����URX_RECEIVE
			if(URX_RECEIVE != '}')
			{
				URX_RECE[rece_count++] = URX_RECEIVE;
			}
			else
			{
				URX_RECE[rece_count++] = URX_RECEIVE;
				URX_RecFlag = 0;       //�������
			}
		}
		USART_ClearFlag(USART2,USART_FLAG_RXNE);
	}
}

