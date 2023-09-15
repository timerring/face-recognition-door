#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "usart2.h"
#include "JQ8X00.h"
#include "string.h"
#include "stdlib.h"
#include "led.h"


int main(void)
{
		delay_init();	    	 //��ʱ������ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�                       //��������ָ�����һ������ʱ�ȴ�ģ�鴦�������һ��ָ��
	
		//����2��ʼ��
		Usart2_Init(9600);
	
		LED_Init();
		delay_ms(100);
	
		LED_On();
		delay_ms(500);
		LED_Off();
		delay_ms(500);
	
		LED_On();
		delay_ms(500);
		LED_Off();
	
		rece_count = 0;
		memset(URX_RECE,0,URX_BUFFSIZE);

    while(1)
    {
			if(URX_RecFlag == 0)   //���ճɹ�
			{
				URX_RecFlag = 1;
				
				//��������
				if(strcmp(URX_RECE,"{cewen_ok}") == 0)
				{
					LED_On();
					delay_ms(1000);
					LED_Off();
				}
				
				rece_count = 0;
				memset(URX_RECE,0,URX_BUFFSIZE); 			 //��ս��ջ�����
			}
    }
}

