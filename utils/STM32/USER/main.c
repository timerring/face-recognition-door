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
		delay_init();	    	 //延时函数初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级                       //连续发送指令，加入一定的延时等待模块处理完成上一条指令
	
		//串口2初始化
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
			if(URX_RecFlag == 0)   //接收成功
			{
				URX_RecFlag = 1;
				
				//体温正常
				if(strcmp(URX_RECE,"{cewen_ok}") == 0)
				{
					LED_On();
					delay_ms(1000);
					LED_Off();
				}
				
				rece_count = 0;
				memset(URX_RECE,0,URX_BUFFSIZE); 			 //清空接收缓冲区
			}
    }
}

