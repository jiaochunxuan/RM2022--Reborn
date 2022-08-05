#include "main.h"
#include "EncoderTask.h"
#include "PIDregulation.h"
#include "MotorTask.h"
#include "timer.h"
#include "RM_Cilent_UI.h"

extern void allInit(void);
//Robomaster工程标准模板
uint8_t LABELLABEL_1=0;//观测光电管变量
extern u8 time5_capture_sta;
extern u16 time5_capture_val;
u32 temp=0,l=0,f=0;		
int main()
{
//delay函数确定可用
//uart函数确定可用
	allInit();//工程车初始化GPIO_InitTypeDef gpio;
	//UI_init();

	while(1)
	{
		//delay_ms(20);  
		uart_test();
	}
	
	}

	
