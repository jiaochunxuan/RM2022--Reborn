#include "TIM6Init.h"
/*函数声明*/
extern void controlTask(void);

void TIM6_Init(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	/* -------------- Configure TIM6 ----------------------------------------*/
	{
		TIM_TimeBaseInitTypeDef  tim;
    
    tim.TIM_Prescaler = 84-1;      //84M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000-1;  //1ms中断一次
    TIM_TimeBaseInit(TIM6,&tim);
	}
	/* -------------- Configure NVIC ----------------------------------------*/
	{
		NVIC_InitTypeDef         nvic;

    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority =0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	}
}

void TIM6_Start(void)
{
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
	
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}

void TIM6_DAC_IRQHandler(void)  
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
		TIM_ClearFlag(TIM6, TIM_FLAG_Update);
		controlTask();//控制环
	}
}


