#include "PWM.h"
#include "stm32f4xx.h"

/*PA2 TIM5 3通道 PB0 TIM3 3通道 图传转向*/
/*函数实现*/
void PWM_PD12_Configuration(void)
{
	  PWM_Set PD12;
		PD12.GPIOx						=	GPIOD;//GPIOx，把x改为要用的
		PD12.GPIO_Pin_x			=	GPIO_Pin_12;//GPIO_Pin_x，把x改为要用的
		PD12.GPIO_AF					=	GPIO_AF_TIM4;//GPIO_AF_TIMx，把x改为要用的
		PD12.TIMx						=	TIM4;//TIMx，把x改为要用的
		PD12.Chx							=	1;//CHx，把x改为要用的
		PD12.Period					=	10000;//period
	  PD12.TIM_Pulse       = 9150;
		PD12.Prescaler				=	168-1;//Prescaler
		PWM_Configuration_AUTO(&PD12);//P波产生
	  TIM_SetCompare1(TIM4,9150);
}

//void PWM_PD13_Configuration(void)//左右转动
//{
//	  PWM_Set PD13;
//		PD13.GPIOx						=	GPIOD;//GPIOx，把x改为要用的
//		PD13.GPIO_Pin_x			=	GPIO_Pin_13;//GPIO_Pin_x，把x改为要用的
//		PD13.GPIO_AF					=	GPIO_AF_TIM4;//GPIO_AF_TIMx，把x改为要用的
//		PD13.TIMx						=	TIM4;//TIMx，把x改为要用的
//		PD13.Chx							=	2;//CHx，把x改为要用的
//		PD13.Period					=	10000;//period
//	  PD13.TIM_Pulse       = 9762;
//		PD13.Prescaler				=	168-1;//Prescaler
//		PWM_Configuration_AUTO(&PD13);//P波产生
//	  TIM_SetCompare2(TIM4,9762);
//}

void PWM_PI5_Configuration(void)
{
	  PWM_Set PI5;
		PI5.GPIOx						=	GPIOI;//GPIOx，把x改为要用的
		PI5.GPIO_Pin_x			=	GPIO_Pin_5;//GPIO_Pin_x，把x改为要用的
		PI5.GPIO_AF					=	GPIO_AF_TIM8;//GPIO_AF_TIMx，把x改为要用的
		PI5.TIMx						=	TIM8;//TIMx，把x改为要用的
		PI5.Chx							=	1;//CHx，把x改为要用的
		PI5.Period					=	10000;//period
	  PI5.TIM_Pulse       = 1000;
		PI5.Prescaler				=	168-1;//Prescaler
		PWM_Configuration_AUTO(&PI5);//P波产生
	  TIM_SetCompare1(TIM8,1700);
}

void PWM_PI6_Configuration(void)
{
	  PWM_Set PI6;
		PI6.GPIOx						=	GPIOI;//GPIOx，把x改为要用的
		PI6.GPIO_Pin_x			=	GPIO_Pin_6;//GPIO_Pin_x，把x改为要用的
		PI6.GPIO_AF					=	GPIO_AF_TIM8;//GPIO_AF_TIMx，把x改为要用的
		PI6.TIMx						=	TIM8;//TIMx，把x改为要用的
		PI6.Chx							=	2;//CHx，把x改为要用的
		PI6.Period					=	10000;//period
	  PI6.TIM_Pulse       = 1000;
		PI6.Prescaler				=	168-1;//Prescaler
		PWM_Configuration_AUTO(&PI6);//P波产生
	  TIM_SetCompare2(TIM8,1700);
}

//void PWM_PA0_Configuration(void)
//{
//	  PWM_Set PA0;
//		PA0.GPIOx						=	GPIOA;//GPIOx，把x改为要用的
//		PA0.GPIO_Pin_x			=	GPIO_Pin_0;//GPIO_Pin_x，把x改为要用的
//		PA0.GPIO_AF					=	GPIO_AF_TIM2;//GPIO_AF_TIMx，把x改为要用的
//		PA0.TIMx						=	TIM2;//TIMx，把x改为要用的
//		PA0.Chx							=	1;//CHx，把x改为要用的
//		PA0.Period					=	10000;//period
//	  PA0.TIM_Pulse       = 1000;
//		PA0.Prescaler				=	168-1;//Prescaler
//		PWM_Configuration_AUTO(&PA0);//P波产生
//	  TIM_SetCompare1(TIM2,1200);
//}

//void PWM_PA1_Configuration(void)
//{
//	  PWM_Set PA1;
//		PA1.GPIOx						=	GPIOA;//GPIOx，把x改为要用的
//		PA1.GPIO_Pin_x			=	GPIO_Pin_2;//GPIO_Pin_x，把x改为要用的
//		PA1.GPIO_AF					=	GPIO_AF_TIM2;//GPIO_AF_TIMx，把x改为要用的
//		PA1.TIMx						=	TIM2;//TIMx，把x改为要用的
//		PA1.Chx							=	2;//CHx，把x改为要用的
//		PA1.Period					=	10000;//period
//	  PA1.TIM_Pulse       = 1000;
//		PA1.Prescaler				=	168-1;//Prescaler
//		PWM_Configuration_AUTO(&PA1);//P波产生
//	  TIM_SetCompare2(TIM2,500);
//}

void PWM_Configuration_AUTO(PWM_Set* pwm)
{
		/*
		pwm->GPIO_Pin_x
		pwm->GPIOx
		pwm->GPIO_PinSource
		pwm->GPIO_AF
		pwm->Prescaler
		pwm->Period
		pwm->TIMx
		pwm->TIM_Pulse;  
		pwm->Chx
		*/

	  GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;

		if(pwm->TIMx == TIM4)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
		gpio.GPIO_Mode	= GPIO_Mode_AF;
		gpio.GPIO_OType	= GPIO_OType_PP;
		gpio.GPIO_Pin		= pwm->GPIO_Pin_x;//GPIO_Pin_x
		gpio.GPIO_PuPd	= GPIO_PuPd_UP;
		gpio.GPIO_Speed	= GPIO_Speed_100MHz;
		GPIO_Init(pwm->GPIOx,&gpio);//GPIOx

		Switch_GPIO_PinSource(pwm);
		GPIO_PinAFConfig(pwm->GPIOx,pwm->GPIO_PinSource,pwm->GPIO_AF);//GPIOx 
																																	//GPIO_PinSource 
																																	//GPIO_AF
		
		tim.TIM_Prescaler					=	pwm->Prescaler;//Prescaler
		tim.TIM_CounterMode				=	TIM_CounterMode_Up;
		tim.TIM_Period						= pwm->Period;//Period
		tim.TIM_ClockDivision			=	TIM_CKD_DIV1;
		tim.TIM_RepetitionCounter	=0;
		TIM_TimeBaseInit(pwm->TIMx,&tim);//TIMx
		
		oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Set;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
		oc.TIM_Pulse = pwm->TIM_Pulse;

		//Chx TIMx
		switch (pwm->Chx)
		{
			case 1:
			{
				TIM_OC1Init(pwm->TIMx,&oc);
			}break;
			case 2:
			{
				TIM_OC2Init(pwm->TIMx,&oc);
			}break;
			case 3:
			{
				TIM_OC3Init(pwm->TIMx,&oc);
			}break;
		
			default:
			{
				TIM_OC4Init(pwm->TIMx,&oc);
			}break;
		}
		TIM_CtrlPWMOutputs(pwm->TIMx,ENABLE);
		TIM_ARRPreloadConfig(TIM4,ENABLE);
		TIM_Cmd(pwm->TIMx,ENABLE);//TIMx
	}
	
	if(pwm->TIMx == TIM5)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);
		gpio.GPIO_Mode	= GPIO_Mode_AF;
		gpio.GPIO_OType	= GPIO_OType_PP;
		gpio.GPIO_Pin		= pwm->GPIO_Pin_x;//GPIO_Pin_x
		gpio.GPIO_PuPd	= GPIO_PuPd_UP;
		gpio.GPIO_Speed	= GPIO_Speed_100MHz;
		GPIO_Init(pwm->GPIOx,&gpio);//GPIOx

		Switch_GPIO_PinSource(pwm);
		GPIO_PinAFConfig(pwm->GPIOx,pwm->GPIO_PinSource,pwm->GPIO_AF);//GPIOx 
																																	//GPIO_PinSource 
																																	//GPIO_AF
		
		tim.TIM_Prescaler					=	pwm->Prescaler;//Prescaler
		tim.TIM_CounterMode				=	TIM_CounterMode_Up;
		tim.TIM_Period						= pwm->Period;//Period
		tim.TIM_ClockDivision			=	TIM_CKD_DIV1;
		tim.TIM_RepetitionCounter	=0;
		TIM_TimeBaseInit(pwm->TIMx,&tim);//TIMx
		
		oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Set;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
		oc.TIM_Pulse = pwm->TIM_Pulse;

		//Chx TIMx
		switch (pwm->Chx)
		{
			case 1:
			{
				TIM_OC1Init(pwm->TIMx,&oc);
			}break;
			case 2:
			{
				TIM_OC2Init(pwm->TIMx,&oc);
			}break;
			case 3:
			{
				TIM_OC3Init(pwm->TIMx,&oc);
			}break;
		
			default:
			{
				TIM_OC4Init(pwm->TIMx,&oc);
			}break;
		}
		TIM_CtrlPWMOutputs(pwm->TIMx,ENABLE);
		TIM_Cmd(pwm->TIMx,ENABLE);//TIMx
	}
	
	if(pwm->TIMx == TIM3)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
		gpio.GPIO_Mode	= GPIO_Mode_AF;
		gpio.GPIO_OType	= GPIO_OType_PP;
		gpio.GPIO_Pin		= pwm->GPIO_Pin_x;//GPIO_Pin_x
		gpio.GPIO_PuPd	= GPIO_PuPd_UP;
		gpio.GPIO_Speed	= GPIO_Speed_100MHz;
		GPIO_Init(pwm->GPIOx,&gpio);//GPIOx

		Switch_GPIO_PinSource(pwm);
		GPIO_PinAFConfig(pwm->GPIOx,pwm->GPIO_PinSource,pwm->GPIO_AF);//GPIOx 
																																	//GPIO_PinSource 
																																	//GPIO_AF
		
		tim.TIM_Prescaler					=	pwm->Prescaler;//Prescaler
		tim.TIM_CounterMode				=	TIM_CounterMode_Up;
		tim.TIM_Period						= pwm->Period;//Period
		tim.TIM_ClockDivision			=	TIM_CKD_DIV1;
		tim.TIM_RepetitionCounter	=0;
		TIM_TimeBaseInit(pwm->TIMx,&tim);//TIMx
		
		oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Set;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
		oc.TIM_Pulse = pwm->TIM_Pulse;

		//Chx TIMx
		switch (pwm->Chx)
		{
			case 1:
			{
				TIM_OC1Init(pwm->TIMx,&oc);
			}break;
			case 2:
			{
				TIM_OC2Init(pwm->TIMx,&oc);
			}break;
			case 3:
			{
				TIM_OC3Init(pwm->TIMx,&oc);
			}break;
		
			default:
			{
				TIM_OC4Init(pwm->TIMx,&oc);
			}break;
		}
		TIM_CtrlPWMOutputs(pwm->TIMx,ENABLE);
		TIM_Cmd(pwm->TIMx,ENABLE);//TIMx
	}
	
		if(pwm->TIMx == TIM8)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		gpio.GPIO_Mode	= GPIO_Mode_AF;
		gpio.GPIO_OType	= GPIO_OType_PP;
		gpio.GPIO_Pin		= pwm->GPIO_Pin_x;//GPIO_Pin_x
		gpio.GPIO_PuPd	= GPIO_PuPd_UP;
		gpio.GPIO_Speed	= GPIO_Speed_100MHz;
		GPIO_Init(pwm->GPIOx,&gpio);//GPIOx

		Switch_GPIO_PinSource(pwm);
		GPIO_PinAFConfig(pwm->GPIOx,pwm->GPIO_PinSource,pwm->GPIO_AF);//GPIOx 
																																	//GPIO_PinSource 
																																	//GPIO_AF
		
		tim.TIM_Prescaler					=	pwm->Prescaler;//Prescaler
		tim.TIM_CounterMode				=	TIM_CounterMode_Up;
		tim.TIM_Period						= pwm->Period;//Period
		tim.TIM_ClockDivision			=	TIM_CKD_DIV1;
		tim.TIM_RepetitionCounter	=0;
		TIM_TimeBaseInit(pwm->TIMx,&tim);//TIMx
		
		oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Set;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
		oc.TIM_Pulse = pwm->TIM_Pulse;

		//Chx TIMx
		switch (pwm->Chx)
		{
			case 1:
			{
				TIM_OC1Init(pwm->TIMx,&oc);
			}break;
			case 2:
			{
				TIM_OC2Init(pwm->TIMx,&oc);
			}break;
			case 3:
			{
				TIM_OC3Init(pwm->TIMx,&oc);
			}break;
		
			default:
			{
				TIM_OC4Init(pwm->TIMx,&oc);
			}break;
		}
		TIM_CtrlPWMOutputs(pwm->TIMx,ENABLE);
		TIM_Cmd(pwm->TIMx,ENABLE);//TIMx
	}
}

void Switch_GPIO_PinSource(PWM_Set* pwm)
{
			switch(pwm->GPIO_Pin_x)
		{
			case GPIO_Pin_0: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource0;
			}break;
			case GPIO_Pin_1: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource1;
			}break;
			case GPIO_Pin_2: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource2;
			}break;
			case GPIO_Pin_3: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource3;
			}break;
			case GPIO_Pin_4: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource4;
			}break;
			case GPIO_Pin_5: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource5;
			}break;
			case GPIO_Pin_6: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource6;
			}break;
			case GPIO_Pin_7: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource7;
			}break;
			case GPIO_Pin_8: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource8;
			}break;
			case GPIO_Pin_9: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource9;
			}break;
			case GPIO_Pin_10: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource10;
			}break;
			case GPIO_Pin_11: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource11;
			}break;
			case GPIO_Pin_12: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource12;
			}break;
			case GPIO_Pin_13: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource13;
			}break;
			case GPIO_Pin_14: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource14;
			}break;
			default: 
			{
				pwm->GPIO_PinSource = GPIO_PinSource15;
			}break;
		}
}

