#include "GPIO.h"
//////////////A板上的可控电源输出口，通过配置PH?管脚，并且置高为输出setbits
//PH2 PH3 PH4 PH5
void GPIO_PH_Init(void)//A板上的可控电源
{		GPIO_InitTypeDef gpio;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 
		
		gpio.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_StructInit(&gpio);
		GPIO_Init(GPIOH, &gpio);
	
		GPIO_ResetBits(GPIOH,GPIO_Pin_5);//下手抓气缸夹紧
		GPIO_ResetBits(GPIOH,GPIO_Pin_3);//抓取气缸张开
		GPIO_ResetBits(GPIOH,GPIO_Pin_4);//推出气缸退回
		GPIO_ResetBits(GPIOH,GPIO_Pin_2);//夹紧气缸松开
}

void GPIO_PF_Init(void)//PF0 输入 用于光电管
{		GPIO_InitTypeDef gpio;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 
		
		gpio.GPIO_Pin = GPIO_Pin_0;
		gpio.GPIO_Mode = GPIO_Mode_IN;
//		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_StructInit(&gpio);
		GPIO_Init(GPIOF, &gpio);
}
void GPIO_PD_Init(void)//PD14\PD15\PD13 输入，检测仓内矿石状态
{		
		GPIO_InitTypeDef gpio;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 
		
		gpio.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_13;
		gpio.GPIO_Mode = GPIO_Mode_IN;
//		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_StructInit(&gpio);
		GPIO_Init(GPIOD, &gpio);
}
void GPIO_PH_Init_1(void)//PH12 Trig端(输出)
{		
		GPIO_InitTypeDef gpio;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 
	
		gpio.GPIO_Pin = GPIO_Pin_12;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
//		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_StructInit(&gpio);
		GPIO_Init(GPIOH, &gpio);
	
//		GPIO_InitTypeDef GPIO_InitStructure;
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
//		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;//输入模式
//		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_Init(GPIOA,&GPIO_InitStructure);
//		GPIO_ResetBits(GPIOA,GPIO_Pin_0);

}
//PE5 机械救援气缸
void GPIO_PE_Init(void)
{		GPIO_InitTypeDef gpio;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 
		
		gpio.GPIO_Pin = GPIO_Pin_5;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_StructInit(&gpio);
		GPIO_Init(GPIOE, &gpio);
	
  	GPIO_SetBits(GPIOE,GPIO_Pin_5);//机械救援收回
}

void GPIO_PE6_Init(void)//刷卡救援气缸
{		GPIO_InitTypeDef gpio;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 
		
		gpio.GPIO_Pin = GPIO_Pin_6;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_StructInit(&gpio);
		GPIO_Init(GPIOE, &gpio);
	
		GPIO_ResetBits(GPIOE,GPIO_Pin_6);//刷卡救援收回
}
