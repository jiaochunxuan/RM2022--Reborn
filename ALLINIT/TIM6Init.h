#ifndef __TIMER_H__
#define __TIMER_H__
#include "main.h"
#include <stm32f4xx.h>
/*TIM6初始化、启动、中断服务函数声明*/
void TIM6_Configuration(void);
void TIM6_Start(void);
void TIM6_DAC_IRQHandler(void);

#endif
