#ifndef __TIMER_H__
#define __TIMER_H__
#include "main.h"
#include <stm32f4xx.h>
/*TIM6��ʼ�����������жϷ���������*/
void TIM6_Configuration(void);
void TIM6_Start(void);
void TIM6_DAC_IRQHandler(void);

#endif
