#include "timer.h"
#include "ControlTask.h"
Time_Count system_micrsecond	 		=	TIME_COUNT_INIT;   //系统时间 单位us
Time_Count shot_frequency_limt 		= TIME_COUNT_INIT;   //发射机构频率控制时间 单位us
Time_Count Yaw_Correction 		= TIME_COUNT_INIT;
Time_Count Remote_microsecond     = TIME_COUNT_INIT;   //测试状态向准备状态切换控制时间
Time_Count usart1_microsecond     = TIME_COUNT_INIT;   //妙算相邻两次发数时间差
Time_Count mpu6050_micrsecond 		= TIME_COUNT_INIT;   //mpu6050监控系统时间 单位us
Time_Count flag1_micrsecond 		= TIME_COUNT_INIT;   //mpu6050监控系统时间 单位us
Time_Count flag2_micrsecond 		= TIME_COUNT_INIT;   //mpu6050监控系统时间 单位us
Time_Count flag3_micrsecond 		= TIME_COUNT_INIT;   //mpu6050监控系统时间 单位us
Time_Count flag4_micrsecond 		= TIME_COUNT_INIT;   //mpu6050监控系统时间 单位us
Time_Count flag5_micrsecond_kj 		= TIME_COUNT_INIT;   //mpu6050监控系统时间 单位us
Time_Count flag6_micrsecond_qd1 		= TIME_COUNT_INIT;   //mpu6050监控系统时间 单位us
Time_Count flag6_micrsecond_qd2 		= TIME_COUNT_INIT;   //mpu6050监控系统时间 单位us
Time_Count flag_move 		= TIME_COUNT_INIT;   //mpu6050监控系统时间 单位us
Time_Count flag_obtain 		= TIME_COUNT_INIT;   //mpu6050监控系统时间 单位us
int time_testtt=0;
/***
函数：TIM2_Configuration()
功能：使能TIM2，作为系统时钟
备注：无
***/
void TIM2_Configuration(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	/* -------------- Configure TIM2 ----------------------------------------*/
  {
		TIM_TimeBaseInitTypeDef tim;
    
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 84 - 1;	 //1M 的时钟 ;1us记一个数
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM2, &tim);
		
		TIM_ARRPreloadConfig(TIM2,ENABLE);
    TIM_Cmd(TIM2,ENABLE);	
	}
}
   
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	}
} 

uint32_t Get_Time_Micros(void)
{
	return TIM2->CNT;
}


