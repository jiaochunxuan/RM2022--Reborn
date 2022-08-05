//Î´¸Ä

#ifndef TIMER_H
#define TIMER_H
#include <stm32f4xx.h>

#define TIME_COUNT_INIT \
{\
	0,\
	0,\
	0,\
	0,\
}\


typedef __packed struct
{
	uint32_t time_last;
	uint32_t time_now;
	int32_t time_error;
	uint8_t flag;
}Time_Count;

extern Time_Count system_micrsecond;
extern Time_Count shot_frequency_limt;
extern Time_Count Yaw_Correction;
extern Time_Count Remote_microsecond;
extern Time_Count usart1_microsecond;//ÒÑ¸Ä
extern Time_Count mpu6050_micrsecond;
extern Time_Count flag1_micrsecond;
extern Time_Count flag2_micrsecond;
extern Time_Count flag3_micrsecond;
extern Time_Count flag4_micrsecond;
extern Time_Count flag5_micrsecond_kj;
extern Time_Count flag6_micrsecond_qd1;
extern Time_Count flag6_micrsecond_qd2;
extern Time_Count flag_move;
extern Time_Count flag_obtain;
void TIM2_Configuration(void);
uint32_t Get_Time_Micros(void);


#endif

