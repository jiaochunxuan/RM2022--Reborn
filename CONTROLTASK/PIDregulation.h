#ifndef __PID_REGULATION_H__
#define __PID_REGULATION_H__
#include "stm32f4xx.h"
#include "main.h"
/*PID结构体类型定义*/
typedef struct PID_Regulation_t
{
	float ref;
	float fdb;
	float err[2];
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	void (*Calc)(struct PID_Regulation_t *pid);
	void (*Reset)(struct PID_Regulation_t *pid);
	float diedzone;
}PID_Regulation_t;

/*底盘电机初始化结构体*/
//#define CHASSIS_MOTOR_CM_1_SPEED_PID_DEFAULT \
//{\
//	0.0,\
//	0.0,\
//	{0.0,0.0},\
//	15.0f,\
//	10.0f,\
//	1.0f,\
//	0.0,\
//	0.0,\
//	0.0,\
//	11000.0,\
//	340.0,\
//	1500.0,\
//	0.0,\
//	16000.0,\
//	&PID_Calc,\
//	&PID_Reset,\
//	0.0f,\
//}\

#define CHASSIS_MOTOR_CM_1_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	15.0f,\
	10.0f,\
	1.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_CM_2_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	15.0f,\
	10.0f,\
	1.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_CM_3_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	15.0f,\
	10.0f,\
	1.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_CM_4_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	15.0f,\
	10.0f,\
	1.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

//#define CHASSIS_Speed_Limit_PID_DEFAULT \
//{\
//	0.0,\
//	0.0,\
//	{0.0,0.0},\
//	15.0f,\
//	10.0f,\
//	1.0f,\
//	0.0,\
//	0.0,\
//	0.0,\
//	11000.0,\
//	340.0,\
//	1500.0,\
//	0.0,\
//	16000.0,\
//	&PID_Calc,\
//	&PID_Reset,\
//	0.0f,\
//}\

#define CHASSIS_MOTOR_LM_1_POSITION_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	1600.0,\
	3500.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_LM_1_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_LM_2_POSITION_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	1600.0,\
	3500.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_LM_2_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_1_POSITION_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	1600.0,\
	5.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc_1,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_2_POSITION_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	1600.0,\
	5.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc_1,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_3_POSITION_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	1600.0,\
	100.0,\
	100.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_4_POSITION_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	1600.0,\
	100.0,\
	100.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_5_POSITION_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	1600.0,\
	100.0,\
	100.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_6_POSITION_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	1600.0,\
	100.0,\
	100.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_7_POSITION_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	1600.0,\
	100.0,\
	100.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_8_POSITION_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	1600.0,\
	100.0,\
	100.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_1_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_2_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_3_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_4_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_5_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_6_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_7_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

#define CHASSIS_MOTOR_TM_8_SPEED_PID_DEFAULT \
{\
	0.0,\
	0.0,\
	{0.0,0.0},\
	7.0f,\
	0.0f,\
	0.0f,\
	0.0,\
	0.0,\
	0.0,\
	11000.0,\
	340.0,\
	1500.0,\
	0.0,\
	16000.0,\
	&PID_Calc,\
	&PID_Reset,\
	0.0f,\
}\

/*PID函数声明*/
void PID_Reset(PID_Regulation_t *pid);
void PID_Calc(PID_Regulation_t *pid);
void PID_Calc_1(PID_Regulation_t *pid);
#endif
