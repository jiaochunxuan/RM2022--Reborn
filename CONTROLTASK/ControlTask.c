#include "ControlTask.h"
#include "MotorTask.h"
#include "PIDregulation.h"
#include "kalman.h"
#include "EncoderTask.h"
#include "PIDregulation.h"
#include "RM_Cilent_UI.h"

/*变量定义*/
uint32_t time_tick_1ms = 0;
uint32_t R_Flag_time_tick_1ms = 0;//用于记录R键长按时间
uint32_t Shift_Flag_time_tick_1ms = 0;//用于记录Shift键长按时间
uint32_t F_Flag_time_tick_1ms = 0;//用于记录F键长按时间
extern Encoder TM6Encoder;			
extern PID_Regulation_t TM6SpeedPID;		
/*函数声明*/
extern void chooseWorkstate(void);
extern void WorkStateSwitchProcess(void);
extern void UI_ReFreshAll(void);
extern PID_Regulation_t CM1SpeedPID;
extern PID_Regulation_t CM2SpeedPID;
extern PID_Regulation_t CM3SpeedPID;
extern PID_Regulation_t CM4SpeedPID;
extern PID_Regulation_t LM1PositionPID;
extern PID_Regulation_t LM2PositionPID;
extern PID_Regulation_t LM1SpeedPID;
extern PID_Regulation_t LM2SpeedPID;
extern PID_Regulation_t TM1SpeedPID;
extern PID_Regulation_t TM2SpeedPID;
extern PID_Regulation_t TM3SpeedPID;
extern PID_Regulation_t TM4SpeedPID;
extern PID_Regulation_t TM5SpeedPID;
extern PID_Regulation_t TM6SpeedPID;
extern PID_Regulation_t TM7SpeedPID;
extern PID_Regulation_t TM8SpeedPID;
extern PID_Regulation_t TM1PositionPID;
extern PID_Regulation_t TM2PositionPID;
extern PID_Regulation_t TM3PositionPID;
extern PID_Regulation_t TM4PositionPID;
extern PID_Regulation_t TM5PositionPID;
extern PID_Regulation_t TM6PositionPID;
extern PID_Regulation_t TM7PositionPID;
extern PID_Regulation_t TM8PositionPID;

extern float LM_Ref;
extern float TM_Ref_0;//上方手爪
extern float TM_Ref_1;//下方手爪
extern float TM_Ref_2;//2006 摄像头
extern float TM_Ref_3;//前传送带
extern float TM_Ref_4;//后传送带
extern float TM_Ref_5;//取障碍块
extern uint8_t R_Flag;
extern uint8_t SHIFT_Flag;
extern uint8_t F_Flag;
/**
函数：Control_Task()
功能：以1000HZ执行该函数，更新各电机工作状态,无返回值
**/
void controlTask(void)
{
	
	if(R_Flag==1)
	{
		R_Flag_time_tick_1ms++;
	}
	else
	{
		R_Flag_time_tick_1ms=0;
	}
	if(SHIFT_Flag==1)
	{
		Shift_Flag_time_tick_1ms++;
	}
	else
	{
		Shift_Flag_time_tick_1ms=0;
	}
	if(F_Flag==1)
	{
		F_Flag_time_tick_1ms++;
	}
	else
	{
		F_Flag_time_tick_1ms=0;
	}
	//uart_test();
	time_tick_1ms++;
	chooseWorkstate();
	WorkStateSwitchProcess();
	motorParameterSettingTask();
	MotorExecution();
	
//	if(time_tick_1ms%1000==0)
//	{
//		UI_ReFreshAll();
//	}
//	if(time_tick_1ms%100==0)
//	{
//		UI_ReFreshAll();
//	}
	
}

void ControtLoopTaskInit(void)
{
	time_tick_1ms = 0; 
	R_Flag_time_tick_1ms = 0;
	LM_Ref=0;
	TM_Ref_0=0;
	TM_Ref_1=0;
	TM_Ref_2=0;
	TM_Ref_3=0;
	TM_Ref_4=0;
	TM_Ref_5=0;
	PID_Reset(&CM1SpeedPID);
	PID_Reset(&CM2SpeedPID);
	PID_Reset(&CM3SpeedPID);
	PID_Reset(&CM4SpeedPID);
	PID_Reset(&LM1PositionPID);
	PID_Reset(&LM2PositionPID);
	PID_Reset(&LM1SpeedPID);
	PID_Reset(&LM2SpeedPID);
	PID_Reset(&TM1PositionPID);
	PID_Reset(&TM2PositionPID);
	PID_Reset(&TM1SpeedPID);
	PID_Reset(&TM2SpeedPID);
	PID_Reset(&TM3PositionPID);
	PID_Reset(&TM4PositionPID);
	PID_Reset(&TM5PositionPID);
	PID_Reset(&TM6PositionPID);
	PID_Reset(&TM7PositionPID);
	PID_Reset(&TM8PositionPID);
	PID_Reset(&TM3SpeedPID);
	PID_Reset(&TM4SpeedPID);
	PID_Reset(&TM5SpeedPID);
	PID_Reset(&TM6SpeedPID);
	PID_Reset(&TM7SpeedPID);
	PID_Reset(&TM8SpeedPID);
	TM1SpeedPID.kp=3.5;
	TM2SpeedPID.kp=3.5;
}
