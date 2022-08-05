#include "RemoteTask.h"
#include "RightSwitch.h"
/*变量定义*/
extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern RC_Ctl_t RC_CtrlData;
RightSwitchMode_e RightSwitchMode = STOP_INPUT;   //输入模式设定
LeftSwitchMode_e LeftSwitchMode = GDG_INPUT;
int left_flag; //用于检录手爪翻下去 为0不翻 为1翻下
/*函数实现*/
void setRightSwitchMode(Remote *rc)
{
	if(rc->s2 == 1)
	{
		RightSwitchMode = REMOTE_INPUT;
	}
	else if(rc->s2 == 3)
	{
		RightSwitchMode = KEY_MOUSE_INPUT;
	}
	else if(rc->s2 == 2)
	{
		RightSwitchMode = STOP_INPUT;
	}	
}

void setLeftSwitchMode(Remote *rc)
{
	if(rc->s1 == 1)
	{
		LeftSwitchMode = UP_INPUT;
	}
	else if(rc->s1 == 3)
	{
//		left_flag=1;
		LeftSwitchMode=GDG_INPUT;
//		GPIO_ResetBits(GPIOH,GPIO_Pin_5);//闭合下手爪
	}
	else if(rc->s1 == 2)
	{
//		left_flag=0;
		LeftSwitchMode=DOWN_INPUT;
//		GPIO_SetBits(GPIOH,GPIO_Pin_5);//张开下手爪
	}
	
	
}

RightSwitchMode_e getRightSwitchMode(void)
{
	return RightSwitchMode;
}

LeftSwitchMode_e getLeftSwitchMode(void)
{
	return LeftSwitchMode;
}

