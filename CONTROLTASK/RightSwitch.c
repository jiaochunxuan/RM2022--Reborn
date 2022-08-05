#include "RemoteTask.h"
#include "RightSwitch.h"
/*��������*/
extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern RC_Ctl_t RC_CtrlData;
RightSwitchMode_e RightSwitchMode = STOP_INPUT;   //����ģʽ�趨
LeftSwitchMode_e LeftSwitchMode = GDG_INPUT;
int left_flag; //���ڼ�¼��צ����ȥ Ϊ0���� Ϊ1����
/*����ʵ��*/
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
//		GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�պ�����צ
	}
	else if(rc->s1 == 2)
	{
//		left_flag=0;
		LeftSwitchMode=DOWN_INPUT;
//		GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
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

