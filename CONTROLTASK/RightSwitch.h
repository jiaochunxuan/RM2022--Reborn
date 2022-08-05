#ifndef __RIGHTSWITCH_H__
#define __RIGHTSWITCH_H__
#include "main.h"
#include "RemoteTask.h"
/*遥控器右侧开关相关声明*/
/*变量声明*/
typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP_INPUT = 2,
}RightSwitchMode_e;

typedef enum
{
	UP_INPUT = 1,
	GDG_INPUT=3,
	DOWN_INPUT=2,
}LeftSwitchMode_e;

static RightSwitchMode_e RightSwitchMode;
static LeftSwitchMode_e LeftSwitchMode;
/*函数声明*/
void setRightSwitchMode(Remote *rc);
void setLeftSwitchMode(Remote *rc);
RightSwitchMode_e getRightSwitchMode(void);
LeftSwitchMode_e getLeftSwitchMode(void);

#endif
