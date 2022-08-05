//#include "main.h"
#include "stdio.h"
#include "RemoteTask.h"
#include "RightSwitch.h"
#include "DataProcessing.h"
/*变量定义*/
ChassisSpeed_Ref_t ChassisSpeedRef;
RC_Ctl_t RC_CtrlData;


//RampGen_t MouseXSpeedRamp = RAMP_GEN_DAFAULT; 
//RampGen_t MouseYSpeedRamp = RAMP_GEN_DAFAULT; 
//RampGen_t LRSpeedRamp  = RAMP_GEN_DAFAULT;   //mouse左右移动斜坡
//RampGen_t FBSpeedRamp  = RAMP_GEN_DAFAULT;   //mouse前后移动斜坡
//RampGen_t RoSpeedRamp  = RAMP_GEN_DAFAULT;	 //q e快速旋转
/*函数实现*/


/**
函数：RemoteDataProcess(uint8_t *pData)
功能：对遥控器信号进行处理
**/
void remoteDataProcess(uint8_t *pData)
{
//	remote_micrsecond.time_last =Get_Time_Micros();
	if(pData == NULL)
	{
			return;
	}
	//ch0~ch3:max=1684,min=364,|error|=660
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; //遥控器通道0，控制左右平移
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;//遥控器通道1，控制前进后退
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;//遥控器通道2，控制旋转
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;//遥控器通道3：控制俯仰
	
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;//遥控器左边开关，有3个挡位，遥控器控制模式下有用，详见 RemoteShootControl
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);//遥控器右边开关，有3个挡位，最下为强制停止，最上为遥控器控制，中间为键盘控制

	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);//鼠标左右
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);//鼠标上下
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    //没用到

	RC_CtrlData.mouse.press_l = pData[12];//鼠标左键
	RC_CtrlData.mouse.press_r = pData[13];//鼠标右键

	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);//每一位对应一个按键
	
/**
0x0001:w 
0x0002:s 
0x0004:a 
0x0008:d 

0x0010:shift 
0x0020:ctrl 
0x0040:q 
0x0080:e 

0x0100:r 
0x0200:f 
0x0400:g 
0x0800:z 

0x1000:x 
0x2000:c 
0x4000:v 
0x8000:b
**/
		
		setRightSwitchMode(&RC_CtrlData.rc);//检测遥控器右边开关，选择模式
		setLeftSwitchMode(&RC_CtrlData.rc);//检测遥控器左边开关

		switch(getRightSwitchMode())
	{
		case REMOTE_INPUT:
		{
			remoteDataProcessing(&(RC_CtrlData.rc));//遥控器控制模式
		}break;
		case KEY_MOUSE_INPUT:
		{
			mouseKeyDataProcessing(&RC_CtrlData.mouse,&RC_CtrlData.key);//键鼠控制模式
		}break;
		case STOP_INPUT:
		{
//			//紧急停车
//			InitFrictionWheel();
//      Set_CM_Speed(CAN1, 0,0,0,0);
//			Set_CM_Speed(CAN2, 0,0,0,0);
//			Set_Gimbal_Current(CAN1, 0,0,0,0);
//			Set_Gimbal_Current(CAN2, 0,0,0,0);
//			
		}break;
	}
	
	
	switch(getLeftSwitchMode())
	{
		case UP_INPUT:
		{
			
		}break;
		case GDG_INPUT:
		{
			
		}break;
		case DOWN_INPUT:
		{
			
		}break;
	}
}
