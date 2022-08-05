//#include "main.h"
#include "stdio.h"
#include "RemoteTask.h"
#include "RightSwitch.h"
#include "DataProcessing.h"
/*��������*/
ChassisSpeed_Ref_t ChassisSpeedRef;
RC_Ctl_t RC_CtrlData;


//RampGen_t MouseXSpeedRamp = RAMP_GEN_DAFAULT; 
//RampGen_t MouseYSpeedRamp = RAMP_GEN_DAFAULT; 
//RampGen_t LRSpeedRamp  = RAMP_GEN_DAFAULT;   //mouse�����ƶ�б��
//RampGen_t FBSpeedRamp  = RAMP_GEN_DAFAULT;   //mouseǰ���ƶ�б��
//RampGen_t RoSpeedRamp  = RAMP_GEN_DAFAULT;	 //q e������ת
/*����ʵ��*/


/**
������RemoteDataProcess(uint8_t *pData)
���ܣ���ң�����źŽ��д���
**/
void remoteDataProcess(uint8_t *pData)
{
//	remote_micrsecond.time_last =Get_Time_Micros();
	if(pData == NULL)
	{
			return;
	}
	//ch0~ch3:max=1684,min=364,|error|=660
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; //ң����ͨ��0����������ƽ��
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;//ң����ͨ��1������ǰ������
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;//ң����ͨ��2��������ת
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;//ң����ͨ��3�����Ƹ���
	
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;//ң������߿��أ���3����λ��ң��������ģʽ�����ã���� RemoteShootControl
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);//ң�����ұ߿��أ���3����λ������Ϊǿ��ֹͣ������Ϊң�������ƣ��м�Ϊ���̿���

	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);//�������
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);//�������
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    //û�õ�

	RC_CtrlData.mouse.press_l = pData[12];//������
	RC_CtrlData.mouse.press_r = pData[13];//����Ҽ�

	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);//ÿһλ��Ӧһ������
	
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
		
		setRightSwitchMode(&RC_CtrlData.rc);//���ң�����ұ߿��أ�ѡ��ģʽ
		setLeftSwitchMode(&RC_CtrlData.rc);//���ң������߿���

		switch(getRightSwitchMode())
	{
		case REMOTE_INPUT:
		{
			remoteDataProcessing(&(RC_CtrlData.rc));//ң��������ģʽ
		}break;
		case KEY_MOUSE_INPUT:
		{
			mouseKeyDataProcessing(&RC_CtrlData.mouse,&RC_CtrlData.key);//�������ģʽ
		}break;
		case STOP_INPUT:
		{
//			//����ͣ��
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
