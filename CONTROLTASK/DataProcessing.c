#include "RemoteTask.h"
#include "DataProcessing.h"
#include "Workstate.h"
#include "key.h"
#include "PIDregulation.h"
/*���ݴ����ļ���ң���豸���������ݽ��д���ת��ΪҪִ�еĶ�����־����״̬���ڵ��ִ�к����н�������־��״̬ת��Ϊ����������*/
/*��������*/
extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern MiningTaskStatus miningTaskStatus;
extern Workstate workstate;
extern float Eular[2];//��¼��������YAW��Ƕȣ�������������ת90��
extern int Direction_switching_sign;//���̿��Ƶ��̷����л���־����Ϊ0ʱͼ��Ϊǰ����Ϊ1ʱצ��Ϊ��ǰ��
extern int jiaodu;
/*��������*/
/*�з��Ŷ�������λ����ʵ��*/
int16_t VAL_LIMIT(int16_t val,int16_t min,int16_t max)
{
	if(val<=min)
	{
		val = min;
	}
	else if(val>=max)
	{
		val = max;
	}
	return val;
}
/**
������RemoteDataProcessing(Remote *rc)
���ܣ�ң�������ݸ���,����������ChassisSpeedRef�ṹ���У��ڵ���������ִ��ݵ�����Ľṹ���У�������������
**/

void remoteDataProcessing(Remote *rc)
{
	if(getWorkstate()!=PREPARE_STATE )//�ֱ�Ϊǰ�����ҡ���������ת
	{
		   ChassisSpeedRef.forward_back_ref   = -(rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT;
		   ChassisSpeedRef.left_right_ref     = -(rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_LR_REF_FACT; 
		   ChassisSpeedRef.rotate_ref       =  (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT;	
	}
	/*��λ*/
//	ChassisSpeedRef.forward_back_ref = VAL_LIMIT(ChassisSpeedRef.forward_back_ref,-300,300);//�����޸ģ�
//	ChassisSpeedRef.left_right_ref = VAL_LIMIT(ChassisSpeedRef.left_right_ref,-170,170);
}


/*���󲿷�*/
/*��������*/
uint8_t W_Flag=0;
uint8_t S_Flag=0;
uint8_t A_Flag=0;
uint8_t D_Flag=0;
uint8_t Q_Flag=0;
uint8_t E_Flag=0;
uint8_t SHIFT_Flag=0;
uint8_t CTRL_Flag=0;
uint8_t R_Flag=0;
uint8_t F_Flag=0;
uint8_t G_Flag=0;
uint8_t Z_Flag=0;
uint8_t X_Flag=0;
uint8_t C_Flag=0;
uint8_t V_Flag=0;
uint8_t B_Flag=0;
uint8_t Mouse_l_Flag=0;
uint8_t Mouse_r_Flag=0;

uint16_t forward_back_speed = HIGH_FORWARD_BACK_SPEED ;
uint16_t left_right_speed = HIGH_LEFT_RIGHT_SPEED;
uint16_t rotate_speed = HIGH_ROTATE_SPEED;
float LM_Ref=0;
float TM_Ref_0=0;
float TM_Ref_1=0;
float TM_Ref_2=0;
float TM_Ref_3=0;
float TM_Ref_4=0;
float TM_Ref_5=0;
float TM_Ref_6=0;

/**
������MouseKeyDataProcessing(Mouse *mouse, Key *key)
���ܣ��������ݸ���
�������ã�WASD���Ƶ���ǰ�����ң����������
**/
void mouseKeyDataProcessing(Mouse *mouse, Key *key)
{

	if(getWorkstate()!=PREPARE_STATE)
	{
		if(Direction_switching_sign==0)//ͼ������Ϊ��ǰ��
		{
						/*WASD���Ƶ���*/
					if(key->v & Key_W)//W:ǰ��
					{
		//			W_Flag=1;
		//			S_Flag=0;
						//forward_back_speed += (forward_back_speed < 700 ? 50 * (700 - forward_back_speed)/700:0);
						ChassisSpeedRef.forward_back_ref = -(forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));

					}
					else if(key->v & Key_S)//S:����
					{
		//			W_Flag=0;
		//			S_Flag=1;
						//ChassisSpeedRef.left_right_ref = (left_right_speed );//* LRSpeedRamp.Calc(&LRSpeedRamp);
						//forward_back_speed += (forward_back_speed < 700 ? 300 * (700 - forward_back_speed)/700:0);
						ChassisSpeedRef.forward_back_ref = (forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));
					}
					else
					{
						ChassisSpeedRef.forward_back_ref = 0;
					}
					if(key->v & Key_A)//A:����
					{
		//			A_Flag=1;
		//			D_Flag=0;
						//ChassisSpeedRef.forward_back_ref = -(forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));
						//left_right_speed += (left_right_speed < 700 ? 300 * (700 - left_right_speed)/700:0);
						ChassisSpeedRef.left_right_ref = (left_right_speed );//* LRSpeedRamp.Calc(&LRSpeedRamp);
					}
					else if(key->v & Key_D)//D:����
					{
						//ChassisSpeedRef.forward_back_ref = (forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));
						//left_right_speed += (left_right_speed < 700 ? 300 * (700 - left_right_speed)/700:0);
						ChassisSpeedRef.left_right_ref = -(left_right_speed );//* LRSpeedRamp.Calc(&LRSpeedRamp);
					}
					else
					{
						ChassisSpeedRef.left_right_ref = 0;
					}
		}
		else//צ��Ϊ��ǰ��
		{
								/*WASD���Ƶ���*/
					if(key->v & Key_W)//W:ǰ��
					{
		//			W_Flag=1;
		//			S_Flag=0;
						//left_right_speed += (left_right_speed < 700 ? 300 * (700 - left_right_speed)/700:0);
						ChassisSpeedRef.left_right_ref = -(left_right_speed );//* LRSpeedRamp.Calc(&LRSpeedRamp);
					}
					else if(key->v & Key_S)//S:����
					{
		//			W_Flag=0;
		//			S_Flag=1;
						//left_right_speed += (left_right_speed < 700 ? 300 * (700 - left_right_speed)/700:0);
						ChassisSpeedRef.left_right_ref = (left_right_speed );//* LRSpeedRamp.Calc(&LRSpeedRamp);
					}
					else
					{
						ChassisSpeedRef.left_right_ref = 0;
					}
					if(key->v & Key_A)//A:����
					{
		//			A_Flag=1;
		//			D_Flag=0;
						//forward_back_speed += (forward_back_speed < 700 ? 300 * (700 - forward_back_speed)/700:0);
						ChassisSpeedRef.forward_back_ref = -(forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));
					}
					else if(key->v & Key_D)//D:����
					{
						//forward_back_speed += (forward_back_speed < 700 ? 300 * (700 - forward_back_speed)/700:0);
						ChassisSpeedRef.forward_back_ref = (forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));
					}
					else
					{
						ChassisSpeedRef.forward_back_ref = 0;
					}
		}
			if(key->v & Key_Q)  // Q:����
			{
//			Q_Flag=1;
//			E_Flag=0;
				ChassisSpeedRef.rotate_ref = -rotate_speed;//* RoSpeedRamp.Calc(&RoSpeedRamp);
			}
			else if(key->v & Key_E) //E:����
			{
//				Q_Flag=0;
//				E_Flag=1;
				ChassisSpeedRef.rotate_ref = rotate_speed;//* RoSpeedRamp.Calc(&RoSpeedRamp);
			}
			else if(mouse->x != 0) //������ת��
			{
				ChassisSpeedRef.rotate_ref = 1.8*mouse->x;//��Ҫ��ϵ�������޸ģ�
//			RoSpeedRamp.ResetCounter(&RoSpeedRamp);
			}
			else
			{
				ChassisSpeedRef.rotate_ref=0;
			}
	}
//G������ɿ�״̬
			if(key->v & Key_G)
			{
				G_Flag=1;
			}
//B�����ɿ�װ�ó�ʼ��
			else if(key->v & Key_B)
			{
				B_Flag=1;
			}
//R������ҿ�״̬
			else if(key->v & Key_R)
			{
				R_Flag=1;
			}
//F������ץȡ���п�ʯ���ŵ�̨��״̬
			else if(key->v & Key_F)
			{
				F_Flag=1;
			}
//V������������״̬���г�ʼ����������STOP�ĵ��������Իָ������Ǽ�ͣ
			else if(key->v & Key_V)
			{
				V_Flag=1;
			}
//SHIFT�����������ʯ�һ�״̬��
			else if(key->v & Key_SHIFT)
			{
				SHIFT_Flag=1;
				if(key->v & Key_C)
			{
				C_Flag=1;
			}
			}
//����
			else if(key->v & Key_C)
			{
				C_Flag=1;
			}
			else
			{
				G_Flag=0;
				B_Flag=0;
				R_Flag=0;
				F_Flag=0;
				V_Flag=0;
				C_Flag=0;
				SHIFT_Flag=0;
			}
			
////cȡ�ϰ����л�
//			uint8_t keyvalue_C;//�����ֵX
//			keyvalue_C=KEY_Scan_C(0,key);
//			if(keyvalue_C)
//			{
//					C_Flag=!C_Flag;
//			}

//X��Ԯ�л�
			uint8_t keyvalue_X;//�����ֵX
			keyvalue_X=KEY_Scan_X(0,key);
			if(keyvalue_X)
			{
					X_Flag=!X_Flag;
			}
//Zͼ���л�
			uint8_t keyvalue_Z;//�����ֵZ
			keyvalue_Z=KEY_Scan_Z(0,key);
			if(keyvalue_Z)
			{
					Z_Flag=!Z_Flag;
			}
			
//CTRL�����ƴ�С��Դ���ɿ�ģʽ�л� 	CTRL_FlagΪ0ʱΪС��Դ��ģʽ��CTRL_FlagΪ1ʱΪ����Դ��ģʽ
			uint8_t keyvalue_ctrl;//�����ֵctrl
			keyvalue_ctrl=KEY_Scan_ctrl(0,key);
			if((keyvalue_ctrl) && (miningTaskStatus==MINING_NOTASK_STATE))
			{
					CTRL_Flag=!CTRL_Flag;
			}
			
////C����ת90��
//			uint8_t keyvalue_C;//�����ֵZ
//			keyvalue_C=KEY_Scan_C(0,key);
//			if(keyvalue_C)
//			{
//				C_Flag=1;//�ô˱�־�жϽ�����ת��ʮ��ģʽ��ת��90���˱�־����
//			}
			
//��������־
if(mouse->press_l==0)
{
	Mouse_l_Flag=0;
}	
else if(mouse->press_l==1)
{
	Mouse_l_Flag=1;
}
//����Ҽ���־
if(mouse->press_r==0)
{
	Mouse_r_Flag=0;
}	
else if(mouse->press_r==1)
{
	Mouse_r_Flag=1;
}
//������¿���ͼ��
if(mouse->y < 0)
{
	if(jiaodu<9300)
	{
	jiaodu+=2;
	}	
	else
	{
		jiaodu=9300;
	}
}
else if(mouse->y > 0)
{
	if(jiaodu>9050)
	{
	jiaodu-=2;
	}
	else
	{
		jiaodu=9050;
	}
	}
	
}
