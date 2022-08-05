#include "MotorTask.h"
#include "Workstate.h"
#include "PIDregulation.h"
#include "RemoteTask.h"
#include "EncoderTask.h"
#include "delay.h"
#include "DataProcessing.h"
#include "kalman.h"
#include "timer.h"
/*��������*/
float height_s=1000;//С��Դ���߶ȣ����ߣ�
float height_s_p=1040;//С��Դ��ȡ���߶ȣ��������ޣ���Ҫ�Ķ���
float height_b=100;//����Դ���߶ȣ����ͣ�
float height_b_p=350;//����Դ��ȡ���߶�
float put_height_first=800;//��һ������ȡ�Ŀ�ʯ�һ�վ�߶�
float put_height_second=950;//������ȡ�Ŀ�ʯ�һ�վ�߶�
float claw_out_angel=180;//צ�ӷ������սǶ�,����ȡ��Ƕ�,���ڱ��Ƕ�


/*��������*/
extern Workstate workstate;
extern Workstate lastWorkstate;
extern MiningTaskStatus miningTaskStatus;
extern MiningTaskStatus lastminingTaskStatus;
extern RescueTaskStatus rescueTaskStatus;
extern BarrierTaskStatus barrierTaskStatus;
extern uint8_t G_Flag;
extern uint8_t B_Flag;
extern uint8_t C_Flag;
extern uint8_t X_Flag;
extern uint8_t R_Flag;
extern uint8_t F_Flag;
extern uint8_t V_Flag;
extern uint8_t Z_Flag;
extern uint8_t SHIFT_Flag;
extern uint8_t CTRL_Flag;
extern uint8_t Mouse_l_Flag;
extern uint8_t Mouse_r_Flag;
extern PID_Regulation_t CM1SpeedPID;
extern PID_Regulation_t CM2SpeedPID;
extern PID_Regulation_t CM3SpeedPID;
extern PID_Regulation_t CM4SpeedPID;
//extern PID_Regulation_t Speed_Limit;
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

extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder LM1Encoder;
extern volatile Encoder LM2Encoder;
extern volatile Encoder TM1Encoder;
extern volatile Encoder TM2Encoder;
extern volatile Encoder TM3Encoder;
extern volatile Encoder TM4Encoder;
extern volatile Encoder TM5Encoder;
extern volatile Encoder TM6Encoder;
extern volatile Encoder TM7Encoder;
extern volatile Encoder TM8Encoder;
extern float LM_Ref;
extern float TM_Ref_0;//�Ϸ���צ
extern float TM_Ref_1;//�·���צ
extern float TM_Ref_2;//2006 ����ͷ
extern float TM_Ref_3;//ǰ���ʹ�
extern float TM_Ref_4;//���ʹ�
extern float TM_Ref_5;//ȡ�ϰ���

extern MiningTaskStatus miningTaskStatus;
extern BarrierTaskStatus barrierTaskStatus;
extern RescueTaskStatus rescueTaskStatus;
extern ObstacleTaskStatus obstacleTaskStatus;
float testCompensationvalue=0;//�����÷�ת��������ֵ
float flipCompensation;//��ת��������ֵ
float flipCompensation_Plus;//�²���ת��������ֵ
float upliftCompensation_1;//̧�������������ֵ
float upliftCompensation_2;//̧�������������ֵ
float TMfdb_error;//�����צ�Ƿ񱻶�ת������¼����
extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern uint16_t rotate_speed;
extern uint16_t forward_back_speed;
extern uint16_t left_right_speed;
extern uint16_t rotate_speed;
extern uint32_t time_tick_1ms;
extern uint32_t R_Flag_time_tick_1ms;//��¼��R������ʱ�䳤��
extern uint32_t Shift_Flag_time_tick_1ms;//��¼��Shift������ʱ�䳤��
extern uint32_t F_Flag_time_tick_1ms;//��¼��F������ʱ�䳤��
uint32_t motor_time_delay=0;//��ȡת���õ��������ʱ��¼ֵ
extern int jiaodu=9150;
extern int csd_speed=1200;

/*��������*/
extern float VAL_LIMITF(float val,float min,float max);
extern int16_t VAL_LIMIT(int16_t val,int16_t min,int16_t max);
void motorstop(void);//��ͣ״̬����
void motorprepare(void);//׼��״̬����
void motornormal(void);//δ����
void chassis_system(void);//���̺���
void move_system(void);//ȡ�ϰ��麯��
void rescue_system(void);//��Ԯ����
void camera_system(void);//����ͷ����
void claw_compensation(void);//��צ��������
void uplift_compensation(void);//̧����������
void Position_set(float position,PositionMode mode);//̧���½���������
void Claw_Position_set(float position,ClawPositionMode mode);//��צ��������
void Position_set_slow(float position,PositionMode mode);//��צ��������
void Claw_Position_set_slow(float position,ClawPositionMode mode);//��צ�������������٣�
void Claw_Position_set_slow2(float position,ClawPositionMode mode);//��צ�������������٣�����
void Claw_Position_set_ground(float position,ClawPositionMode mode);//ȡ����צ
void Claw_Position_set_ground_slow2(float position,ClawPositionMode mode);//�Ե���צ��������
//void Convey1_Speed_set(float speed,ConveyPositionMode mode);//ǰ���ʹ���������
//void Convey2_Speed_set(float speed,ConveyPositionMode mode);//���ʹ���������

int GPIOD14=0;
int GPIOD15=0;
int GPIOD13=0;
int GPIOH2=0;
int GPIOH3=0;
int GPIOH4=0;
int GPIOH5=0;
int GPIOF0=0;
int GPIOE4=0;
int GPIOE5=0;
int GPIOE6=0;
int bbb=0;
int aaa=0;
int ccc=0;
int ddd=0;
int eee=0;
int flag_ob = 0,flag_kj_ts = 0,flag_kj_sz = 0,flag_qk3=0;
float YAW_angle_initial;//��¼��תYAW���ʼ�Ƕ�
float YAW_angle_target;//��¼��תYAW��Ŀ��Ƕ�
int count_target;//Ŀ��Ȧ��
//extern float Eular[2];//ʵʱ������YAW��Ƕ�
extern float Angles;//ʵʱ������YAW��Ƕ�
extern float last;//�ϴ������ǽǶ�
extern float now;//��ǰ�����ǽǶ�
extern int count11;//��ǰȦ��
extern int left_flag;//ң������߿��� ����Ϊ0 ����Ϊ1
int flag_ex = 0;
int Direction_switching_sign=0;//�����л���־����Ϊ0ʱͼ��Ϊ��ǰ����Ϊ1ʱצ�ӷ���Ϊ��ǰ����
extern kalman p2,p1,p_LM1,p_LM2;
float LM1_Data,LM2_Data; 
int gl;
int pre=0;
int flag_qudi=0;
int move_speed;
/*����ʵ��*/
//��fdb�ж��з��գ������ִﲻ��ֵ�Ῠס
//LM2�Ǻ��̧����� ����ֵ��׼ȷ ����Ҳ������LM2����ֵ�ж� ̧��ʱLM2��ת ����ֵΪ��
//צ�ӷ���ʱTM1��ת������ֵΪ��
//����׼��ģʽ���л�Ϊ����Դ��ģʽ��Ŀǰ����Դ��ģʽ��bug
void motorParameterSettingTask()
{
	switch(workstate)
	{
		case STOP_STATE:
		{//��ͣ״̬

			miningTaskStatus=MINING_NOTASK_STATE;
			barrierTaskStatus=BARRIER_NOTASK_STATE;
//			rescueTaskStatus=RESCUE_NOTASK_STATE;
//			obstacleTaskStatus=OBSTACLE_NOTASK_STATE;
			X_Flag=0;
			Z_Flag=0;
			C_Flag=0;
			GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н����ײ��ӽ�
      GPIO_ResetBits(GPIOE,GPIO_Pin_6);//ˢ����Ԯ�����ջ�
			GPIO_SetBits(GPIOE,GPIO_Pin_5);//��е��Ԯ�����ջ�
			GPIO_ResetBits(GPIOH,GPIO_Pin_3);//ץȡ�����ſ�
			GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�Ƴ������˻�
			GPIO_ResetBits(GPIOH,GPIO_Pin_5);//����ץ�պ�	
			motorstop();//��ͣ
			pre=0;
			
//			aaa=0;
		}break;
		
		case UP_STATE:
		{
					{
					TM1SpeedPID.kp=2.0f;
					TM1SpeedPID.ki=0;
					TM1SpeedPID.kd=0;
					TM1PositionPID.kp=5.0f;
					TM1PositionPID.ki=0;
					TM1PositionPID.kd=0;
					TM2SpeedPID.kp=2.0f;
					TM2SpeedPID.ki=0;
					TM2SpeedPID.kd=0;
					TM2PositionPID.kp=5.0f;
					TM2PositionPID.ki=0;
					TM2PositionPID.kd=0;
					}
			{//��ת���
			TM1PositionPID.ref = -TM_Ref_0;
			TM2PositionPID.ref = TM_Ref_0;
			TM1PositionPID.fdb = TM1Encoder.ecd_angle;
			TM2PositionPID.fdb = TM2Encoder.ecd_angle;
			TM1PositionPID.Calc(&TM1PositionPID);
			TM2PositionPID.Calc(&TM2PositionPID);
			TM1SpeedPID.ref = TM1PositionPID.output*5;
			TM2SpeedPID.ref = TM2PositionPID.output*5;
			TM1SpeedPID.fdb = TM1Encoder.filter_rate;
			TM2SpeedPID.fdb = TM2Encoder.filter_rate;
			TM1SpeedPID.Calc(&TM1SpeedPID);
			TM2SpeedPID.Calc(&TM2SpeedPID);
			TM1SpeedPID.output -=flipCompensation;
			TM2SpeedPID.output +=flipCompensation;
			TM1SpeedPID.output = VAL_LIMITF(TM1SpeedPID.output,-5000,5000);
			TM2SpeedPID.output = VAL_LIMITF(TM2SpeedPID.output,-5000,5000);
		}

			Claw_Position_set_slow2(-270,back);
			pre=0;
			
		}
		case DOWN_STATE:
		{
					{
					TM5SpeedPID.kp=2.5f;
					TM5SpeedPID.ki=0;
					TM5SpeedPID.kd=0;
					TM5PositionPID.kp=2.0f;
					TM5PositionPID.ki=0;
					TM5PositionPID.kd=0;
					TM6SpeedPID.kp=2.0f;
					TM6SpeedPID.ki=0;
					TM6SpeedPID.kd=0;
					TM6PositionPID.kp=2.5f;
					TM6PositionPID.ki=0;
					TM6PositionPID.kd=0;
					}
			{//����צ��ת���
			TM5PositionPID.ref = -TM_Ref_1;
			TM6PositionPID.ref = TM_Ref_1;
			TM5PositionPID.fdb = TM5Encoder.ecd_angle;
			TM6PositionPID.fdb = TM6Encoder.ecd_angle;
			TM5PositionPID.Calc(&TM5PositionPID);
			TM6PositionPID.Calc(&TM6PositionPID);
			TM5SpeedPID.ref = TM5PositionPID.output*5;
			TM6SpeedPID.ref = TM6PositionPID.output*5;
			TM5SpeedPID.fdb = TM5Encoder.filter_rate;
			TM6SpeedPID.fdb = TM6Encoder.filter_rate;
			TM5SpeedPID.Calc(&TM5SpeedPID);
			TM6SpeedPID.Calc(&TM6SpeedPID);
			TM5SpeedPID.output -=flipCompensation;
			TM6SpeedPID.output +=flipCompensation;
			TM5SpeedPID.output = VAL_LIMITF(TM5SpeedPID.output,-5000,5000);
			TM6SpeedPID.output = VAL_LIMITF(TM6SpeedPID.output,-5000,5000);
		}
			Claw_Position_set_ground_slow2(-120,back);
			pre=0;
			
		}
		
		case PREPARE_STATE:
		{//׼��״̬
			CTRL_Flag=1;//��ѡ����Դ��ģʽ
//			TM1Encoder.ecd_bias =4321;
//			TM1Encoder.raw_value=4321;
//			TM1Encoder.ecd_value=4321;
//			TM2Encoder.ecd_bias =4321;
//			TM2Encoder.raw_value=4321;
//			TM2Encoder.ecd_value=4321;
			if(lastWorkstate==UP_STATE)
			{
				pre=1;
			}
			if(lastWorkstate==DOWN_STATE)
			{
				pre=2;
			}
		}break;
		
		case NORMAL_STATE:
		{//����״̬

			chassis_system();//����
//			barrier_system();//ȡ�ϰ���
			move_system();
			rescue_system();//��Ԯ
			camera_system();//ͼ��
			uint8_t shouzhua=GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_3);//����0������צ��
			uint8_t shensuo=GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_4);//����0������צ�˻�
			pre=0;
			
		{//��ת90��
//				if(C_Flag==1)
//				{
//					if(ccc==0)
//					{
//						YAW_angle_target=Angles+60;
//						count_target=count11;
//						if(YAW_angle_target>=180)
//						{
//							YAW_angle_target=YAW_angle_target-360;
//							count_target=count_target+1;
//						}
//						ccc=1;
//					}
//					else
//					{
//						if((count11==count_target)&&(Angles>=YAW_angle_target))
//						{
//							ccc=0;
//							C_Flag=0;
//							YAW_angle_target=0;
//						}
//						else
//						{
//							ChassisSpeedRef.rotate_ref = -300;//��ʼ����
//						}
//					}		
//				}
			
			}
		{//�ж�PD14��������ǰ���
			if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14)==1)
			{
				GPIOD14=0;
			}
			else
			{
				GPIOD14=1;
			}
		}
		
		{//�ж�PD15�����������
			if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_15)==1)
			{
				GPIOD15=0;
			}
			else
			{
				GPIOD15=1;
			}
		}
		{//�ж�PD13��������ս�
			if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13)==1)
			{
				GPIOD13=1;
			}
			else
			{
				GPIOD13=0;
			}
		}
		{//�ж�����״̬
			if (GPIO_ReadOutputDataBit(GPIOH,GPIO_Pin_2)==0)
			{
				GPIOH2=0;
			}
			else
			{
				GPIOH2=1;
			}
			if (GPIO_ReadOutputDataBit(GPIOH,GPIO_Pin_3)==0)
			{
				GPIOH3=0;
			}
			else
			{
				GPIOH3=1;
			}
			if (GPIO_ReadOutputDataBit(GPIOH,GPIO_Pin_4)==0)
			{
				GPIOH4=0;
			}
			else
			{
				GPIOH4=1;
			}
			if (GPIO_ReadOutputDataBit(GPIOH,GPIO_Pin_5)==0)
			{
				GPIOH5=0;
			}
			else
			{
				GPIOH5=1;
			}
			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)==0)
			{
				GPIOE6=0;
			}
			else
			{
				GPIOE6=1;
			}
			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5)==0)
			{
				GPIOE5=0;
			}
			else
			{
				GPIOE5=1;
			}
		}
		
	if(CTRL_Flag==1)//����Դ��״̬
	{		
		if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
			{
					flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
			}
			else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
			{
					flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
			}
			if((miningTaskStatus==MINING_NOTASK_STATE||miningTaskStatus==MININGRETURN3_STATE)&&(rescueTaskStatus != RESCUE_STATE))
			{
				gl=16384;
			}
			else
			{
				gl=5000;
			}
			
		switch(miningTaskStatus)
		{
			case MINING_NOTASK_STATE:
			{
				TM_Ref_3 = 0;
				TM_Ref_4 = 0;
				{//̧������
					upliftCompensation_1=400;
					upliftCompensation_2=400;
					
				}
				{//��צ����
					flipCompensation=0;
				}
				{//PID
					TM1SpeedPID.kp=2.0f;
					TM1SpeedPID.ki=0;
					TM1SpeedPID.kd=0;
					TM1PositionPID.kp=5.0f;
					TM1PositionPID.ki=0;
					TM1PositionPID.kd=0;
					TM2SpeedPID.kp=2.0f;
					TM2SpeedPID.ki=0;
					TM2SpeedPID.kd=0;
					TM2PositionPID.kp=5.0f;
					TM2PositionPID.ki=0;
					TM2PositionPID.kd=0;
					
					TM5SpeedPID.kp=2.0f;
					TM5SpeedPID.ki=0;
					TM5SpeedPID.kd=0;
					TM5PositionPID.kp=5.0f;
					TM5PositionPID.ki=0;
					TM5PositionPID.kd=0;
					TM6SpeedPID.kp=2.0f;
					TM6SpeedPID.ki=0;
					TM6SpeedPID.kd=0;
					TM6PositionPID.kp=5.0f;
					TM6PositionPID.ki=0;
					TM6PositionPID.kd=0;
							
					LM1SpeedPID.kp=2;
					LM2SpeedPID.kp=2;
					LM1SpeedPID.ki=0;
					LM2SpeedPID.ki=0;
					LM1SpeedPID.kd=0;
					LM2SpeedPID.kd=0;
					LM1PositionPID.kp=2;
					LM2PositionPID.kp=2;
					LM1PositionPID.ki=0;
					LM2PositionPID.ki=0;
					LM1PositionPID.kd=0;
					LM2PositionPID.kd=0;
					
					TM3SpeedPID.kp=8;
					TM4SpeedPID.kp=8;
					TM3SpeedPID.ki=0;
					TM4SpeedPID.ki=0;
					TM3SpeedPID.kd=0;
					TM4SpeedPID.kd=0;
				}
				{//���׶���

					  GPIO_ResetBits(GPIOH,GPIO_Pin_4);//��צ�˻�
					  GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�պ�����צ
//						GPIO_ResetBits(GPIOE,GPIO_Pin_5);//��е��Ԯ��������
//						GPIO_ResetBits(GPIOE,GPIO_Pin_6);//ˢ����Ԯ��������
					
				}
				{//�������
					Claw_Position_set_slow(0,back); //��צ����
					if(TM2PositionPID.ref<=15)
					{
						Position_set_slow(0,down);       //̧���½�
					}
				}
				flag_kj_sz=0;
				flag_kj_ts=0;
				flag_qk3=0;
				flag6_micrsecond_qd1.flag=0;
				flag6_micrsecond_qd2.flag=0;
				flag1_micrsecond.flag = 0;
				flag_qudi=0;
				switch(lastminingTaskStatus) //�Ӹ���״̬���ص�������״̬
				{
					case MININGOBTAIN_STATE:
					{
						flag_ex=0;
						flag_ob=0;
						flag_kj_sz=0;
						flag_kj_ts=0;
						flag_qk3=0;
						flag_qudi=0;
						GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
						Claw_Position_set_ground(0,back);
						GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н����ײ��ӽ�
						

					}break;
					
					
					case MINING2_STATE:
					{
						GPIO_SetBits(GPIOH,GPIO_Pin_2);//�н����׼н�
					}break;
					case MININGRETURN1_STATE:
					{
						GPIO_SetBits(GPIOH,GPIO_Pin_3);//�н���צ
						GPIO_ResetBits(GPIOH,GPIO_Pin_2);
					}break;
					
					case MININGRETURN2_STATE:
					{
						GPIO_SetBits(GPIOH,GPIO_Pin_3);//�н���צ
						GPIO_SetBits(GPIOH,GPIO_Pin_2);//�н����׼н�
						
					}break;
					
					case MININGGROUND_STATE:
					{
						GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
						Claw_Position_set_ground(0,back);
						flag_qudi=0;
						GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н����ײ��ӽ�
					}break;
					case MININGGROUND1_STATE:
					{
						GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
						Claw_Position_set_ground(0,back);
						flag_qudi=0;
						GPIO_SetBits(GPIOH,GPIO_Pin_2);//�н����׼н�
					}break;
					case MININGGROUND2_STATE:
					{
						Claw_Position_set_ground(0,back);
						GPIO_SetBits(GPIOH,GPIO_Pin_3);
						flag_qudi=0;
						GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н����ײ��ӽ�
					}break;
					
					case MININGPUSH_STATE:
					{
						GPIO_ResetBits(GPIOH,GPIO_Pin_4);//��צ�˻�
						if(TM2PositionPID.ref<=95)
					{
						GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
					}
						else
					{
						GPIO_SetBits(GPIOH,GPIO_Pin_3);
					}
					}break;
					default:
					{
						GPIO_ResetBits(GPIOH,GPIO_Pin_4);//��צ�˻�
						GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
						Claw_Position_set_ground(0,back);
						GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н����ײ��ӽ�
					}break;
				}
			}break;
			case MINING1_STATE:
			{
				{//��צ����
				
				}
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л����ɿ�״̬1
					{
						{//̧������
							upliftCompensation_1 = 1000;
							upliftCompensation_2 = 1000;
						}
						{//PID
							
							{
								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
								{
									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
								}
								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
								{
									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
								}
							}
							
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
						}
						{//���嶯��

//							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
//							GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
//							Position_set(100,up);//̧��
//							if(LM1PositionPID.ref>=(height_b-5))
//							{
//								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ���צ
//								Claw_Position_set(90,out);//��ת�Ƕ�
//								if(Mouse_l_Flag==1) //���
//								{
//									Claw_Position_set(claw_out_angel,out);
//								}
//								else if(Mouse_r_Flag==1) //�Ҽ�
//								{
//									Claw_Position_set(90,back);
//								}
//							}					
							
							if(flag_kj_sz==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							}
							else if(flag_kj_sz==1)
							{
								if(flag5_micrsecond_kj.flag==1)
								{
									flag5_micrsecond_kj.time_now = Get_Time_Micros();
									flag5_micrsecond_kj.time_error = flag5_micrsecond_kj.time_now - flag5_micrsecond_kj.time_last;
								}
								if(flag5_micrsecond_kj.time_error>22000)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_3);//�պ�
									flag5_micrsecond_kj.flag=0;
								}
								else
								{
									GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
								}
								
							}
							
							GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							GPIO_ResetBits(GPIOH,GPIO_Pin_2);
							if(flag_kj_ts==0)
							{
								Position_set(100,up);//̧��
							}
							else if(flag_kj_ts==1)
							{
								Position_set(put_height_first,up);
							}
							if(LM1PositionPID.ref>=(height_b-5))
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ���צ
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}
						{//�սӲ���
							if(F_Flag==1)
							{
								flag_kj_ts=1;
							}
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								if(GPIOD13==0&&flag_kj_sz==0)
								{
									flag_kj_sz=1;
									if(flag5_micrsecond_kj.flag==0)
									{
										flag5_micrsecond_kj.time_last = Get_Time_Micros();
										flag5_micrsecond_kj.flag=1;
									}
									
								}
							}
						}
						}
					}break;
					case MININGRETURN1_STATE: //�ӷ���״̬1�л����ɿ�״̬1����һ�βɿ�ʧ�ܣ�
					{
//						if(TM2Encoder.ecd_angle>85)
//						{//PID
//							TM1SpeedPID.kp=8.0f;
//							TM1SpeedPID.ki=0;
//							TM1SpeedPID.kd=10.0f;
//							TM1PositionPID.kp=8.0f;
//							TM1PositionPID.ki=0.01;
//							TM1PositionPID.kd=5;
//							TM2SpeedPID.kp=8.0f;
//							TM2SpeedPID.ki=0;
//							TM2SpeedPID.kd=10.0f;
//							TM2PositionPID.kp=8.0f;
//							TM2PositionPID.ki=0.01;
//							TM2PositionPID.kd=5;
//						}
//						else if(TM2Encoder.ecd_angle<=85)
//						{
//							TM1SpeedPID.kp=2.0f;
//							TM1SpeedPID.ki=0;
//							TM1SpeedPID.kd=1.0f;
//							TM1PositionPID.kp=3.0f;
//							TM1PositionPID.ki=0;
//							TM1PositionPID.kd=2;
//							TM2SpeedPID.kp=3.0f;
//							TM2SpeedPID.ki=0;
//							TM2SpeedPID.kd=1.0f;
//							TM2PositionPID.kp=2.0f;
//							TM2PositionPID.ki=0;
//							TM2PositionPID.kd=2;
//						}	
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						
						
//						{//���嶯��
//							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
//							Position_set(height_b,down);//�½�
//							if(LM1PositionPID.ref>=(height_b-5))
//							{
//								Claw_Position_set(90,out);//��ת�Ƕ�
//								if(Mouse_l_Flag==1)       //���
//								{
//									Claw_Position_set(claw_out_angel,out);
//								}
//								else if(Mouse_r_Flag==1) //�Ҽ�
//								{
//									Claw_Position_set(90,back);
//								}
//							}
////							Claw_Position_set(90,out);//��ת�Ƕ�
////							if(Mouse_l_Flag==1)       //���
////							{
////								Claw_Position_set(claw_out_angel,out);
////							}
////							else if(Mouse_r_Flag==1) //�Ҽ�
////							{
////								Claw_Position_set(90,back);
////							}
//						}
						{//���嶯��
							if(flag_kj_sz==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							}
							else if(flag_kj_sz==1)
							{
								if(flag5_micrsecond_kj.flag==1)
								{
									flag5_micrsecond_kj.time_now = Get_Time_Micros();
									flag5_micrsecond_kj.time_error = flag5_micrsecond_kj.time_now - flag5_micrsecond_kj.time_last;
								}
								if(flag5_micrsecond_kj.time_error>22000)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_3);//�պ�
									flag5_micrsecond_kj.flag=0;
								}
								else
								{
									GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
								}
								
							}
							
							GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							
							if(flag_kj_ts==0)
							{
								Position_set(100,down);//�½�
							}
							else if(flag_kj_ts==1)
							{
								Position_set(put_height_first,up);
							}
							if(LM1PositionPID.ref>=(height_b-5))
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ���צ
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}
						{//�սӲ���
							if(F_Flag==1)
							{
								flag_kj_ts=1;
							}
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								if(GPIOD13==0&&flag_kj_sz==0)
								{
									flag_kj_sz=1;
									if(flag5_micrsecond_kj.flag==0)
									{
										flag5_micrsecond_kj.time_last = Get_Time_Micros();
										flag5_micrsecond_kj.flag=1;
									}
									
								}
							}
						}
						}
					}break;
					default:break;
				}
			}break;
			case MINING2_STATE:
			{
				{//��צ����
				
				}
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л����ɿ�״̬2
					{
						{//̧������
							upliftCompensation_1 = 1000;
							upliftCompensation_2 = 1000;
						}
						{//PID
							{
//								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
//								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
							}
							
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
						}
						{//���嶯��
							if( getMINERAL() == 1)
							{	                       
								TM_Ref_3 = csd_speed;
								TM_Ref_4 = -csd_speed;
								GPIO_ResetBits(GPIOH,GPIO_Pin_2);
							}
							else if(getMINERAL() == 2 && flag1_micrsecond.flag == 0)
							{
								flag1_micrsecond.flag = 1;
								flag1_micrsecond.time_last = Get_Time_Micros();
							}
							if(flag1_micrsecond.flag==1)
							{
								flag1_micrsecond.time_now = Get_Time_Micros();
								flag1_micrsecond.time_error = flag1_micrsecond.time_now - flag1_micrsecond.time_last;
								
							}
							if(flag1_micrsecond.time_error>2000000&&flag1_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;
								flag1_micrsecond.flag = 0;
								GPIO_SetBits(GPIOH,GPIO_Pin_2);//�н����׼ӽ�
							}
														
						{//�սӶ���							
							if(flag_kj_sz==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							}
							else if(flag_kj_sz==1)
							{
								if(flag5_micrsecond_kj.flag==1)
								{
									flag5_micrsecond_kj.time_now = Get_Time_Micros();
									flag5_micrsecond_kj.time_error = flag5_micrsecond_kj.time_now - flag5_micrsecond_kj.time_last;
								}
								if(flag5_micrsecond_kj.time_error>22000)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_3);//�պ�
									flag5_micrsecond_kj.flag=0;
								}
								else
								{
									GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
								}
								
							}
							
							GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							
							if(flag_kj_ts==0)
							{
								Position_set(100,up);//̧��
							}
							else if(flag_kj_ts==1)
							{
								Position_set(put_height_first,up);
							}
							if(LM1PositionPID.ref>=(height_b-5))
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ���צ
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}
							{//�սӲ���
							if(F_Flag==1)
							{
								flag_kj_ts=1;
							}
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								if(GPIOD13==0&&flag_kj_sz==0)
								{
									flag_kj_sz=1;
									if(flag5_micrsecond_kj.flag==0)
									{
										flag5_micrsecond_kj.time_last = Get_Time_Micros();
										flag5_micrsecond_kj.flag=1;
									}
									
								}
							}
							}
						}
							
					}
					}break;
					case MININGRETURN1_STATE: //�ӷ���״̬1�л����ɿ�״̬2����һ�βɿ�ɹ���
					{
						{//̧������
						
						}
						{
//								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
//								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
						}
						{//PID

							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
						}
						{//���嶯��
							
							if(getMINERAL() == 1)
							{	
								TM_Ref_3 = csd_speed;
								TM_Ref_4 = -csd_speed;
							}
							else if(getMINERAL() == 2 && flag1_micrsecond.flag == 0)
							{
								flag1_micrsecond.flag = 1;
								flag1_micrsecond.time_last = Get_Time_Micros();
							}
							if(flag1_micrsecond.flag==1)
							{
								flag1_micrsecond.time_now = Get_Time_Micros();
								flag1_micrsecond.time_error = flag1_micrsecond.time_now - flag1_micrsecond.time_last;
								
							}
							if(flag1_micrsecond.time_error>2000000&&flag1_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;
								flag1_micrsecond.flag = 0;
								GPIO_SetBits(GPIOH,GPIO_Pin_2);//�н����׼ӽ�
							}
							
						{//���嶯��							
							if(flag_kj_sz==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							}
							else if(flag_kj_sz==1)
							{
								if(flag5_micrsecond_kj.flag==1)
								{
									flag5_micrsecond_kj.time_now = Get_Time_Micros();
									flag5_micrsecond_kj.time_error = flag5_micrsecond_kj.time_now - flag5_micrsecond_kj.time_last;
								}
								if(flag5_micrsecond_kj.time_error>22000)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_3);//�պ�
									flag5_micrsecond_kj.flag=0;
								}
								else
								{
									GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
								}
								
							}
							
							GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							
							if(flag_kj_ts==0)
							{
								Position_set(100,down);//̧��
							}
							else if(flag_kj_ts==1)
							{
								Position_set(put_height_first,up);
							}
							if(LM1PositionPID.ref>=(height_b-5))
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ���צ
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}
						{//�սӲ���
							if(F_Flag==1)
							{
								flag_kj_ts=1;
							}
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								if(GPIOD13==0&&flag_kj_sz==0)
								{
									flag_kj_sz=1;
									if(flag5_micrsecond_kj.flag==0)
									{
										flag5_micrsecond_kj.time_last = Get_Time_Micros();
										flag5_micrsecond_kj.flag=1;
									}
									
								}
							}
						}
						}
						}
					}break;
					case MININGRETURN2_STATE: //�ӷ���״̬2�л����ɿ�״̬2���ڶ��βɿ�ʧ�ܣ�
					{
						{//̧������
						
						}
						{
//								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
//								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
						}
						{//PID
//						if(TM2Encoder.ecd_angle>85)
//						{//PID
//							TM1SpeedPID.kp=8.0f;
//							TM1SpeedPID.ki=0;
//							TM1SpeedPID.kd=10.0f;
//							TM1PositionPID.kp=8.0f;
//							TM1PositionPID.ki=0.01;
//							TM1PositionPID.kd=5;
//							TM2SpeedPID.kp=8.0f;
//							TM2SpeedPID.ki=0;
//							TM2SpeedPID.kd=10.0f;
//							TM2PositionPID.kp=8.0f;
//							TM2PositionPID.ki=0.01;
//							TM2PositionPID.kd=5;
//						}
//						else if(TM2Encoder.ecd_angle<=85)
//						{
//							TM1SpeedPID.kp=2.0f;
//							TM1SpeedPID.ki=0;
//							TM1SpeedPID.kd=1.0f;
//							TM1PositionPID.kp=3.0f;
//							TM1PositionPID.ki=0;
//							TM1PositionPID.kd=2;
//							TM2SpeedPID.kp=3.0f;
//							TM2SpeedPID.ki=0;
//							TM2SpeedPID.kd=1.0f;
//							TM2PositionPID.kp=2.0f;
//							TM2PositionPID.ki=0;
//							TM2PositionPID.kd=2;
//						}
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						{//���嶯��							
							if(flag_kj_sz==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							}
							else if(flag_kj_sz==1)
							{
								if(flag5_micrsecond_kj.flag==1)
								{
									flag5_micrsecond_kj.time_now = Get_Time_Micros();
									flag5_micrsecond_kj.time_error = flag5_micrsecond_kj.time_now - flag5_micrsecond_kj.time_last;
								}
								if(flag5_micrsecond_kj.time_error>22000)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_3);//�պ�
									flag5_micrsecond_kj.flag=0;
								}
								else
								{
									GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
								}
								
							}
							
							GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							
							if(flag_kj_ts==0)
							{
								Position_set(100,down);//̧��
							}
							else if(flag_kj_ts==1)
							{
								Position_set(put_height_first,up);
							}
							if(LM1PositionPID.ref>=(height_b-5))
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ���צ
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}
						{//�սӲ���
							if(F_Flag==1)
							{
								flag_kj_ts=1;
							}
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								if(GPIOD13==0&&flag_kj_sz==0)
								{
									flag_kj_sz=1;
									if(flag5_micrsecond_kj.flag==0)
									{
										flag5_micrsecond_kj.time_last = Get_Time_Micros();
										flag5_micrsecond_kj.flag=1;
									}
									
								}
							}
						}
						}
					}break;
					default:break;
				}
			}break;
			case MINING3_STATE:
			{
				{//��צ����
				
				}
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л����ɿ�״̬3
					{
						{//̧������
							upliftCompensation_1 = 1000;
							upliftCompensation_2 = 1000;
						}
						{//PID
							{
//								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
//								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
							}
							
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						{//���嶯��
						  if( getMINERAL() == 1)
							{	
								TM_Ref_3 = csd_speed;
								TM_Ref_4 = -csd_speed;
							}
							else if(getMINERAL() == 2 && flag1_micrsecond.flag == 0)
							{
								flag1_micrsecond.flag = 1;
								flag1_micrsecond.time_last = Get_Time_Micros();
							}
							if(flag1_micrsecond.flag==1)
							{
								flag1_micrsecond.time_now = Get_Time_Micros();
								flag1_micrsecond.time_error = flag1_micrsecond.time_now - flag1_micrsecond.time_last;
								
							}
							if(flag1_micrsecond.time_error>950000&&flag1_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;
								flag1_micrsecond.flag = 0;
							}
							
{//�սӶ���							
							if(flag_kj_sz==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							}
							else if(flag_kj_sz==1)
							{
								if(flag5_micrsecond_kj.flag==1)
								{
									flag5_micrsecond_kj.time_now = Get_Time_Micros();
									flag5_micrsecond_kj.time_error = flag5_micrsecond_kj.time_now - flag5_micrsecond_kj.time_last;
								}
								if(flag5_micrsecond_kj.time_error>22000)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_3);//�պ�
									flag5_micrsecond_kj.flag=0;
								}
								else
								{
									GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
								}
								
							}
							
							GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							
							if(flag_kj_ts==0)
							{
								Position_set(100,up);//̧��
							}
							else if(flag_kj_ts==1)
							{
								Position_set(put_height_first,up);
							}
							if(LM1PositionPID.ref>=(height_b-5))
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ���צ
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}
							{//�սӲ���
							if(F_Flag==1)
							{
								flag_kj_ts=1;
							}
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								if(GPIOD13==0&&flag_kj_sz==0)
								{
									flag_kj_sz=1;
									if(flag5_micrsecond_kj.flag==0)
									{
										flag5_micrsecond_kj.time_last = Get_Time_Micros();
										flag5_micrsecond_kj.flag=1;
									}
									
								}
							}
							}
						}
						}
					}break;
					case MININGRETURN2_STATE: //�ӷ���״̬2�л����ɿ�״̬3���ڶ��βɿ�ɹ���
					{
						{//̧������
						
						}
						{
//								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
//								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
						}
						{//PID
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						{//�սӶ���							
							if(flag_kj_sz==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							}
							else if(flag_kj_sz==1)
							{
								if(flag5_micrsecond_kj.flag==1)
								{
									flag5_micrsecond_kj.time_now = Get_Time_Micros();
									flag5_micrsecond_kj.time_error = flag5_micrsecond_kj.time_now - flag5_micrsecond_kj.time_last;
								}
								if(flag5_micrsecond_kj.time_error>22000)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_3);//�պ�
									flag5_micrsecond_kj.flag=0;
								}
								else
								{
									GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
								}
								
							}
							
							GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							
							if(flag_kj_ts==0)
							{
								Position_set(100,down);//̧��
							}
							else if(flag_kj_ts==1)
							{
								Position_set(put_height_first,up);
							}
							if(LM1PositionPID.ref>=(height_b-5))
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ���צ
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}
							{//�սӲ���
							if(F_Flag==1)
							{
								flag_kj_ts=1;
							}
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								if(GPIOD13==0&&flag_kj_sz==0)
								{
									flag_kj_sz=1;
									if(flag5_micrsecond_kj.flag==0)
									{
										flag5_micrsecond_kj.time_last = Get_Time_Micros();
										flag5_micrsecond_kj.flag=1;
									}
									
								}
							}
							}
						}
					}break;
											
					case MININGRETURN3_STATE://�ӷ���״̬3�л����ɿ�״̬3�������βɿ�ʧ�ܣ�
					{
						{//̧������
						
						}
						{//PID
//						if(TM2Encoder.ecd_angle>85)
//						{//PID
//							TM1SpeedPID.kp=8.0f;
//							TM1SpeedPID.ki=0;
//							TM1SpeedPID.kd=10.0f;
//							TM1PositionPID.kp=8.0f;
//							TM1PositionPID.ki=0.01;
//							TM1PositionPID.kd=5;
//							TM2SpeedPID.kp=8.0f;
//							TM2SpeedPID.ki=0;
//							TM2SpeedPID.kd=10.0f;
//							TM2PositionPID.kp=8.0f;
//							TM2PositionPID.ki=0.01;
//							TM2PositionPID.kd=5;
//						}
//						else if(TM2Encoder.ecd_angle<=85)
//						{
//							TM1SpeedPID.kp=2.0f;
//							TM1SpeedPID.ki=0;
//							TM1SpeedPID.kd=1.0f;
//							TM1PositionPID.kp=3.0f;
//							TM1PositionPID.ki=0;
//							TM1PositionPID.kd=2;
//							TM2SpeedPID.kp=3.0f;
//							TM2SpeedPID.ki=0;
//							TM2SpeedPID.kd=1.0f;
//							TM2PositionPID.kp=2.0f;
//							TM2PositionPID.ki=0;
//							TM2PositionPID.kd=2;
//						}
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						{//�սӶ���							
							if(flag_kj_sz==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							}
							else if(flag_kj_sz==1)
							{
								if(flag5_micrsecond_kj.flag==1)
								{
									flag5_micrsecond_kj.time_now = Get_Time_Micros();
									flag5_micrsecond_kj.time_error = flag5_micrsecond_kj.time_now - flag5_micrsecond_kj.time_last;
								}
								if(flag5_micrsecond_kj.time_error>22000)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_3);//�պ�
									flag5_micrsecond_kj.flag=0;
								}
								else
								{
									GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
								}
								
							}
							
							GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							
							if(flag_kj_ts==0)
							{
								Position_set(100,down);//̧��
							}
							else if(flag_kj_ts==1)
							{
								Position_set(put_height_first,up);
							}
							if(LM1PositionPID.ref>=(height_b-5))
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�Ƴ���צ
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}
							{//�սӲ���
							if(F_Flag==1)
							{
								flag_kj_ts=1;
							}
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								if(GPIOD13==0&&flag_kj_sz==0)
								{
									flag_kj_sz=1;
									if(flag5_micrsecond_kj.flag==0)
									{
										flag5_micrsecond_kj.time_last = Get_Time_Micros();
										flag5_micrsecond_kj.flag=1;
									}
									
								}
							}
							}
						}
					}break;
					
			 }break;
		 }
			case MININGRETURN1_STATE:
			{
				flag_kj_sz=0;
				flag_kj_ts=0;
				flag1_micrsecond.flag = 0;
				{//��צ����
					
				}
				switch(lastminingTaskStatus)
				{
					case MINING1_STATE: //�Ӳɿ�״̬1�л�������״̬1
					{
						{//��צ����800
//							if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
//							else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
						}
						if(TM2Encoder.ecd_angle>90)
						{
							TM1SpeedPID.kp=2.0f;
					    TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=5.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=2.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=5.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}
						else if(TM2Encoder.ecd_angle<=90)
						{
							TM1SpeedPID.kp=-1.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=0.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=-1.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=0.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}

							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						
						{//���嶯��
							GPIO_ResetBits(GPIOH,GPIO_Pin_2);
							GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
							if(LM1PositionPID.ref<600)
							{
								Position_set(height_b_p,up);//̧��
								if(LM1PositionPID.ref>=(height_b_p-5))
								{
									Claw_Position_set_slow(0,back);//��ת�Ƕ�
								}
							}
							else if(LM1PositionPID.ref>=600)
							{
								Position_set(height_b_p,down);//�½�
								if(LM1PositionPID.ref<=(height_b_p+5))
								{
									Claw_Position_set_slow(0,back);//��ת�Ƕ�
								}
							}

						}
					}break;
					default:break;
				}
			}break;
			case MININGRETURN2_STATE:
			{
				{//��צ����
					
				}
				flag_kj_sz=0;
				flag_kj_ts=0;
				switch(lastminingTaskStatus)
				{
					case MINING2_STATE: //�Ӳɿ�״̬2�л�������״̬2
					{
						{//PID
						{//��צ����800
//							if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
//							else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
						}
						if(TM2Encoder.ecd_angle>90)
						{
							TM1SpeedPID.kp=2.0f;
					    TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=5.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=2.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=5.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}
						else if(TM2Encoder.ecd_angle<=90)
						{
							TM1SpeedPID.kp=-1.0f;
					        TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=0.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=-1.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=0.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						{//���嶯��
							GPIO_SetBits(GPIOH,GPIO_Pin_2);
							GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
							if(LM1PositionPID.ref<600)
							{
								Position_set(height_b_p,up);//̧��
								if(LM1PositionPID.ref>=(height_b_p-5))
								{
									Claw_Position_set_slow(0,back);//��ת�Ƕ�
								}
							}
							else if(LM1PositionPID.ref>=600)
							{
								Position_set(height_b_p,down);//�½�
								if(LM1PositionPID.ref<=(height_b_p+5))
								{
									Claw_Position_set_slow(0,back);//��ת�Ƕ�
								}
							}

						}
					}break;
					default:break;
				}
			}break;
			case MININGRETURN3_STATE:
			{
				{//��צ����
					
				}
				flag_kj_sz=0;
				flag_kj_ts=0;
				switch(lastminingTaskStatus)
				{
					case MINING3_STATE: //�Ӳɿ�״̬3�л�������״̬3
					{
						{//PID
						{//��צ����800
//							if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
//							else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
						}
						if(TM2Encoder.ecd_angle>90)
						{
							TM1SpeedPID.kp=2.0f;
					        TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=5.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=2.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=5.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}
						else if(TM2Encoder.ecd_angle<=90)
						{
							TM1SpeedPID.kp=-1.0f;
					        TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=0.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=-1.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=0.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
//						{//���嶯��
//						  GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
//						  GPIO_SetBits(GPIOH,GPIO_Pin_5);//ѹ����ʯ
//							if(flag_qk3==0)
//							{
//								Position_set(height_b_p,up);//̧��
//							}
//								if(LM1PositionPID.ref>=(height_b_p-5))
//								{
//									flag_qk3=1;
//									Claw_Position_set_slow(60,back);//��ת�Ƕ�
//								}
//								if(TM2PositionPID.ref<=65&&flag_qk3==1)
//								{
//									GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�ջ���צ
//									Position_set_slow(0,down);
//								}
//								
//						}
						{//���嶯��
							GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
							GPIO_SetBits(GPIOH,GPIO_Pin_2);//ѹ����ʯ
							if(LM1PositionPID.ref<600)
							{
								if(flag_qk3==0)
								{
									Position_set(height_b_p,up);//̧��
								}
								if(LM1PositionPID.ref>=(height_b_p-5))
								{
									flag_qk3=1;
									Claw_Position_set_slow(60,back);//��ת�Ƕ�
								}
								if(TM2PositionPID.ref<=65&&flag_qk3==1)
								{
									GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�ջ���צ
									Position_set_slow(0,down);
								}
							}
							else if(LM1PositionPID.ref>=600)
							{
								Position_set(height_b_p,down);//�½�
								if(LM1PositionPID.ref<=(height_b_p+5))
								{
									Claw_Position_set_slow(60,back);//��ת�Ƕ�
								}
								if(TM2PositionPID.ref<=65)
								{
									Position_set_slow(0,down);
								}
								
							}

						}
					}break;
					default:break;
				}
			}break;
			
			case MININGRETURN4_STATE:
			{
				switch(lastminingTaskStatus)
				{
					{//��צ����
					
				  }
					case MININGPUSH_STATE:  //��PUSH״̬�л�������״̬״̬4
					{
						{//PID
					  TM1SpeedPID.kp=2.0f;
					  TM1SpeedPID.ki=0;
					  TM1SpeedPID.kd=0;
					  TM1PositionPID.kp=5.0f;
					  TM1PositionPID.ki=0;
					  TM1PositionPID.kd=0;
				  	TM2SpeedPID.kp=2.0f;
					  TM2SpeedPID.ki=0;
				  	TM2SpeedPID.kd=0;
				   	TM2PositionPID.kp=5.0f;
				   	TM2PositionPID.ki=0;
					  TM2PositionPID.kd=0;
						
								LM1SpeedPID.kp=2;
								LM2SpeedPID.kp=2;
								LM1SpeedPID.ki=0;
								LM2SpeedPID.ki=0;
								LM1SpeedPID.kd=0;
								LM2SpeedPID.kd=0;
								LM1PositionPID.kp=2;
								LM2PositionPID.kp=2;
								LM1PositionPID.ki=0;
								LM2PositionPID.ki=0;
								LM1PositionPID.kd=0;
								LM2PositionPID.kd=0;
								
				    	TM3SpeedPID.kp=8;
					    TM4SpeedPID.kp=8;
					    TM3SpeedPID.ki=0;
					    TM4SpeedPID.ki=0;
					    TM3SpeedPID.kd=0;
				    	TM4SpeedPID.kd=0;
					}
						{//���嶯��
						  GPIO_ResetBits(GPIOH,GPIO_Pin_4);//��צ�˻�
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�צ��
				    	Claw_Position_set_slow(90,back);//��ת�Ƕ�
							//������ǰת���Ҽ�����
							if(getMINERAL()==2)
							{
							  if(Mouse_l_Flag==1) //���
							   	{
								TM_Ref_3 = csd_speed;
								TM_Ref_4 = -csd_speed;

								  }
							  else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
  							TM_Ref_3 = -csd_speed;
								TM_Ref_4 = csd_speed;
								  }
							  else if(Mouse_l_Flag==0)
								  {
							  TM_Ref_3 = 0;
								TM_Ref_4 = 0;	    
								  }
							
						  	else if(Mouse_r_Flag==0)
						    	{
							  TM_Ref_3 = 0;
								TM_Ref_4 = 0;	  
							    }
								}
							if(getMINERAL()==1||3)
							{
							  if(Mouse_l_Flag==1) //���
							   	{
								TM_Ref_3 = csd_speed;
								  }
							  else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
  							TM_Ref_3 = -csd_speed;
								  }
							  else if(Mouse_l_Flag==0)
								  {
							  TM_Ref_3 = 0;
								TM_Ref_4 = 0;	
								  }
							
						  	else if(Mouse_r_Flag==0)
						    	{
							  TM_Ref_3 = 0;
								TM_Ref_4 = 0;	
							    }
								}
						 }
					}break;
					default:break;
				}
			}break;
			case MININGEXCHANGE_STATE:
			{//R���������ɿ���ʯ,��צ90��
					flag_ex=0;
					flag_qk3=0;
					flag_qudi=0;
					GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�ɿ���ʯ
				switch(lastminingTaskStatus)
				{
					
					case MINING_NOTASK_STATE:  //��������״̬�л����һ�״̬
					{
						{//PID
							TM1SpeedPID.kp=8.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=8.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
						
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
								
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
					}
					{//���嶯��
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�ջ���צ
							GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�ɿ���ʯ
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ɿ���ʯ
							Claw_Position_set(90,out);//��ת�Ƕ�
						
               if(getMINERAL() == 3||1)
							 {
								if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
							 if(getMINERAL() == 2)
							 {
								 if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = csd_speed;	
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = -csd_speed;	
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
							 
						}  
				
					}break;
					case MININGPUSH_STATE:
					{
						{//PID
							TM1SpeedPID.kp=8.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=8.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
						
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
					}
					{//���嶯��
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�ջ���צ
							GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�ɿ���ʯ
							Claw_Position_set(90,back);//��ת�Ƕ�
						  GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ɿ���ʯ
							 if(getMINERAL() == 3||1)
							 {
								if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
							 if(getMINERAL() == 2)
							 {
								 if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = csd_speed;	
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = -csd_speed;	
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
					}
					}break;
					case MININGOBTAIN_STATE:  //��ȡ����״̬�л����һ�״̬
					{
						{//PID
							TM1SpeedPID.kp=8.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=8.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
						
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
								
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
					}
					{//���嶯��
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�ջ���צ
							GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�ɿ���ʯ
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ɿ���ʯ
							Claw_Position_set(90,back);//��ת�Ƕ�
						  
						
               if(getMINERAL() == 3||1)
							 {
								if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
							 if(getMINERAL() == 2)
							 {
								 if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = csd_speed;	
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = 800;
									TM_Ref_4 = -800;	
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
							 
						}  
				
					}break;
				
				}
			}break;
			case MININGEXCHANGE2_STATE:
			{
					switch(lastminingTaskStatus)
				{
					case MININGRETURN3_STATE:  //�ӷ���3״̬�л����һ�״̬
					{
						{//PID
							TM1SpeedPID.kp=8.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=8.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=0;
						
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
								
				    	TM3SpeedPID.kp=8;
					    TM4SpeedPID.kp=8;
					    TM3SpeedPID.ki=0;
					    TM4SpeedPID.ki=0;
					    TM3SpeedPID.kd=0;
				    	TM4SpeedPID.kd=0;
					}
						{//���嶯��
							
							GPIO_SetBits(GPIOH,GPIO_Pin_2);//�����׼н�
							GPIO_SetBits(GPIOH,GPIO_Pin_3);//�н���ʯ
							Position_set(put_height_first,up);
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								Claw_Position_set_slow(180,out);
							}
						}
					}break;
				}
			}break;
			case MININGOBTAIN_STATE:
			{//F����ȡ����+��̨��
				GPIO_SetBits(GPIOH,GPIO_Pin_2);
				switch(lastminingTaskStatus)
				{
					case MININGEXCHANGE_STATE:  //�Ӷһ�״̬�л���ȡ����״̬
					{		
					{//PID
							TM1SpeedPID.kp=8.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=8.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
						
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
								
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
					}
						

						
					if(flag_ex==0)
						{
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//Ĭ��ץȡ������ǰ�ɿ���ʯ
							Claw_Position_set_slow(0,back);//צ�ӷ���ȡ���ڿ�ʯ
						}	
						
						if(TM2Encoder.ecd_angle<=3||flag_ex==1)
						{
							flag_ex=1;
							Position_set(put_height_first,up);
							GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
							
							if(flag_obtain.flag == 0)
							{
								flag_obtain.flag = 1;
								flag_obtain.time_last = Get_Time_Micros();
							}
							if(flag_obtain.flag==1)
							{
								flag_obtain.time_now = Get_Time_Micros();
								flag_obtain.time_error = flag_obtain.time_now - flag_obtain.time_last;
							}
							if(flag_obtain.time_error>50000&&flag_obtain.flag==1&&LM1PositionPID.ref>=(put_height_first-5))
							{
								Claw_Position_set_slow(180,out);
							}
							
						}

//						
//							if(getMINERAL()==2&&flag2_micrsecond.flag==0)
//							{
//								GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н������ɿ�
//								TM_Ref_3 = -csd_speed;
//								TM_Ref_4 = csd_speed;
//								flag2_micrsecond.flag = 1;
//								flag2_micrsecond.time_last = Get_Time_Micros();
//							}
//							if(flag2_micrsecond.flag==1)
//							{
//								flag2_micrsecond.time_now = Get_Time_Micros();
//								flag2_micrsecond.time_error = flag2_micrsecond.time_now - flag2_micrsecond.time_last;
//							}
//							if(flag2_micrsecond.time_error>95000&&flag2_micrsecond.flag==1)
//							{
//								TM_Ref_3 = 0;
//								TM_Ref_4 = 0;	
//								flag2_micrsecond.flag = 0;
//							}
//					
					}break;
					case MININGRETURN4_STATE:  //�ӷ���״̬4�л���ȡ����״̬
					{
						
						Claw_Position_set_slow(0,back);//צ�ӷ���ȡ���ڿ�ʯ
						GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
							if(flag2_micrsecond.flag == 0)
							{
								flag2_micrsecond.flag = 1;
								flag2_micrsecond.time_last = Get_Time_Micros();
							}

							if(getMINERAL()==2&&flag2_micrsecond.flag==1)
							{
								//GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н������ɿ�
								TM_Ref_3 = -csd_speed;
								TM_Ref_4 = csd_speed;
								flag2_micrsecond.time_now = Get_Time_Micros();
								flag2_micrsecond.time_error = flag2_micrsecond.time_now - flag2_micrsecond.time_last;
							}
							if(flag2_micrsecond.time_error>950000&&flag2_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;	
								flag2_micrsecond.flag = 0;
							}
					}break;
					default:break;
				}
			}break;
			case MININGPUSH_STATE:
			{//shift���������ʯ
				flag_ex=0;
				flag_ob=0;
				flag_obtain.flag = 0;
				switch(lastminingTaskStatus)
				{
					case MININGOBTAIN_STATE:  //��ȡ����״̬�л���PUSH״̬
					{
						if(Shift_Flag_time_tick_1ms>=300)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ɿ�צ��
								GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�ջ�צ��
							}
							else
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�����ʯ
							}
						
//							if(flag2_micrsecond.flag==1)
//							{
//								flag2_micrsecond.time_now = Get_Time_Micros();
//								flag2_micrsecond.time_error = flag2_micrsecond.time_now - flag2_micrsecond.time_last;
//							}
//							if(flag2_micrsecond.time_error>1700000&&flag2_micrsecond.flag==1)
//							{
//								TM_Ref_3 = 0;
//								TM_Ref_4 = 0;	
//								flag2_micrsecond.flag = 0;
//							}
						
					}break;
					case MININGEXCHANGE2_STATE:  //��ȡ����״̬�л���PUSH״̬
					{
						if(Shift_Flag_time_tick_1ms>=300)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ɿ�צ��
								GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�ջ�צ��
							}
							else
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�����ʯ
							}
							
						
					}break;
				}
			}
			case MININGGROUND_STATE:
			{//SHIFT����ȡ��һϵ�ж���
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л���ȡ��״̬
					{
						{//PID
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							TM5SpeedPID.kp=8.0f;
							TM5SpeedPID.ki=0;
							TM5SpeedPID.kd=10.0f;
							TM5PositionPID.kp=8.0f;
							TM5PositionPID.ki=0.01;
							TM5PositionPID.kd=5;
							TM6SpeedPID.kp=8.0f;
							TM6SpeedPID.ki=0;
							TM6SpeedPID.kd=10.0f;
							TM6PositionPID.kp=8.0f;
							TM6PositionPID.ki=0.01;
							TM6PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
						}
						{//���嶯��

							
							
						if(flag_qudi==0)
							{
								Claw_Position_set_ground(125,out);
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								if(F_Flag==1)
							{flag_qudi=1;}
							}
						if(flag_qudi==1)
						{
							if(TM6Encoder.ecd_angle>=120&&TM2Encoder.ecd_angle>=200&&flag6_micrsecond_qd1.flag==0)
							{
								flag6_micrsecond_qd1.flag=1;
								flag6_micrsecond_qd1.time_last= Get_Time_Micros();
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								//Position_set(100,up);
							}
							else if(flag6_micrsecond_qd1.flag==0)
							{
								Claw_Position_set(210,out);
								Claw_Position_set_ground(125,out);
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								//Position_set(100,up);
							}
						
							
							if(flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd1.time_error<100000)
							{
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								//Position_set(100,up);
							}
							if(flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0&&flag6_micrsecond_qd1.time_error>=100000)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�պ�����צ
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
							}
							if(flag6_micrsecond_qd1.time_error>400000&&flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0)
							{
								Claw_Position_set_ground(66,back);
							
							}
							if(TM6Encoder.ecd_angle<=68&&flag6_micrsecond_qd1.flag==1)
							{
								Claw_Position_set_ground(66,back);
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								if(flag6_micrsecond_qd2.flag==0)
								{
									flag6_micrsecond_qd2.flag=1;
									flag6_micrsecond_qd2.time_last= Get_Time_Micros();
								}
								if(flag6_micrsecond_qd2.flag==1)
								{
									flag6_micrsecond_qd2.time_now = Get_Time_Micros();
									flag6_micrsecond_qd2.time_error = flag6_micrsecond_qd2.time_now - flag6_micrsecond_qd2.time_last;
								}
								if(flag6_micrsecond_qd2.time_error>500000&&flag6_micrsecond_qd2.flag==1)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								}
							}
							if(flag6_micrsecond_qd2.time_error>800000&&flag6_micrsecond_qd2.flag==1)
							{
								Claw_Position_set_slow(0,back);          //����צ�պϺ󷭻�
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								//Position_set(0,down);
								//flag6_micrsecond_qd1.flag=0;
							}
						}
						}
					}break;
					default:break;
				}
			}break;

			case MININGGROUND1_STATE:
			{//SHIFT����ȡ��1һϵ�ж���
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л���ȡ��״̬
					{
						{//PID
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							TM5SpeedPID.kp=8.0f;
							TM5SpeedPID.ki=0;
							TM5SpeedPID.kd=10.0f;
							TM5PositionPID.kp=8.0f;
							TM5PositionPID.ki=0.01;
							TM5PositionPID.kd=5;
							TM6SpeedPID.kp=8.0f;
							TM6SpeedPID.ki=0;
							TM6SpeedPID.kd=10.0f;
							TM6PositionPID.kp=8.0f;
							TM6PositionPID.ki=0.01;
							TM6PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
						}
						{//���ʹ�
							if( getMINERAL() == 1)
							{	                       
								TM_Ref_3 = csd_speed;
								TM_Ref_4 = -csd_speed;
								GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н����׼ӽ�
							}
							else if(getMINERAL() == 2 && flag1_micrsecond.flag == 0)
							{
								flag1_micrsecond.flag = 1;
								flag1_micrsecond.time_last = Get_Time_Micros();
							}
							if(flag1_micrsecond.flag==1)
							{
								flag1_micrsecond.time_now = Get_Time_Micros();
								flag1_micrsecond.time_error = flag1_micrsecond.time_now - flag1_micrsecond.time_last;
								
							}
							if(flag1_micrsecond.time_error>950000&&flag1_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;
								flag1_micrsecond.flag = 0;
								GPIO_SetBits(GPIOH,GPIO_Pin_2);//�н����׼ӽ�
							}
						}
						{//���嶯��

							if(flag_qudi==0)
							{
								Claw_Position_set_ground(125,out);
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								if(F_Flag==1)
							{flag_qudi=1;}
							}
						if(flag_qudi==1)
						{
							if(TM6Encoder.ecd_angle>=120&&TM2Encoder.ecd_angle>=200&&flag6_micrsecond_qd1.flag==0)
							{
								flag6_micrsecond_qd1.flag=1;
								flag6_micrsecond_qd1.time_last= Get_Time_Micros();
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							else if(flag6_micrsecond_qd1.flag==0)
							{
								Claw_Position_set(210,out);
								Claw_Position_set_ground(125,out);
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							if(flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd1.time_error<100000)
							{
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							if(flag6_micrsecond_qd1.time_error>=100000&&flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�պ�����צ
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
							}
							if(flag6_micrsecond_qd1.time_error>400000&&flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0)
							{
								Claw_Position_set_ground(66,back);
							}
							if(TM6Encoder.ecd_angle<=68&&flag6_micrsecond_qd1.flag==1)
							{
								Claw_Position_set_ground(66,back);
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								if(flag6_micrsecond_qd2.flag==0)
								{
									flag6_micrsecond_qd2.flag=1;
									flag6_micrsecond_qd2.time_last= Get_Time_Micros();
								}
								if(flag6_micrsecond_qd2.flag==1)
								{
									flag6_micrsecond_qd2.time_now = Get_Time_Micros();
									flag6_micrsecond_qd2.time_error = flag6_micrsecond_qd2.time_now - flag6_micrsecond_qd2.time_last;
								}
								if(flag6_micrsecond_qd2.time_error>700000&&flag6_micrsecond_qd2.flag==1)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								}
							}
							if(flag6_micrsecond_qd2.time_error>1000000&&flag6_micrsecond_qd2.flag==1)
							{
								Claw_Position_set_slow(0,back);          //����צ�պϺ󷭻�
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								//flag6_micrsecond_qd1.flag=0;
							}
						}
						}
					}break;
					default:break;
				}
			}break;
			case MININGGROUND2_STATE:
			{//SHIFT����ȡ��2һϵ�ж���
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л���ȡ��״̬
					{
						{//PID
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							TM5SpeedPID.kp=8.0f;
							TM5SpeedPID.ki=0;
							TM5SpeedPID.kd=10.0f;
							TM5PositionPID.kp=8.0f;
							TM5PositionPID.ki=0.01;
							TM5PositionPID.kd=5;
							TM6SpeedPID.kp=8.0f;
							TM6SpeedPID.ki=0;
							TM6SpeedPID.kd=10.0f;
							TM6PositionPID.kp=8.0f;
							TM6PositionPID.ki=0.01;
							TM6PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
						}
						{//���嶯��

							if(flag_qudi==0)
							{
								Claw_Position_set_ground(125,out);
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								if(F_Flag==1)
							{flag_qudi=1;}
							}
						if(flag_qudi==1)
						{
							if(TM6Encoder.ecd_angle>=120&&TM2Encoder.ecd_angle>200&&flag6_micrsecond_qd1.flag==0)
							{
								flag6_micrsecond_qd1.flag=1;
								flag6_micrsecond_qd1.time_last= Get_Time_Micros();
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							else if(flag6_micrsecond_qd1.flag==0)
							{
								Claw_Position_set(210,out);
								Claw_Position_set_ground(125,out);
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							if(flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd1.time_error<100000)
							{
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							if(flag6_micrsecond_qd1.time_error>=100000&&flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�պ�����צ
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
							}
							if(flag6_micrsecond_qd1.time_error>400000&&flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0)
							{
								Claw_Position_set_ground(66,back);
							}
							if(TM6Encoder.ecd_angle<=68&&flag6_micrsecond_qd1.flag==1)
							{
								Claw_Position_set_ground(66,back);
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								if(flag6_micrsecond_qd2.flag==0)
								{
									flag6_micrsecond_qd2.flag=1;
									flag6_micrsecond_qd2.time_last= Get_Time_Micros();
								}
								if(flag6_micrsecond_qd2.flag==1)
								{
									flag6_micrsecond_qd2.time_now = Get_Time_Micros();
									flag6_micrsecond_qd2.time_error = flag6_micrsecond_qd2.time_now - flag6_micrsecond_qd2.time_last;
								}
								if(flag6_micrsecond_qd2.time_error>700000&&flag6_micrsecond_qd2.flag==1)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								}
							}
							if(flag6_micrsecond_qd2.time_error>1000000&&flag6_micrsecond_qd2.flag==1)
							{
								Claw_Position_set_slow(60,back);          //����צ�պϺ󷭻�
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								//flag6_micrsecond_qd1.flag=0;
							}
						}
						}
					}break;
					default:break;
				}
			}break;
		  default:break;
		}
	}
	else if(CTRL_Flag==0)//С��Դ��״̬
	{		
		if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
			{
					flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
			}
			else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
			{
					flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
			}
			if(miningTaskStatus==MINING_NOTASK_STATE||miningTaskStatus==MININGRETURN3_STATE)
			{
				gl=16384;
			}
			else
			{
				gl=6000;
			}
			
		switch(miningTaskStatus)
		{
			case MINING_NOTASK_STATE:
			{
				TM_Ref_3 = 0;
				TM_Ref_4 = 0;
				{//̧������
					upliftCompensation_1=400;
					upliftCompensation_2=400;
					
				}
				{//��צ����
					flipCompensation=0;
				}
				{//PID
					TM1SpeedPID.kp=2.0f;
					TM1SpeedPID.ki=0;
					TM1SpeedPID.kd=0;
					TM1PositionPID.kp=5.0f;
					TM1PositionPID.ki=0;
					TM1PositionPID.kd=0;
					TM2SpeedPID.kp=2.0f;
					TM2SpeedPID.ki=0;
					TM2SpeedPID.kd=0;
					TM2PositionPID.kp=5.0f;
					TM2PositionPID.ki=0;
					TM2PositionPID.kd=0;
					
					TM5SpeedPID.kp=2.0f;
					TM5SpeedPID.ki=0;
					TM5SpeedPID.kd=0;
					TM5PositionPID.kp=5.0f;
					TM5PositionPID.ki=0;
					TM5PositionPID.kd=0;
					TM6SpeedPID.kp=2.0f;
					TM6SpeedPID.ki=0;
					TM6SpeedPID.kd=0;
					TM6PositionPID.kp=5.0f;
					TM6PositionPID.ki=0;
					TM6PositionPID.kd=0;
							
					LM1SpeedPID.kp=2;
					LM2SpeedPID.kp=2;
					LM1SpeedPID.ki=0;
					LM2SpeedPID.ki=0;
					LM1SpeedPID.kd=0;
					LM2SpeedPID.kd=0;
					LM1PositionPID.kp=2;
					LM2PositionPID.kp=2;
					LM1PositionPID.ki=0;
					LM2PositionPID.ki=0;
					LM1PositionPID.kd=0;
					LM2PositionPID.kd=0;
					
					TM3SpeedPID.kp=8;
					TM4SpeedPID.kp=8;
					TM3SpeedPID.ki=0;
					TM4SpeedPID.ki=0;
					TM3SpeedPID.kd=0;
					TM4SpeedPID.kd=0;
				}
				{//���׶���

					GPIO_ResetBits(GPIOH,GPIO_Pin_4);//��צ�˻�
					GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�պ�����צ
				}
				{//�������
					Claw_Position_set_slow(0,back); //��צ����
					if(TM2PositionPID.ref<=15)
					{
						Position_set_slow(0,down);       //̧���½�
					}
				}
				flag_kj_sz=0;
				flag_kj_ts=0;
				flag_qk3=0;
				flag6_micrsecond_qd1.flag=0;
				flag6_micrsecond_qd2.flag=0;
				flag_qudi=0;
				switch(lastminingTaskStatus) //�Ӹ���״̬���ص�������״̬
				{
					case MININGOBTAIN_STATE:
					{
						flag_ex=0;
						flag_ob=0;
						flag_kj_sz=0;
						flag_kj_ts=0;
						flag_qk3=0;
						flag_qudi=0;
						GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
						Claw_Position_set_ground(0,back);
						GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н����ײ��ӽ�
						

					}break;
					
					
					case MINING2_STATE:
					{
						GPIO_SetBits(GPIOH,GPIO_Pin_2);//�н����׼н�
					}break;
					case MININGRETURN1_STATE:
					{
						GPIO_SetBits(GPIOH,GPIO_Pin_3);//�н���צ
					}break;
					
					case MININGRETURN2_STATE:
					{
						GPIO_SetBits(GPIOH,GPIO_Pin_3);//�н���צ
						GPIO_SetBits(GPIOH,GPIO_Pin_2);//�н����׼н�
						
					}break;
					
					case MININGGROUND_STATE:
					{
						GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
						Claw_Position_set_ground(0,back);
						flag_qudi=0;
						GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н����ײ��ӽ�
					}break;
					case MININGGROUND1_STATE:
					{
						GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
						Claw_Position_set_ground(0,back);
						flag_qudi=0;
						GPIO_SetBits(GPIOH,GPIO_Pin_2);//�н����׼н�
					}break;
					case MININGGROUND2_STATE:
					{
						Claw_Position_set_ground(0,back);
						GPIO_SetBits(GPIOH,GPIO_Pin_3);
						flag_qudi=0;
						GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н����ײ��ӽ�
					}break;
					
					case MININGPUSH_STATE:
					{
						GPIO_ResetBits(GPIOH,GPIO_Pin_4);//��צ�˻�
						if(TM2PositionPID.ref<=95)
					{
						GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
					}
						else
					{
						GPIO_SetBits(GPIOH,GPIO_Pin_3);
					}
					}break;
					default:
					{
						GPIO_ResetBits(GPIOH,GPIO_Pin_4);//��צ�˻�
						GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
						Claw_Position_set_ground(0,back);
						GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н����ײ��ӽ�
					}break;
				}
			}break;
			case MINING1_STATE:
			{
				{//��צ����
				
				}
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л����ɿ�״̬1
					{
						{//̧������
							upliftCompensation_1 = 1000;
							upliftCompensation_2 = 1000;
						}
						{//PID
							
							{
								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
								{
									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
								}
								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
								{
									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
								}
							}
							
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
						}
						{//���嶯��

							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);//���Ƴ�
							Position_set(height_s,up);//̧��
							if(LM1PositionPID.ref>=(height_s-5))
							{
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}					
							
						}
					}break;
					case MININGRETURN1_STATE: //�ӷ���״̬1�л����ɿ�״̬1����һ�βɿ�ʧ�ܣ�
					{
//						if(TM2Encoder.ecd_angle>85)
//						{//PID
//							TM1SpeedPID.kp=8.0f;
//							TM1SpeedPID.ki=0;
//							TM1SpeedPID.kd=10.0f;
//							TM1PositionPID.kp=8.0f;
//							TM1PositionPID.ki=0.01;
//							TM1PositionPID.kd=5;
//							TM2SpeedPID.kp=8.0f;
//							TM2SpeedPID.ki=0;
//							TM2SpeedPID.kd=10.0f;
//							TM2PositionPID.kp=8.0f;
//							TM2PositionPID.ki=0.01;
//							TM2PositionPID.kd=5;
//						}
//						else if(TM2Encoder.ecd_angle<=85)
//						{
//							TM1SpeedPID.kp=2.0f;
//							TM1SpeedPID.ki=0;
//							TM1SpeedPID.kd=1.0f;
//							TM1PositionPID.kp=3.0f;
//							TM1PositionPID.ki=0;
//							TM1PositionPID.kd=2;
//							TM2SpeedPID.kp=3.0f;
//							TM2SpeedPID.ki=0;
//							TM2SpeedPID.kd=1.0f;
//							TM2PositionPID.kp=2.0f;
//							TM2PositionPID.ki=0;
//							TM2PositionPID.kd=2;
//						}	
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						
						
						{//���嶯��
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							Position_set(height_s,down);//�½�
							if(LM1PositionPID.ref>=(height_s-5))
							{
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1)       //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}
						}
					}break;
					default:break;
				}
			}break;
			case MINING2_STATE:
			{
				{//��צ����
				
				}
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л����ɿ�״̬2
					{
						{//̧������
							upliftCompensation_1 = 1000;
							upliftCompensation_2 = 1000;
						}
						{//PID
							{
//								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
//								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
							}
							
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						{//���嶯��
							if( getMINERAL() == 1)
							{	                       
								TM_Ref_3 = csd_speed;
								TM_Ref_4 = -csd_speed;
							}
							else if(getMINERAL() == 2 && flag1_micrsecond.flag == 0)
							{
								flag1_micrsecond.flag = 1;
								flag1_micrsecond.time_last = Get_Time_Micros();
							}
							if(flag1_micrsecond.flag==1)
							{
								flag1_micrsecond.time_now = Get_Time_Micros();
								flag1_micrsecond.time_error = flag1_micrsecond.time_now - flag1_micrsecond.time_last;
								
							}
							if(flag1_micrsecond.time_error>950000&&flag1_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;
								flag1_micrsecond.flag = 0;
							}
							
						{//�������
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);//���Ƴ�
							Position_set(height_s,up);//̧��
							if(LM1PositionPID.ref>=(height_s-5))
							{
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}		
						}
						}
				  }break;
					case MININGRETURN1_STATE: //�ӷ���״̬1�л����ɿ�״̬2����һ�βɿ�ɹ���
					{
						{//̧������
						
						}
						{
//								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
//								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
						}
						{//PID

							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
						}
						{//���嶯��
							
							if(getMINERAL() == 1)
							{	
								TM_Ref_3 = csd_speed;
								TM_Ref_4 = -csd_speed;
							}
							else if(getMINERAL() == 2 && flag1_micrsecond.flag == 0)
							{
								flag1_micrsecond.flag = 1;
								flag1_micrsecond.time_last = Get_Time_Micros();
							}
							if(flag1_micrsecond.flag==1)
							{
								flag1_micrsecond.time_now = Get_Time_Micros();
								flag1_micrsecond.time_error = flag1_micrsecond.time_now - flag1_micrsecond.time_last;
								
							}
							if(flag1_micrsecond.time_error>950000&&flag1_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;
								flag1_micrsecond.flag = 0;
							}
							
						
							{//�������
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);
							Position_set(height_s,up);//̧��
							if(LM1PositionPID.ref>=(height_s-5))
								{
									Claw_Position_set(90,out);//��ת�Ƕ�
									if(Mouse_l_Flag==1) //���
									{
										Claw_Position_set(claw_out_angel,out);
									}
									else if(Mouse_r_Flag==1) //�Ҽ�
									{
										Claw_Position_set(90,back);
									}
								}		
							}
						}
					}break;
					case MININGRETURN2_STATE: //�ӷ���״̬2�л����ɿ�״̬2���ڶ��βɿ�ʧ�ܣ�
					{
						{//̧������
						
						}
						{
//								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
//								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
						}
						{//PID
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						
						{//�������
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);
							Position_set(height_s,up);//̧��
							if(LM1PositionPID.ref>=(height_s-5))
							{
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}		
						}

					}break;
					default:break;
				}
			}break;
			case MINING3_STATE:
			{
				{//��צ����
				
				}
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л����ɿ�״̬3
					{
						{//̧������
							upliftCompensation_1 = 1000;
							upliftCompensation_2 = 1000;
						}
						{//PID
							{
//								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
//								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
							}
							
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
							
						}
						{//���嶯��
						  if( getMINERAL() == 1)
							{	
								TM_Ref_3 = csd_speed;
								TM_Ref_4 = -csd_speed;
							}
							else if(getMINERAL() == 2 && flag1_micrsecond.flag == 0)
							{
								flag1_micrsecond.flag = 1;
								flag1_micrsecond.time_last = Get_Time_Micros();
							}
							if(flag1_micrsecond.flag==1)
							{
								flag1_micrsecond.time_now = Get_Time_Micros();
								flag1_micrsecond.time_error = flag1_micrsecond.time_now - flag1_micrsecond.time_last;
								
							}
							if(flag1_micrsecond.time_error>950000&&flag1_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;
								flag1_micrsecond.flag = 0;
							}
							{//�������
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							Position_set(height_s,up);//̧��
							if(LM1PositionPID.ref>=(height_s-5))
							{
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}		
							}
						}
					}break;
					case MININGRETURN2_STATE: //�ӷ���״̬2�л����ɿ�״̬3���ڶ��βɿ�ɹ���
					{
						{//̧������
						
						}
						{
//								if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
//								else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//								{
//									flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//								}
						}
						{//PID
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						{//�������
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							Position_set(height_s,up);//̧��
							if(LM1PositionPID.ref>=(height_s-5))
							{
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}		
						}
				}break;
					case MININGRETURN3_STATE://�ӷ���״̬3�л����ɿ�״̬3�������βɿ�ʧ�ܣ�
					{
						{//̧������
						
						}
						{//PID
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						{//�������
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ���צ
							GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�Ƴ�
							Position_set(height_s,up);//̧��
							if(LM1PositionPID.ref>=(height_s-5))
							{
								Claw_Position_set(90,out);//��ת�Ƕ�
								if(Mouse_l_Flag==1) //���
								{
									Claw_Position_set(claw_out_angel,out);
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
								{
									Claw_Position_set(90,back);
								}
							}		
						}
					}
					default:break;
				}
			}		
			case MININGRETURN1_STATE:
			{
				flag_kj_sz=0;
				flag_kj_ts=0;
				{//��צ����
					
				}
				switch(lastminingTaskStatus)
				{
					case MINING1_STATE: //�Ӳɿ�״̬1�л�������״̬1
					{
						{//��צ����800
//							if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
//							else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
						}
						if(TM2Encoder.ecd_angle>90)
						{
							TM1SpeedPID.kp=2.0f;
					    TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=5.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=2.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=5.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}
						else if(TM2Encoder.ecd_angle<=90)
						{
							TM1SpeedPID.kp=-1.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=0.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=-1.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=0.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}

							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						
						{//���嶯��
							  GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
								Position_set(height_s_p,up);//̧��
								if(LM1PositionPID.ref>=(height_s_p-5))
								{
									Claw_Position_set_slow(0,back);//��ת�Ƕ�
								}

						}
					}break;
					default:break;
				}
			}break;
			case MININGRETURN2_STATE:
			{
				{//��צ����
					
				}
				flag_kj_sz=0;
				flag_kj_ts=0;
				switch(lastminingTaskStatus)
				{
					case MINING2_STATE: //�Ӳɿ�״̬2�л�������״̬2
					{
						{//PID
						{//��צ����800
//							if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
//							else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
						}
						if(TM2Encoder.ecd_angle>90)
						{
							TM1SpeedPID.kp=2.0f;
					    TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=5.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=2.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=5.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}
						else if(TM2Encoder.ecd_angle<=90)
						{
							TM1SpeedPID.kp=-1.0f;
					        TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=0.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=-1.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=0.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						{//���嶯��
							  GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
								Position_set(height_s_p,up);//̧��
								if(LM1PositionPID.ref>=(height_s_p-5))
								{
									Claw_Position_set_slow(0,back);//��ת�Ƕ�
								}
						}
					}break;
					default:break;
				}
			}break;
			case MININGRETURN3_STATE:
			{
				{//��צ����
					
				}
				flag_kj_sz=0;
				flag_kj_ts=0;
				switch(lastminingTaskStatus)
				{
					case MINING3_STATE: //�Ӳɿ�״̬3�л�������״̬3
					{
						{//PID
						{//��צ����800
//							if(TM2Encoder.ecd_angle>=(-10)&&TM2Encoder.ecd_angle<=90)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
//							else if(TM2Encoder.ecd_angle>90&&TM2Encoder.ecd_angle<=270)
//							{
//								flipCompensation=800.0f*cos(TM2Encoder.ecd_angle*3.1415926f/180);
//							}
						}
						if(TM2Encoder.ecd_angle>90)
						{
							TM1SpeedPID.kp=2.0f;
					        TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=5.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=2.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=5.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}
						else if(TM2Encoder.ecd_angle<=90)
						{
							TM1SpeedPID.kp=-1.0f;
					    TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=0;
							TM1PositionPID.kp=0.0f;
							TM1PositionPID.ki=0;
							TM1PositionPID.kd=0;
							TM2SpeedPID.kp=-1.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=0;
							TM2PositionPID.kp=0.0f;
							TM2PositionPID.ki=0;
							TM2PositionPID.kd=0;
						}
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
						}
						{//���嶯��
						  GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
						  GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�ջ���צ
							Position_set(height_s_p,up);//̧��
							if(LM1PositionPID.ref>=(height_s_p-5))
							{
									Claw_Position_set_slow(60,back);//��ת�Ƕ�
							}
							Position_set_slow(0,down);
						}
					}break;
					default:break;
				}
			}break;
			case MININGEXCHANGE_STATE:
			{//R���������ɿ���ʯ,��צ90��
				switch(lastminingTaskStatus)
				{
					flag_qk3=0;
					flag_qudi=0;
					case MINING_NOTASK_STATE:  //��������״̬�л����һ�״̬
					{
						{//PID
							TM1SpeedPID.kp=8.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=8.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
						
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
								
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
					}
					{//���嶯��
							GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�ɿ���ʯ
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ɿ���ʯ
							Claw_Position_set(90,out);//��ת�Ƕ�
						
               if(getMINERAL() == 3||1)
							 {
								if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
							 if(getMINERAL() == 2)
							 {
								 if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = csd_speed;	
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = -csd_speed;	
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
							 
						}  
				
					}break;
					case MININGPUSH_STATE:
					{
						{//PID
							TM1SpeedPID.kp=8.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=8.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
						
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
					}
					{//���嶯��
							GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�ɿ���ʯ
							Claw_Position_set(90,back);//��ת�Ƕ�
						  GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ɿ���ʯ
							 if(getMINERAL() == 3||1)
							 {
								if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
							 if(getMINERAL() == 2)
							 {
								 if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = csd_speed;	
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = -csd_speed;	
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
					}
					}break;
					case MININGOBTAIN_STATE:  //��ȡ����״̬�л����һ�״̬
					{
						{//PID
							TM1SpeedPID.kp=8.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=8.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
						
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
								
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
					}
					{//���嶯��
							GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�ɿ���ʯ
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ɿ���ʯ
							Claw_Position_set(90,back);//��ת�Ƕ�
						  flag_ex=0;
						
               if(getMINERAL() == 3||1)
							 {
								if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = 0;
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
							 if(getMINERAL() == 2)
							 {
								 if(Mouse_l_Flag==1) //���
							   	{
									TM_Ref_3 = -csd_speed;
									TM_Ref_4 = csd_speed;	
								}
								else if(Mouse_r_Flag==1) //�Ҽ�
							  	{
									TM_Ref_3 = csd_speed;
									TM_Ref_4 = -csd_speed;	
								}
								else if(Mouse_l_Flag==0)
								{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
								}
								else if(Mouse_r_Flag==0)
						    	{
									TM_Ref_3 = 0;
									TM_Ref_4 = 0;	
							    }
							 }
							 
						}  
				
					}break;
				
				}
			}break;
			case MININGEXCHANGE2_STATE:
			{
					switch(lastminingTaskStatus)
				{
					case MININGRETURN3_STATE:  //�ӷ���3״̬�л����һ�״̬
					{
						{//PID
							TM1SpeedPID.kp=8.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=8.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=0;
						
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
								
				    	TM3SpeedPID.kp=8;
					    TM4SpeedPID.kp=8;
					    TM3SpeedPID.ki=0;
					    TM4SpeedPID.ki=0;
					    TM3SpeedPID.kd=0;
				    	TM4SpeedPID.kd=0;
					}
						{//���嶯��
							GPIO_SetBits(GPIOH,GPIO_Pin_2);//�����׼н�
							GPIO_SetBits(GPIOH,GPIO_Pin_3);//�н���ʯ
							Position_set(put_height_first,up);
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								Claw_Position_set_slow(180,out);
							}
						}
					}break;
				}
			}break;
			case MININGOBTAIN_STATE:
			{//F����ȡ����+��̨��
				switch(lastminingTaskStatus)
				{
					case MININGEXCHANGE_STATE:  //�Ӷһ�״̬�л���ȡ����״̬
					{
						
					{//PID
							TM1SpeedPID.kp=8.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=8.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
						
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
								
							TM3SpeedPID.kp=8;
							TM4SpeedPID.kp=8;
							TM3SpeedPID.ki=0;
							TM4SpeedPID.ki=0;
							TM3SpeedPID.kd=0;
							TM4SpeedPID.kd=0;
					}
						

						
					if(flag_ex==0)
						{
							GPIO_ResetBits(GPIOH,GPIO_Pin_3);//Ĭ��ץȡ������ǰ�ɿ���ʯ
							Claw_Position_set_slow(0,back);//צ�ӷ���ȡ���ڿ�ʯ
						}	
						
						if(TM2Encoder.ecd_angle<=1||flag_ex==1)
						{
							flag_ex=1;
							Position_set(put_height_first,up);
							GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
							
							if(LM1PositionPID.ref>=(put_height_first-5))
							{
								Claw_Position_set_slow(180,out);
							}
							
						}

							if(getMINERAL()==2&&flag2_micrsecond.flag==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н������ɿ�
								TM_Ref_3 = -csd_speed;
								TM_Ref_4 = csd_speed;
								flag2_micrsecond.flag = 1;
								flag2_micrsecond.time_last = Get_Time_Micros();
							}
							if(flag2_micrsecond.flag==1)
							{
								flag2_micrsecond.time_now = Get_Time_Micros();
								flag2_micrsecond.time_error = flag2_micrsecond.time_now - flag2_micrsecond.time_last;
							}
							if(flag2_micrsecond.time_error>95000&&flag2_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;	
								flag2_micrsecond.flag = 0;
							}
					
					}break;
					case MININGRETURN4_STATE:  //�ӷ���״̬4�л���ȡ����״̬
					{
						
						Claw_Position_set_slow(0,back);//צ�ӷ���ȡ���ڿ�ʯ
						GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
							if(flag2_micrsecond.flag == 0)
							{
								flag2_micrsecond.flag = 1;
								flag2_micrsecond.time_last = Get_Time_Micros();
							}

							if(getMINERAL()==2&&flag2_micrsecond.flag==1)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_2);//�н������ɿ�
								TM_Ref_3 = -csd_speed;
								TM_Ref_4 = csd_speed;
								flag2_micrsecond.time_now = Get_Time_Micros();
								flag2_micrsecond.time_error = flag2_micrsecond.time_now - flag2_micrsecond.time_last;
							}
							if(flag2_micrsecond.time_error>23000&&flag2_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;	
								flag2_micrsecond.flag = 0;
							}
					}break;
					default:break;
				}
			}break;
			case MININGPUSH_STATE:
			{//shift���������ʯ
				flag_ex=0;
				flag_ob=0;
				switch(lastminingTaskStatus)
				{
					case MININGOBTAIN_STATE:  //��ȡ����״̬�л���PUSH״̬
					{
						if(Shift_Flag_time_tick_1ms>=300)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ɿ�צ��
								GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�ջ�צ��
							}
							else
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�����ʯ
							}
						
							if(flag2_micrsecond.flag==1)
							{
								flag2_micrsecond.time_now = Get_Time_Micros();
								flag2_micrsecond.time_error = flag2_micrsecond.time_now - flag2_micrsecond.time_last;
							}
							if(flag2_micrsecond.time_error>1700000&&flag2_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;	
								flag2_micrsecond.flag = 0;
							}
						
					}break;
					case MININGEXCHANGE2_STATE:  //��ȡ����״̬�л���PUSH״̬
					{
						if(Shift_Flag_time_tick_1ms>=300)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ɿ�צ��
								GPIO_ResetBits(GPIOH,GPIO_Pin_4);//�ջ�צ��
							}
							else
							{
								GPIO_SetBits(GPIOH,GPIO_Pin_3);//����צ��
								GPIO_SetBits(GPIOH,GPIO_Pin_4);//�����ʯ
							}
							
						
					}break;
				}
			}
			case MININGGROUND_STATE:
			{//SHIFT����ȡ��һϵ�ж���
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л���ȡ��״̬
					{
						{//PID
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							TM5SpeedPID.kp=8.0f;
							TM5SpeedPID.ki=0;
							TM5SpeedPID.kd=10.0f;
							TM5PositionPID.kp=8.0f;
							TM5PositionPID.ki=0.01;
							TM5PositionPID.kd=5;
							TM6SpeedPID.kp=8.0f;
							TM6SpeedPID.ki=0;
							TM6SpeedPID.kd=10.0f;
							TM6PositionPID.kp=8.0f;
							TM6PositionPID.ki=0.01;
							TM6PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
						}
						{//���嶯��

							
							
						if(flag_qudi==0)
							{
								Claw_Position_set_ground(125,out);
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								if(F_Flag==1)
							{flag_qudi=1;}
							}
						if(flag_qudi==1)
						{
							if(TM6Encoder.ecd_angle>=120&&TM2Encoder.ecd_angle>=200&&flag6_micrsecond_qd1.flag==0)
							{
								flag6_micrsecond_qd1.flag=1;
								flag6_micrsecond_qd1.time_last= Get_Time_Micros();
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								//Position_set(100,up);
							}
							else if(flag6_micrsecond_qd1.flag==0)
							{
								Claw_Position_set(210,out);
								Claw_Position_set_ground(125,out);
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								//Position_set(100,up);
							}
						
							
							if(flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd1.time_error<100000)
							{
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								//Position_set(100,up);
							}
							if(flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0&&flag6_micrsecond_qd1.time_error>=100000)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�պ�����צ
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
							}
							if(flag6_micrsecond_qd1.time_error>400000&&flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0)
							{
								Claw_Position_set_ground(66,back);
							
							}
							if(TM6Encoder.ecd_angle<=68&&flag6_micrsecond_qd1.flag==1)
							{
								Claw_Position_set_ground(66,back);
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								if(flag6_micrsecond_qd2.flag==0)
								{
									flag6_micrsecond_qd2.flag=1;
									flag6_micrsecond_qd2.time_last= Get_Time_Micros();
								}
								if(flag6_micrsecond_qd2.flag==1)
								{
									flag6_micrsecond_qd2.time_now = Get_Time_Micros();
									flag6_micrsecond_qd2.time_error = flag6_micrsecond_qd2.time_now - flag6_micrsecond_qd2.time_last;
								}
								if(flag6_micrsecond_qd2.time_error>500000&&flag6_micrsecond_qd2.flag==1)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								}
							}
							if(flag6_micrsecond_qd2.time_error>800000&&flag6_micrsecond_qd2.flag==1)
							{
								Claw_Position_set_slow(0,back);          //����צ�պϺ󷭻�
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								//Position_set(0,down);
								//flag6_micrsecond_qd1.flag=0;
							}
						}
						}
					}break;
					default:break;
				}
			}break;
			case MININGGROUND1_STATE:
			{//SHIFT����ȡ��1һϵ�ж���
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л���ȡ��״̬
					{
						{//PID
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							TM5SpeedPID.kp=8.0f;
							TM5SpeedPID.ki=0;
							TM5SpeedPID.kd=10.0f;
							TM5PositionPID.kp=8.0f;
							TM5PositionPID.ki=0.01;
							TM5PositionPID.kd=5;
							TM6SpeedPID.kp=8.0f;
							TM6SpeedPID.ki=0;
							TM6SpeedPID.kd=10.0f;
							TM6PositionPID.kp=8.0f;
							TM6PositionPID.ki=0.01;
							TM6PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
						}
						{//���ʹ�
							if( getMINERAL() == 1)
							{	                       
								TM_Ref_3 = csd_speed;
								TM_Ref_4 = -csd_speed;
							}
							else if(getMINERAL() == 2 && flag1_micrsecond.flag == 0)
							{
								flag1_micrsecond.flag = 1;
								flag1_micrsecond.time_last = Get_Time_Micros();
							}
							if(flag1_micrsecond.flag==1)
							{
								flag1_micrsecond.time_now = Get_Time_Micros();
								flag1_micrsecond.time_error = flag1_micrsecond.time_now - flag1_micrsecond.time_last;
								
							}
							if(flag1_micrsecond.time_error>950000&&flag1_micrsecond.flag==1)
							{
								TM_Ref_3 = 0;
								TM_Ref_4 = 0;
								flag1_micrsecond.flag = 0;
							}
						}
						{//���嶯��

							if(flag_qudi==0)
							{
								Claw_Position_set_ground(125,out);
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								if(F_Flag==1)
							{flag_qudi=1;}
							}
						if(flag_qudi==1)
						{
							if(TM6Encoder.ecd_angle>=120&&TM2Encoder.ecd_angle>=200&&flag6_micrsecond_qd1.flag==0)
							{
								flag6_micrsecond_qd1.flag=1;
								flag6_micrsecond_qd1.time_last= Get_Time_Micros();
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							else if(flag6_micrsecond_qd1.flag==0)
							{
								Claw_Position_set(210,out);
								Claw_Position_set_ground(125,out);
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							if(flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd1.time_error<100000)
							{
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							if(flag6_micrsecond_qd1.time_error>=100000&&flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�պ�����צ
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
							}
							if(flag6_micrsecond_qd1.time_error>400000&&flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0)
							{
								Claw_Position_set_ground(66,back);
							}
							if(TM6Encoder.ecd_angle<=68&&flag6_micrsecond_qd1.flag==1)
							{
								Claw_Position_set_ground(66,back);
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								if(flag6_micrsecond_qd2.flag==0)
								{
									flag6_micrsecond_qd2.flag=1;
									flag6_micrsecond_qd2.time_last= Get_Time_Micros();
								}
								if(flag6_micrsecond_qd2.flag==1)
								{
									flag6_micrsecond_qd2.time_now = Get_Time_Micros();
									flag6_micrsecond_qd2.time_error = flag6_micrsecond_qd2.time_now - flag6_micrsecond_qd2.time_last;
								}
								if(flag6_micrsecond_qd2.time_error>700000&&flag6_micrsecond_qd2.flag==1)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								}
							}
							if(flag6_micrsecond_qd2.time_error>1000000&&flag6_micrsecond_qd2.flag==1)
							{
								Claw_Position_set_slow(0,back);          //����צ�պϺ󷭻�
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								//flag6_micrsecond_qd1.flag=0;
							}
						}
						}
					}break;
					default:break;
				}
			}break;
			case MININGGROUND2_STATE:
			{//SHIFT����ȡ��2һϵ�ж���
				switch(lastminingTaskStatus)
				{
					case MINING_NOTASK_STATE:  //��������״̬�л���ȡ��״̬
					{
						{//PID
							TM1SpeedPID.kp=20.0f;
							TM1SpeedPID.ki=0;
							TM1SpeedPID.kd=10.0f;
							TM1PositionPID.kp=8.0f;
							TM1PositionPID.ki=0.01;
							TM1PositionPID.kd=5;
							TM2SpeedPID.kp=20.0f;
							TM2SpeedPID.ki=0;
							TM2SpeedPID.kd=10.0f;
							TM2PositionPID.kp=8.0f;
							TM2PositionPID.ki=0.01;
							TM2PositionPID.kd=5;
							
							TM5SpeedPID.kp=8.0f;
							TM5SpeedPID.ki=0;
							TM5SpeedPID.kd=10.0f;
							TM5PositionPID.kp=8.0f;
							TM5PositionPID.ki=0.01;
							TM5PositionPID.kd=5;
							TM6SpeedPID.kp=8.0f;
							TM6SpeedPID.ki=0;
							TM6SpeedPID.kd=10.0f;
							TM6PositionPID.kp=8.0f;
							TM6PositionPID.ki=0.01;
							TM6PositionPID.kd=5;
							
							LM1SpeedPID.kp=5;
							LM2SpeedPID.kp=5;
							LM1SpeedPID.ki=0;
							LM2SpeedPID.ki=0;
							LM1SpeedPID.kd=5;
							LM2SpeedPID.kd=5;
							LM1PositionPID.kp=2;
							LM2PositionPID.kp=2;
							LM1PositionPID.ki=0;
							LM2PositionPID.ki=0;
							LM1PositionPID.kd=10;
							LM2PositionPID.kd=10;
							LM1PositionPID.kd=5;
							LM2PositionPID.kd=5;
							
						}
						{//���嶯��

							if(flag_qudi==0)
							{
								Claw_Position_set_ground(125,out);
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								if(F_Flag==1)
							{flag_qudi=1;}
							}
						if(flag_qudi==1)
						{
							if(TM6Encoder.ecd_angle>=120&&TM2Encoder.ecd_angle>200&&flag6_micrsecond_qd1.flag==0)
							{
								flag6_micrsecond_qd1.flag=1;
								flag6_micrsecond_qd1.time_last= Get_Time_Micros();
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							else if(flag6_micrsecond_qd1.flag==0)
							{
								Claw_Position_set(210,out);
								Claw_Position_set_ground(125,out);
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							if(flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd1.time_error<100000)
							{
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
							}
							if(flag6_micrsecond_qd1.time_error>=100000&&flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0)
							{
								GPIO_ResetBits(GPIOH,GPIO_Pin_3);//�ſ�����צ
								GPIO_ResetBits(GPIOH,GPIO_Pin_5);//�պ�����צ
								flag6_micrsecond_qd1.time_now = Get_Time_Micros();
								flag6_micrsecond_qd1.time_error = flag6_micrsecond_qd1.time_now - flag6_micrsecond_qd1.time_last;
							}
							if(flag6_micrsecond_qd1.time_error>400000&&flag6_micrsecond_qd1.flag==1&&flag6_micrsecond_qd2.flag==0)
							{
								Claw_Position_set_ground(66,back);
							}
							if(TM6Encoder.ecd_angle<=68&&flag6_micrsecond_qd1.flag==1)
							{
								Claw_Position_set_ground(66,back);
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								if(flag6_micrsecond_qd2.flag==0)
								{
									flag6_micrsecond_qd2.flag=1;
									flag6_micrsecond_qd2.time_last= Get_Time_Micros();
								}
								if(flag6_micrsecond_qd2.flag==1)
								{
									flag6_micrsecond_qd2.time_now = Get_Time_Micros();
									flag6_micrsecond_qd2.time_error = flag6_micrsecond_qd2.time_now - flag6_micrsecond_qd2.time_last;
								}
								if(flag6_micrsecond_qd2.time_error>700000&&flag6_micrsecond_qd2.flag==1)
								{
									GPIO_SetBits(GPIOH,GPIO_Pin_5);//�ſ�����צ
								}
							}
							if(flag6_micrsecond_qd2.time_error>1000000&&flag6_micrsecond_qd2.flag==1)
							{
								Claw_Position_set_slow(60,back);          //����צ�պϺ󷭻�
								GPIO_SetBits(GPIOH,GPIO_Pin_3);  //�պ�����צ
								//flag6_micrsecond_qd1.flag=0;
							}
						}
						}
					}break;
					default:break;
				}
			}break;
		  default:break;
		}
	}
//			LM1PositionPID.ref = LM_Ref;
//			LM2PositionPID.ref = -LM_Ref;
	{//������Ϣ����
		{//̧�����
			LM1_Data = LM1Encoder.ecd_angle;
			LM1_Data = KalmanFilter(&p_LM1,LM1_Data);
			LM2_Data = LM2Encoder.ecd_angle;
			LM2_Data = KalmanFilter(&p_LM2,LM2_Data);
			LM1PositionPID.ref = LM_Ref;
			LM2PositionPID.ref = -LM_Ref;
			LM1PositionPID.fdb = LM1Encoder.ecd_angle;
			LM2PositionPID.fdb = LM2Encoder.ecd_angle;
			LM1PositionPID.fdb = LM1_Data;
			LM2PositionPID.fdb = LM2_Data;
			LM1PositionPID.Calc(&LM1PositionPID);
			LM2PositionPID.Calc(&LM2PositionPID);
			LM1SpeedPID.ref = LM1PositionPID.output*5;
			LM2SpeedPID.ref = LM2PositionPID.output*5;
			LM1SpeedPID.fdb = LM1Encoder.filter_rate;
			LM2SpeedPID.fdb = LM2Encoder.filter_rate;
			LM1SpeedPID.Calc(&LM1SpeedPID);
			LM2SpeedPID.Calc(&LM2SpeedPID);
			LM1SpeedPID.output +=upliftCompensation_1;
			LM2SpeedPID.output -=upliftCompensation_2;
			LM1SpeedPID.output = VAL_LIMITF(LM1SpeedPID.output,-16384,16384);
			LM2SpeedPID.output = VAL_LIMITF(LM2SpeedPID.output,-16384,16384);
		}
		
		{//��ת���
			TM1PositionPID.ref = -TM_Ref_0;
			TM2PositionPID.ref = TM_Ref_0;
			TM1PositionPID.fdb = TM1Encoder.ecd_angle;
			TM2PositionPID.fdb = TM2Encoder.ecd_angle;
			TM1PositionPID.Calc(&TM1PositionPID);
			TM2PositionPID.Calc(&TM2PositionPID);
			TM1SpeedPID.ref = TM1PositionPID.output*5;
			TM2SpeedPID.ref = TM2PositionPID.output*5;
			TM1SpeedPID.fdb = TM1Encoder.filter_rate;
			TM2SpeedPID.fdb = TM2Encoder.filter_rate;
			TM1SpeedPID.Calc(&TM1SpeedPID);
			TM2SpeedPID.Calc(&TM2SpeedPID);
			TM1SpeedPID.output -=flipCompensation;
			TM2SpeedPID.output +=flipCompensation;
			TM1SpeedPID.output = VAL_LIMITF(TM1SpeedPID.output,-5000,5000);
			TM2SpeedPID.output = VAL_LIMITF(TM2SpeedPID.output,-5000,5000);
		}
		
		{//����צ��ת���
			TM5PositionPID.ref = -TM_Ref_1;
			TM6PositionPID.ref = TM_Ref_1;
			TM5PositionPID.fdb = TM5Encoder.ecd_angle;
			TM6PositionPID.fdb = TM6Encoder.ecd_angle;
			TM5PositionPID.Calc(&TM5PositionPID);
			TM6PositionPID.Calc(&TM6PositionPID);
			TM5SpeedPID.ref = TM5PositionPID.output*5;
			TM6SpeedPID.ref = TM6PositionPID.output*5;
			TM5SpeedPID.fdb = TM5Encoder.filter_rate;
			TM6SpeedPID.fdb = TM6Encoder.filter_rate;
			TM5SpeedPID.Calc(&TM5SpeedPID);
			TM6SpeedPID.Calc(&TM6SpeedPID);
			TM5SpeedPID.output -=flipCompensation;
			TM6SpeedPID.output +=flipCompensation;
			TM5SpeedPID.output = VAL_LIMITF(TM5SpeedPID.output,-5000,5000);
			TM6SpeedPID.output = VAL_LIMITF(TM6SpeedPID.output,-5000,5000);
		}
		
		{//ȡ�ϰ�����
			TM7PositionPID.ref = -TM_Ref_5;
			TM8PositionPID.ref = TM_Ref_5;
			TM7PositionPID.fdb = TM7Encoder.ecd_angle;
			TM8PositionPID.fdb = TM8Encoder.ecd_angle;
			TM7PositionPID.Calc(&TM7PositionPID);
			TM8PositionPID.Calc(&TM8PositionPID);
			TM7SpeedPID.ref = TM7PositionPID.output*5;
			TM8SpeedPID.ref = TM8PositionPID.output*5;
			TM7SpeedPID.fdb = TM7Encoder.filter_rate;
			TM8SpeedPID.fdb = TM8Encoder.filter_rate;
			TM7SpeedPID.Calc(&TM7SpeedPID);
			TM8SpeedPID.Calc(&TM8SpeedPID);
			TM7SpeedPID.output -=flipCompensation;
			TM8SpeedPID.output +=flipCompensation;
			TM7SpeedPID.output = VAL_LIMITF(TM7SpeedPID.output,-5000,5000);
			TM8SpeedPID.output = VAL_LIMITF(TM8SpeedPID.output,-5000,5000);
		}
		
		{//���ʹ����
			TM3SpeedPID.ref = TM_Ref_3;
			TM3SpeedPID.fdb = TM3Encoder.filter_rate;
			TM3SpeedPID.Calc(&TM3SpeedPID);
			TM3SpeedPID.output = VAL_LIMITF(TM3SpeedPID.output,-5000,5000);
			
			TM4SpeedPID.ref = TM_Ref_4;
			TM4SpeedPID.fdb = TM4Encoder.filter_rate;
			TM4SpeedPID.Calc(&TM4SpeedPID);
			TM4SpeedPID.output = VAL_LIMITF(TM4SpeedPID.output,-5000,5000);
		}

	}
}break;
	}
}

	void MotorExecution()
{

	Set_CM_Speed(CAN1,CM1SpeedPID.output,CM2SpeedPID.output,CM3SpeedPID.output,CM4SpeedPID.output); //���̵��
	Set_XM_Speed(CAN1,LM1SpeedPID.output,1.2*LM2SpeedPID.output,0,0); //̧�����+���ʹ�
	Set_XM_SpeedPlus(CAN2,TM1SpeedPID.output,TM2SpeedPID.output,TM3SpeedPID.output,TM4SpeedPID.output);//��ת���
	Set_XM_SpeedPlus2(CAN2,TM5SpeedPID.output,TM6SpeedPID.output,TM7SpeedPID.output,TM8SpeedPID.output);//��ת���2
}

//����ϵͳ����������CM1234
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(m1_iq >> 8);
    tx_message.Data[1] = (uint8_t)m1_iq;
    tx_message.Data[2] = (uint8_t)(m2_iq >> 8);
    tx_message.Data[3] = (uint8_t)m2_iq;
    tx_message.Data[4] = (uint8_t)(m3_iq >> 8);
    tx_message.Data[5] = (uint8_t)m3_iq;
    tx_message.Data[6] = (uint8_t)(m4_iq >> 8);
    tx_message.Data[7] = (uint8_t)m4_iq;
    CAN_Transmit(CANx,&tx_message);
}

//̧������������LM12\TM4
void Set_XM_Speed(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(m1_iq >> 8);
    tx_message.Data[1] = (uint8_t)m1_iq;
    tx_message.Data[2] = (uint8_t)(m2_iq >> 8);
    tx_message.Data[3] = (uint8_t)m2_iq;
    tx_message.Data[4] = (uint8_t)(m3_iq >> 8);
    tx_message.Data[5] = (uint8_t)m3_iq;
    tx_message.Data[6] = (uint8_t)(m4_iq >> 8);
    tx_message.Data[7] = (uint8_t)m4_iq;
    CAN_Transmit(CANx,&tx_message);
}

//CAN2����������TM123
void Set_XM_SpeedPlus(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(m1_iq >> 8);
    tx_message.Data[1] = (uint8_t)m1_iq;
    tx_message.Data[2] = (uint8_t)(m2_iq >> 8);
    tx_message.Data[3] = (uint8_t)m2_iq;
    tx_message.Data[4] = (uint8_t)(m3_iq >> 8);
    tx_message.Data[5] = (uint8_t)m3_iq;
    tx_message.Data[6] = (uint8_t)(m4_iq >> 8);
    tx_message.Data[7] = (uint8_t)m4_iq;
    CAN_Transmit(CANx,&tx_message);
}
//CAN2����������TM5678
void Set_XM_SpeedPlus2(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(m1_iq >> 8);
    tx_message.Data[1] = (uint8_t)m1_iq;
    tx_message.Data[2] = (uint8_t)(m2_iq >> 8);
    tx_message.Data[3] = (uint8_t)m2_iq;
    tx_message.Data[4] = (uint8_t)(m3_iq >> 8);
    tx_message.Data[5] = (uint8_t)m3_iq;
    tx_message.Data[6] = (uint8_t)(m4_iq >> 8);
    tx_message.Data[7] = (uint8_t)m4_iq;
    CAN_Transmit(CANx,&tx_message);
}

void Position_set(float position,PositionMode mode)//̧���½�����
{
	if(mode == up)
	{
		if(LM_Ref<=position)
		{
			LM_Ref+=1.2f;//0.9f
		}
	}
	else if(mode == down)
	{
		if(LM_Ref>=position)
		{
			LM_Ref-=0.9f;
		}
	}
	if(LM_Ref<0)
	{     
		LM_Ref=0;
	}
	else if(LM_Ref>=1050)
	{
		LM_Ref=1050;
	}
}
void Position_set_slow(float position,PositionMode mode)//̧���½�����
{
	if(mode == up)
	{
		if(LM_Ref<=position)
		{
			LM_Ref+=0.9f;//0.9f
		}
	}
	else if(mode == down)
	{
		if(LM_Ref>=position)
		{
			LM_Ref-=0.6f;
		}
	}
	if(LM_Ref<0)
	{
		LM_Ref=0;
	}
	else if(LM_Ref>=1050)
	{
		LM_Ref=1050;
	}
}
void Claw_Position_set(float position,ClawPositionMode mode)//��צ��������
{
	if(mode == out)
	{
		if(TM_Ref_0<=position)
		{
			TM_Ref_0+=0.4f;
		}
	}
	else if(mode == back)
	{
		if(TM_Ref_0>=position)
		{
			TM_Ref_0-=0.4f;
		}
	}
	if(TM_Ref_0<0)
	{
		TM_Ref_0=0;
	}
	else if(TM_Ref_0>270)
	{
		TM_Ref_0=270;
	}
}

void Claw_Position_set_slow2(float position,ClawPositionMode mode)//��צ�������������٣�����
{
	if(mode == out)
	{
		if(TM_Ref_0<=position)
		{
			TM_Ref_0+=0.05f;
		}
	}
	else if(mode == back)
	{
		if(TM_Ref_0>=position)
		{
			TM_Ref_0-=0.05f;
		}
	}
	if(TM_Ref_0>0)
	{
		TM_Ref_0=0;
	}
	else if(TM_Ref_0<-270)
	{
		TM_Ref_0=-270;
	}
}


void Claw_Position_set_slow(float position,ClawPositionMode mode)//��צ�������������٣�
{
	if(mode == out)
	{
		if(TM_Ref_0<=position)
		{
			TM_Ref_0+=0.2f;
		}
	}
	else if(mode == back)
	{
		if(TM_Ref_0>=position)
		{
			TM_Ref_0-=0.2f;
		}
	}
	if(TM_Ref_0<0)
	{
		TM_Ref_0=0;
	}
	else if(TM_Ref_0>270)
	{
		TM_Ref_0=270;
	}
}

void Claw_Position_set_ground(float position,ClawPositionMode mode)//�Ե���צ��������
{
	if(mode == out)
	{
		if(TM_Ref_1<=position)
		{
			TM_Ref_1+=0.3f;
		}
	}
	else if(mode == back)
	{
		if(TM_Ref_1>=position)
		{
			TM_Ref_1-=0.3f;
		}
	}
	if(TM_Ref_1<0)
	{
		TM_Ref_1=0;
	}
	else if(TM_Ref_1>180)
	{
		TM_Ref_1=180;
	}
}

void Claw_Position_set_ground_slow2(float position,ClawPositionMode mode)//�Ե���צ��������
{
	if(mode == out)
	{
		if(TM_Ref_1<=position)
		{
			TM_Ref_1+=0.05f;
		}
	}
	else if(mode == back)
	{
		if(TM_Ref_1>=position)
		{
			TM_Ref_1-=0.05f;
		}
	}
	if(TM_Ref_1>0)
	{
		TM_Ref_1=0;
	}
	else if(TM_Ref_0<-125)
	{
		TM_Ref_0=-125;
	}
}


void Barrier_set(float position,ClawPositionMode mode)//ȡ�ϰ��鶯������
{
	if(mode == out)
	{
		if(TM_Ref_5<=position)
		{
			TM_Ref_5+=0.4f;
		}
	}
	else if(mode == back)
	{
		if(TM_Ref_5>=position)
		{
			TM_Ref_5-=0.4f;
		}
	}
	if(TM_Ref_5<0)
	{
		TM_Ref_5=0;
	}
	else if(TM_Ref_5>135)
	{
		TM_Ref_5=135;
	}
}



void motorstop()//���������ֹͣ����Ҫ������¼���룬��������¹��̳��ص���ʼ״̬���л�����ͣ״̬
{
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
	PID_Reset(&TM7PositionPID);
	PID_Reset(&TM8PositionPID);
	PID_Reset(&TM3SpeedPID);
	PID_Reset(&TM4SpeedPID);
	PID_Reset(&TM5SpeedPID);
	PID_Reset(&TM6SpeedPID);
	PID_Reset(&TM7SpeedPID);
	PID_Reset(&TM8SpeedPID);
//	EncoderReset(&CM1Encoder);
//	EncoderReset(&CM2Encoder);
//	EncoderReset(&CM3Encoder);
//	EncoderReset(&CM4Encoder);
//	EncoderReset(&LM1Encoder);
//	EncoderReset(&LM2Encoder);
//	EncoderReset(&TM1Encoder);
//	EncoderReset(&TM2Encoder);
//	EncoderReset(&TM3Encoder);
//	EncoderReset(&TM4Encoder);
//	EncoderReset(&TM5Encoder);
//	EncoderReset(&TM6Encoder);
//	EncoderReset(&TM7Encoder);
}

void motorprepare()
{
	//��ʱû������
	//����ϵͳ���
	CM1SpeedPID.ref = 0;
	CM2SpeedPID.ref = 0;
	CM3SpeedPID.ref = 0;
	CM4SpeedPID.ref = 0;
	CM1SpeedPID.output = 0;
	CM2SpeedPID.output = 0;
	CM3SpeedPID.output = 0;
	CM4SpeedPID.output = 0;
}

void motornormal()
{
}

void chassis_system()
{
			//����ϵͳ���
			ChassisSpeedRef.rotate_ref = ChassisSpeedRef.rotate_ref = VAL_LIMIT(ChassisSpeedRef.rotate_ref,-350,350);
			CM1SpeedPID.ref = 1*(-ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref);
			CM2SpeedPID.ref = 1*(ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref);
			CM3SpeedPID.ref = 1*(ChassisSpeedRef.forward_back_ref+ ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref);
			CM4SpeedPID.ref = 1*(-ChassisSpeedRef.forward_back_ref+ ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref);
			CM1SpeedPID.fdb = CM1Encoder.filter_rate;
			CM2SpeedPID.fdb = CM2Encoder.filter_rate;
			CM3SpeedPID.fdb = CM3Encoder.filter_rate;
			CM4SpeedPID.fdb = CM4Encoder.filter_rate;
			CM1SpeedPID.Calc(&CM1SpeedPID);
			CM2SpeedPID.Calc(&CM2SpeedPID);
			CM3SpeedPID.Calc(&CM3SpeedPID);
			CM4SpeedPID.Calc(&CM4SpeedPID);
			CM1SpeedPID.output = VAL_LIMITF(CM1SpeedPID.output,-gl,gl);
			CM2SpeedPID.output = VAL_LIMITF(CM2SpeedPID.output,-gl,gl);
			CM3SpeedPID.output = VAL_LIMITF(CM3SpeedPID.output,-gl,gl);
			CM4SpeedPID.output = VAL_LIMITF(CM4SpeedPID.output,-gl,gl);
//			CM1SpeedPID.output = VAL_LIMITF(CM1SpeedPID.output,-16384,16384);
//			CM2SpeedPID.output = VAL_LIMITF(CM2SpeedPID.output,-16384,16384);
//			CM3SpeedPID.output = VAL_LIMITF(CM3SpeedPID.output,-16384,16384);
//			CM4SpeedPID.output = VAL_LIMITF(CM4SpeedPID.output,-16384,16384);


//		Speed_Limit.kp = 1;
//		Speed_Limit.ki = 1;
//		Speed_Limit.kd = 0;
//		Speed_Limit.ref = 16384;
//		Speed_Limit.fdb = robotPowerHeat.ChassisPower;
//		Speed_Limit.Calc(&Speed_Limit);
}

void move_system()
{
			if(C_Flag==1&&SHIFT_Flag!=1)
			{
							if(flag_move.flag== 0)
							{
								flag_move.flag= 1;
								flag_move.time_last = Get_Time_Micros();
								move_speed=1000;
							}
			}
			else if(C_Flag==1&&SHIFT_Flag==1)
			{	
							if(flag_move.flag== 0)
							{
								flag_move.flag= 1;
								flag_move.time_last = Get_Time_Micros();
								move_speed=-1000;
							}
			}
			
							if(flag_move.flag==1)
							{
								flag_move.time_now = Get_Time_Micros();
								flag_move.time_error =flag_move.time_now - flag_move.time_last;
								
							}
							if(flag_move.time_error>500000&&flag_move.flag==1)
							{
								
								flag_move.flag = 0;
								move_speed=0;
							}
}




void rescue_system()
{
	if(obstacleTaskStatus == OBSTACLE_NOTASK_STATE)
	{
		//��Ԯϵͳ����
		if(rescueTaskStatus == RESCUE_STATE)
		{
			gl=6000;
			TIM_SetCompare1(TIM4,9050);                                                                     //����ת��
			//�ж�������Ҽ����ƾ�Ԯϵͳ���ף�������»�е��Ԯ���Ҽ�����ˢ����Ԯ�����߿���ͬʱ
			if(Mouse_l_Flag==1)
			{
//				GPIO_ResetBits(GPIOE,GPIO_Pin_5);//��е��Ԯ�������
				TIM_SetCompare1(TIM8,2000);
				TIM_SetCompare2(TIM8,2000);
			}
			else if(Mouse_l_Flag==0)
			{
//				GPIO_SetBits(GPIOE,GPIO_Pin_5);//��е��Ԯ��������
				TIM_SetCompare1(TIM8,1000);
				TIM_SetCompare2(TIM8,1000);
			}
			if(Mouse_r_Flag==1)
			{
				GPIO_SetBits(GPIOE,GPIO_Pin_6);//ˢ����Ԯ�������
			}
			else if(Mouse_r_Flag==0)
			{
				GPIO_ResetBits(GPIOE,GPIO_Pin_6);//ˢ����Ԯ��������
			}
		}
		else if(rescueTaskStatus == RESCUE_NOTASK_STATE)
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_5);//��е��Ԯ��������
			GPIO_ResetBits(GPIOE,GPIO_Pin_6);//ˢ����Ԯ��������
		}
	}
	else if(obstacleTaskStatus == OBSTACLE_TURN_STATE)
	{
	 //P�����ٿ��ƣ���Ԯϵͳ����ȫ���ջ�
		GPIO_SetBits(GPIOE,GPIO_Pin_5);//��е��Ԯ��������
		GPIO_ResetBits(GPIOE,GPIO_Pin_6);//ˢ����Ԯ��������
	}
}

void camera_system()
{
	//����ͷת��
	if(obstacleTaskStatus == OBSTACLE_NOTASK_STATE)
	{
		if(rescueTaskStatus == RESCUE_STATE)
		{
			if(GPIOH2==0)
			{
				forward_back_speed = LOW_FORWARD_BACK_SPEED ;
				left_right_speed = LOW_LEFT_RIGHT_SPEED;
				rotate_speed = LOW_ROTATE_SPEED;
			}
			else
			{
				forward_back_speed = HIGH_FORWARD_BACK_SPEED ;
				left_right_speed = HIGH_LEFT_RIGHT_SPEED;
				rotate_speed = HIGH_ROTATE_SPEED;
			}
		}
		else
		{
			TIM_SetCompare1(TIM4,jiaodu);//����ת��
			forward_back_speed = HIGH_FORWARD_BACK_SPEED ;
			left_right_speed = HIGH_LEFT_RIGHT_SPEED;
			rotate_speed = HIGH_ROTATE_SPEED;
		}
		//ͼ��Ϊ��ǰ��
		Direction_switching_sign=0;
	}
	else if(obstacleTaskStatus == OBSTACLE_TURN_STATE)
	{
			TIM_SetCompare1(TIM4,8760);//����ת�� 
			//TIM_SetCompare2(TIM4,9260);//����ת��
			forward_back_speed = LOW_FORWARD_BACK_SPEED ;
			left_right_speed = LOW_LEFT_RIGHT_SPEED;
			rotate_speed = LOW_ROTATE_SPEED;
			//צ��Ϊ��ǰ��
			Direction_switching_sign=1;
	}
}

void claw_compensation()
{ 
	//����צ������
	if(TM1Encoder.ecd_angle>=(-10)&&TM1Encoder.ecd_angle<=90)
	{
		flipCompensation=1000.0f*cos(TM1Encoder.ecd_angle*3.1415926f/180);
	}
		else if(TM1Encoder.ecd_angle>90&&TM1Encoder.ecd_angle<=270)
	{
		flipCompensation=1000.0f*cos(TM1Encoder.ecd_angle*3.1415926f/180);
	}
//	if(TM3Encoder.ecd_angle>=(-10)&&TM3Encoder.ecd_angle<=90)//����Ҫ�޸�
//	{
//		flipCompensation=1267.5f*cos(TM3Encoder.ecd_angle*3.1415926f/180);
//	}
//		else if(TM3Encoder.ecd_angle>90&&TM3Encoder.ecd_angle<=190)
//	{
//		flipCompensation=1267.5f*cos(TM3Encoder.ecd_angle*3.1415926f/180);
//	}
}

void uplift_compensation()
{
	//̧���������ֵ
	//upliftCompensation=272.5;
}

