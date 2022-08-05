#include "Workstate.h"
#include "RightSwitch.h"
#include "RemoteTask.h"
#include "MotorTask.h"
/*��������*/
Workstate workstate=STOP_STATE;
Workstate lastWorkstate=STOP_STATE;
MiningTaskStatus miningTaskStatus=MINING_NOTASK_STATE;
MiningTaskStatus lastminingTaskStatus=MINING_NOTASK_STATE;
BarrierTaskStatus barrierTaskStatus=BARRIER_NOTASK_STATE;
RescueTaskStatus rescueTaskStatus=RESCUE_NOTASK_STATE;
ObstacleTaskStatus obstacleTaskStatus=OBSTACLE_NOTASK_STATE;
uint8_t LABELLABEL;//�۲���ܱ���
extern uint32_t time_tick_1ms;
extern uint8_t G_Flag;
extern uint8_t B_Flag;
extern uint8_t C_Flag;
extern uint8_t X_Flag;
extern uint8_t R_Flag;
extern uint8_t F_Flag;
extern uint8_t V_Flag;
extern uint8_t SHIFT_Flag;
extern uint8_t CTRL_Flag;
extern uint8_t Z_Flag;
extern uint8_t GPIOD14;
extern uint8_t GPIOD15;
/*��������*/
extern void ControtLoopTaskInit(void);
#define PREPARE_TIME_TICK_MS 	1000
/*����ʵ��*/
/*������chooseWorkstate(void)���ܣ�ѡ����״̬*/
void chooseWorkstate(void)
{
//	LABELLABEL=GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_0);//�۲���ܷ���ֵ
lastWorkstate = workstate;//�����ϴι���״̬
	switch(getWorkstate())
	{	
		case STOP_STATE:                            
		{
			//��ʼ��ȫ����־
			G_Flag = 0;
			B_Flag = 0;
			C_Flag = 0;
			X_Flag = 0;
			R_Flag = 0;
			F_Flag = 0;
			V_Flag = 0;
			CTRL_Flag = 0;
			SHIFT_Flag = 0;
			Z_Flag = 0;
//			miningTaskStatus = MINING_NOTASK_STATE;
//			lastminingTaskStatus = MINING_NOTASK_STATE;
//			rescueTaskStatus = RESCUE_NOTASK_STATE;
			if((getRightSwitchMode() != STOP_INPUT)&&(getLeftSwitchMode() == GDG_INPUT))
			{
				workstate = PREPARE_STATE;		
				lastWorkstate=STOP_STATE;
			}
			
			else if((getRightSwitchMode() != STOP_INPUT)&&(getLeftSwitchMode() == UP_INPUT))
			{
				workstate = UP_STATE;
				lastWorkstate=STOP_STATE;
			}
			else if((getRightSwitchMode() != STOP_INPUT)&&(getLeftSwitchMode() == DOWN_INPUT))
			{
				workstate = DOWN_STATE;
				lastWorkstate=STOP_STATE;
			}
		}break;
		
		
		
		case UP_STATE:
		{
			if(getRightSwitchMode() == STOP_INPUT)//ң����ǿ��ֹͣ
			{
				workstate = STOP_STATE;
				lastWorkstate=UP_STATE;
			}
			if((getRightSwitchMode() != STOP_INPUT)&&(getLeftSwitchMode() == GDG_INPUT))
			{
				workstate = PREPARE_STATE;
				lastWorkstate=UP_STATE;
			}
		}break;
		
		case DOWN_STATE:
		{
			if(getRightSwitchMode() == STOP_INPUT)//ң����ǿ��ֹͣ
			{
				workstate = STOP_STATE;
				lastWorkstate=DOWN_STATE;
			}
			if((getRightSwitchMode() != STOP_INPUT)&&(getLeftSwitchMode() == GDG_INPUT))
			{
				workstate = PREPARE_STATE;
				lastWorkstate=DOWN_STATE;
			}
		}break;
		
		
		case PREPARE_STATE://׼��״̬
		{
			if(getRightSwitchMode() == STOP_INPUT)//ң����ǿ��ֹͣ
			{
				workstate = STOP_STATE;
			}
			else if(time_tick_1ms > PREPARE_TIME_TICK_MS)//׼��״̬�ﵽָ��ʱ���ת���ٿ�״̬
			{
				workstate = NORMAL_STATE;
			}
		}break;
		
		
		case NORMAL_STATE://�����ֿ���״̬�������룩
		{
			if(getRightSwitchMode() == STOP_INPUT)//ң����ǿ��ֹͣ
			{
				workstate = STOP_STATE;
			}
			
				
			if(X_Flag==1)//�����Ԯ״̬
			{
				if(obstacleTaskStatus == OBSTACLE_NOTASK_STATE)
				{
					rescueTaskStatus = RESCUE_STATE;
				}
				else
				{
					rescueTaskStatus = RESCUE_NOTASK_STATE;
					X_Flag=0;
				}
			}
			else if(X_Flag==0)//�˳���Ԯ״̬
			{
				rescueTaskStatus = RESCUE_NOTASK_STATE;
			}	
			
			
			if(Z_Flag==1)//����ͷת�򵹳��״�
			{
				if(rescueTaskStatus == RESCUE_NOTASK_STATE)
				{
					obstacleTaskStatus = OBSTACLE_TURN_STATE;
				}
				else
				{
					obstacleTaskStatus = OBSTACLE_NOTASK_STATE;
					Z_Flag=0;
				}
			}
			else if(Z_Flag==0)//����ͷ����ǰ
			{
				obstacleTaskStatus = OBSTACLE_NOTASK_STATE;
			}
			
			switch(miningTaskStatus)
			{
				case MINING_NOTASK_STATE:
				{
					if(G_Flag==1&&getMINERAL()==0)
					{
						miningTaskStatus=MINING1_STATE;
						lastminingTaskStatus=MINING_NOTASK_STATE;
					}
					else if(G_Flag==1&&getMINERAL()==1)
					{
						miningTaskStatus=MINING2_STATE;
						lastminingTaskStatus=MINING_NOTASK_STATE;
					}
					else if(G_Flag==1&&getMINERAL()==2)
					{
						miningTaskStatus=MINING2_STATE;
						lastminingTaskStatus=MINING_NOTASK_STATE;
					}
					else if(G_Flag==1&&getMINERAL()==3)
					{
						miningTaskStatus=MINING3_STATE;
						lastminingTaskStatus=MINING_NOTASK_STATE;
					}
//					else if(C_Flag==1)
//					{
//						miningTaskStatus=BARRIER_STATE;
//						lastminingTaskStatus=MINING_NOTASK_STATE;
//					}
					else if(R_Flag==1)
					{
						miningTaskStatus=MININGEXCHANGE_STATE;
						lastminingTaskStatus=MINING_NOTASK_STATE;
					}
					else if(SHIFT_Flag==1&&getMINERAL()==0)
					{
						miningTaskStatus=MININGGROUND_STATE;
						lastminingTaskStatus=MINING_NOTASK_STATE;
					}
					else if(SHIFT_Flag==1&&(getMINERAL()==1||getMINERAL()==2))
					{
						miningTaskStatus=MININGGROUND1_STATE;
						lastminingTaskStatus=MINING_NOTASK_STATE;
					}
					else if(SHIFT_Flag==1&&getMINERAL()==3)
					{
						miningTaskStatus=MININGGROUND2_STATE;
						lastminingTaskStatus=MINING_NOTASK_STATE;
					}
				}break;
				case MINING1_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MINING1_STATE;
					}
					else if(B_Flag==1)
					{
							miningTaskStatus=MININGRETURN1_STATE;
							lastminingTaskStatus=MINING1_STATE;
					}
				}break;
				case MINING2_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MINING2_STATE;
					}
					else if(B_Flag==1&&(getMINERAL()==0||getMINERAL()==2))
					{
							miningTaskStatus=MININGRETURN2_STATE;
							lastminingTaskStatus=MINING2_STATE;
					}
					else if(B_Flag==1&&(getMINERAL()==3||getMINERAL()==1))
					{
							miningTaskStatus=MININGRETURN3_STATE;
							lastminingTaskStatus=MINING2_STATE;
					}
				}break;
				case MINING3_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MINING3_STATE;
					}
					else if(B_Flag==1)
					{
							miningTaskStatus=MININGRETURN3_STATE;
							lastminingTaskStatus=MINING3_STATE;
					}
				}break;
				case MININGRETURN1_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGRETURN1_STATE;
					}
					else if(G_Flag==1&&getMINERAL()==1)
					{
						miningTaskStatus=MINING2_STATE;
						lastminingTaskStatus=MININGRETURN1_STATE;
					}
					else if(G_Flag==1&&getMINERAL()==0)
					{
						miningTaskStatus=MINING1_STATE;
						lastminingTaskStatus=MININGRETURN1_STATE;
					}
				}break;
				case MININGRETURN2_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGRETURN2_STATE;
					}
					else if(G_Flag==1&&getMINERAL()==3)
					{
						miningTaskStatus=MINING3_STATE;
						lastminingTaskStatus=MININGRETURN2_STATE;
					}
					else if(G_Flag==1&&getMINERAL()==2)
					{
						miningTaskStatus=MINING2_STATE;
						lastminingTaskStatus=MININGRETURN2_STATE;
					}
				}break;
				case MININGRETURN3_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGRETURN3_STATE;
					}
					if(G_Flag==1&&getMINERAL()==3)
					{
						miningTaskStatus=MINING3_STATE;
						lastminingTaskStatus=MININGRETURN3_STATE;
					}
					if(R_Flag==1)
					{
						miningTaskStatus=MININGEXCHANGE2_STATE;
						lastminingTaskStatus=MININGRETURN3_STATE;
					}
				}break;
				case MININGEXCHANGE_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGEXCHANGE_STATE;
					}
					if(F_Flag==1)
					{
						miningTaskStatus=MININGOBTAIN_STATE;
						lastminingTaskStatus=MININGEXCHANGE_STATE;
					}
				}break;
				case MININGEXCHANGE2_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGEXCHANGE2_STATE;
					}
					if(SHIFT_Flag==1)
					{
						miningTaskStatus=MININGPUSH_STATE;
						lastminingTaskStatus=MININGEXCHANGE2_STATE;
					}
				}break;
				case MININGOBTAIN_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGOBTAIN_STATE;
					}
					if(SHIFT_Flag==1)
					{
						miningTaskStatus=MININGPUSH_STATE;
						lastminingTaskStatus=MININGOBTAIN_STATE;
					}
					if(R_Flag==1)
					{
						miningTaskStatus=MININGEXCHANGE_STATE;
						lastminingTaskStatus=MININGOBTAIN_STATE;
					}
				}break;
				case MININGPUSH_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGPUSH_STATE;
					}
					if(R_Flag==1)
					{
						miningTaskStatus=MININGEXCHANGE_STATE;
						lastminingTaskStatus=MININGPUSH_STATE;
					}
				}break;
				case MININGRETURN4_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGRETURN4_STATE;
					}
					if(F_Flag==1)
					{
						miningTaskStatus=MININGOBTAIN_STATE;
						lastminingTaskStatus=MININGRETURN4_STATE;
					}
				}break;
				case MININGGROUND_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGGROUND_STATE;
					}
				}break;
				case MININGGROUND1_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGGROUND1_STATE;
					}
				}break;
				case MININGGROUND2_STATE:
				{
					if(V_Flag==1)
					{
						miningTaskStatus=MINING_NOTASK_STATE;
						lastminingTaskStatus=MININGGROUND2_STATE;
					}
					if(R_Flag==1)
					{
						miningTaskStatus=MININGEXCHANGE2_STATE;
						lastminingTaskStatus=MININGRETURN3_STATE;
					}
				}break;
				default:break;
			}
		}break;
		default:
		{
		}
	}
}

void WorkStateSwitchProcess(void)
{
	if((lastWorkstate != workstate) && (workstate == PREPARE_STATE))  
	{
		ControtLoopTaskInit();
	}
}

/*������Workstate GetWorkState(void)*/
Workstate getWorkstate()
{
	return workstate;
}

int getMINERAL(void)
{	
   if(GPIOD14==0&&GPIOD15==0)
	 {return 0;}
	 else if(GPIOD14==1&&GPIOD15==0)
	 {return 1;}
	 else if(GPIOD14==0&&GPIOD15==1)
	 {return 2;}
	 else if(GPIOD14==1&&GPIOD15==1)
	 {return 3;}
}
	
