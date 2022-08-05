#ifndef __WORKSTATE_H__
#define __WORKSTATE_H__
#include "stm32f4xx.h"
#include "main.h"
/*�������Ͷ���*/
//������ң������״̬�������ǹ��̳�������״̬
typedef enum
{
	PREPARE_STATE,
	NORMAL_STATE,					//�����ֿ���״̬
	STOP_STATE,        		//ֹͣ״̬
	UP_STATE,
	DOWN_STATE,
}Workstate;//����״̬

typedef enum MiningTaskStatus
{
	MINING_NOTASK_STATE,	//�ɿ�ϵͳ������״̬
	MINING1_STATE,         //�ɿ�״̬
	MINING2_STATE,         //�ɿ�״̬
	MINING3_STATE,         //�ɿ�״̬
	MININGRETURN1_STATE,	//�ɿ�ϵͳ����״̬1
	MININGRETURN2_STATE,	//�ɿ�ϵͳ����״̬2
	MININGRETURN3_STATE,   //�ɿ�ϵͳ����״̬3
	MININGRETURN4_STATE,   //�ɿ�ϵͳ����״̬4
	MININGPLACE_STATE,		//�ɿ�ϵͳ���ÿ�ʯ״̬
	MININGOBTAIN_STATE,		//�ɿ�ϵͳ��ȡ���ڿ�ʯ״̬
	MININGPUSH_STATE,     //�ɿ�ϵͳ�����ʯ���һ�վ״̬
	MININGGROUND_STATE,   //ȡ�ؿ�ʯ״̬
	MININGGROUND1_STATE,   //ȡ�ؿ�ʯ״̬
	MININGGROUND2_STATE,   //ȡ�ؿ�ʯ״̬
	MININGEXCHANGE_STATE,  //�ɿ�ϵͳ�һ���ʯ״̬
	MININGEXCHANGE2_STATE,  //�ɿ�ϵͳ�һ���ʯ״̬

}MiningTaskStatus;//����״̬�²ɿ�ϵͳ��״̬

typedef enum
{
	BARRIER_NOTASK_STATE,  //ȡ�ϰ���������״̬
	BARRIER_STATE,					//ȡ�ϰ���״̬
}BarrierTaskStatus;

typedef enum
{
	RESCUE_NOTASK_STATE,  //��Ԯϵͳ������״̬
	RESCUE_STATE,					//��Ԯ״̬
}RescueTaskStatus;

typedef enum
{
	OBSTACLE_NOTASK_STATE,
	OBSTACLE_TURN_STATE,
}ObstacleTaskStatus;

/*��������*/
Workstate getWorkstate(void);
void WorkStateSwitchProcess(void);
int getMINERAL(void);

#endif
