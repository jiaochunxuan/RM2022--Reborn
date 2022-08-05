#ifndef __WORKSTATE_H__
#define __WORKSTATE_H__
#include "stm32f4xx.h"
#include "main.h"
/*数据类型定义*/
//区别于遥控器的状态，这里是工程车的整体状态
typedef enum
{
	PREPARE_STATE,
	NORMAL_STATE,					//操作手控制状态
	STOP_STATE,        		//停止状态
	UP_STATE,
	DOWN_STATE,
}Workstate;//整车状态

typedef enum MiningTaskStatus
{
	MINING_NOTASK_STATE,	//采矿系统无任务状态
	MINING1_STATE,         //采矿状态
	MINING2_STATE,         //采矿状态
	MINING3_STATE,         //采矿状态
	MININGRETURN1_STATE,	//采矿系统返回状态1
	MININGRETURN2_STATE,	//采矿系统返回状态2
	MININGRETURN3_STATE,   //采矿系统返回状态3
	MININGRETURN4_STATE,   //采矿系统返回状态4
	MININGPLACE_STATE,		//采矿系统放置矿石状态
	MININGOBTAIN_STATE,		//采矿系统获取车内矿石状态
	MININGPUSH_STATE,     //采矿系统推入矿石进兑换站状态
	MININGGROUND_STATE,   //取地矿石状态
	MININGGROUND1_STATE,   //取地矿石状态
	MININGGROUND2_STATE,   //取地矿石状态
	MININGEXCHANGE_STATE,  //采矿系统兑换矿石状态
	MININGEXCHANGE2_STATE,  //采矿系统兑换矿石状态

}MiningTaskStatus;//操作状态下采矿系统的状态

typedef enum
{
	BARRIER_NOTASK_STATE,  //取障碍块无任务状态
	BARRIER_STATE,					//取障碍块状态
}BarrierTaskStatus;

typedef enum
{
	RESCUE_NOTASK_STATE,  //救援系统无任务状态
	RESCUE_STATE,					//救援状态
}RescueTaskStatus;

typedef enum
{
	OBSTACLE_NOTASK_STATE,
	OBSTACLE_TURN_STATE,
}ObstacleTaskStatus;

/*函数声明*/
Workstate getWorkstate(void);
void WorkStateSwitchProcess(void);
int getMINERAL(void);

#endif
