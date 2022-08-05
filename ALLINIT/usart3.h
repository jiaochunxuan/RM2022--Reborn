#ifndef __USART3_H__
#define __USART3_H__
#include "main.h"

#define BUFFER_MAX   100
#define REMOTE_DMA1_RX_BUF_LEN    100
#define Robot_State_Rx_Len        17          //机器人状态数据，应接受的数据量
#define Robot_Hurt_Rx_Len         10          //机器人伤害数据，应接受的数据量
#define Robot_PowerHeat_Rx_Len    29          //机器人功率热量数据，应接受的数据量
#define Robot_Shoot_Rx_Len        15          //机器人射击数据，应接受的数据量

typedef __packed struct
{
	int headPosition;
	int tailPosition;
	uint8_t ringBuf[BUFFER_MAX];
}ringBuffer_t;

typedef __packed struct
{
	uint8_t robot_id;
	uint16_t stageRemainTime; //当前阶段剩余时间
	uint8_t gameProgress;     //当前比赛处于哪个阶段
	uint8_t robotLevel;       //机器人当前等级
	uint16_t remainHP;        //机器人当前血量
	uint16_t maxHP;           //机器人满血量
	uint16_t shooter_42_cooling_rate;//42mm枪口每秒冷却值
	uint16_t shooter_42_cooling_limit;//42mm枪口热量上限
	uint16_t shooter_42_speed_limit;//42mm枪口上限速度
	uint16_t chassis_power_limit;//底盘功率上限
}extGameRobotState_t;  //比赛机器人状态

extern ringBuffer_t buffer;
extern extGameRobotState_t robotState;

void USART3_Init(void);
extern uint8_t REMOTE_DMA1_RX_BUF[2][REMOTE_DMA1_RX_BUF_LEN];
void UART3_PrintBlock(uint8_t* pdata, uint8_t len);//数组发送
void UART3_PrintBlock1(void);
void getRobotState(uint8_t *stateData);

void RingBuffer_Write(uint8_t data);
uint8_t   RingBuffer_Read(uint8_t *pdata);
#endif
