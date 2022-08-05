#ifndef __USART3_H__
#define __USART3_H__
#include "main.h"

#define BUFFER_MAX   100
#define REMOTE_DMA1_RX_BUF_LEN    100
#define Robot_State_Rx_Len        17          //������״̬���ݣ�Ӧ���ܵ�������
#define Robot_Hurt_Rx_Len         10          //�������˺����ݣ�Ӧ���ܵ�������
#define Robot_PowerHeat_Rx_Len    29          //�����˹����������ݣ�Ӧ���ܵ�������
#define Robot_Shoot_Rx_Len        15          //������������ݣ�Ӧ���ܵ�������

typedef __packed struct
{
	int headPosition;
	int tailPosition;
	uint8_t ringBuf[BUFFER_MAX];
}ringBuffer_t;

typedef __packed struct
{
	uint8_t robot_id;
	uint16_t stageRemainTime; //��ǰ�׶�ʣ��ʱ��
	uint8_t gameProgress;     //��ǰ���������ĸ��׶�
	uint8_t robotLevel;       //�����˵�ǰ�ȼ�
	uint16_t remainHP;        //�����˵�ǰѪ��
	uint16_t maxHP;           //��������Ѫ��
	uint16_t shooter_42_cooling_rate;//42mmǹ��ÿ����ȴֵ
	uint16_t shooter_42_cooling_limit;//42mmǹ����������
	uint16_t shooter_42_speed_limit;//42mmǹ�������ٶ�
	uint16_t chassis_power_limit;//���̹�������
}extGameRobotState_t;  //����������״̬

extern ringBuffer_t buffer;
extern extGameRobotState_t robotState;

void USART3_Init(void);
extern uint8_t REMOTE_DMA1_RX_BUF[2][REMOTE_DMA1_RX_BUF_LEN];
void UART3_PrintBlock(uint8_t* pdata, uint8_t len);//���鷢��
void UART3_PrintBlock1(void);
void getRobotState(uint8_t *stateData);

void RingBuffer_Write(uint8_t data);
uint8_t   RingBuffer_Read(uint8_t *pdata);
#endif
