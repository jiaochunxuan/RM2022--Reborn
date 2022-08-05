#ifndef __MOTORTASK_H__
#define __MOTORTASK_H__
#include "stm32f4xx.h"
#include "main.h"

typedef enum
{
	up,//����
	down,//�½�
}PositionMode;//̧������λ��ģʽ

typedef enum
{
	out,//����
	back,//�½�
}ClawPositionMode;//��צ����λ��ģʽ

typedef enum
{
	advance,//��ǰת
	backward,//����ת
}ConveyPositionMode;//���ʹ�����λ��ģʽ

/*��������*/
void motorParameterSettingTask(void);
void MotorExecution(void);
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq);
void Set_XM_Speed(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq);
void Set_XM_SpeedPlus(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq);
void Set_XM_SpeedPlus2(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq);
#endif
