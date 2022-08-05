#ifndef __ENCODERTASK_H__
#define __ENCODERTASK_H__
#include <stm32f4xx.h>
#include "main.h"
/*���ֺ궨��*/
#define RATE_BUF_SIZE 6

/*CANͨ�ŵ���������*/
/*CAN1����*/
/*���̵��������ã�4*3508�����*/
#define CAN_BUS1_CM1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS1_CM2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS1_CM3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS1_CM4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS1_LM1_FEEDBACK_MSG_ID           0x205
#define CAN_BUS1_LM2_FEEDBACK_MSG_ID           0x206

/*CAN2����*/
#define CAN_BUS2_TM1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS2_TM2_FEEDBACK_MSG_ID           0x202
#define CAN_BUS2_TM3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_TM4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS2_TM5_FEEDBACK_MSG_ID           0x205
#define CAN_BUS2_TM6_FEEDBACK_MSG_ID           0x206
#define CAN_BUS2_TM7_FEEDBACK_MSG_ID           0x207
#define CAN_BUS2_TM8_FEEDBACK_MSG_ID           0x208

/*��������Ϣ�ṹ�嶨��*///��ʱ�Ȳ���
/*˵����ecd������ rawԭʼ value��ֵ*/
typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ,�������Ϊ�ٶ�
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	//����RATE_BUF_SIZE�ε�diff
	int32_t round_cnt;										//��¼Ȧ��
	float filter_rate;											//�ٶ�,��diff�RATE_BUF_SIZE��ƽ��
	double ecd_angle;											//�Ƕ�
	double ecd_xtl_angle;
  float real_torque_current;          //ʵ��ת�ص���
}Encoder;


/*��������*/
void GetEncoderBias(volatile Encoder *v, CanRxMsg *msg);
void CanReceiveMsgProcess(CanRxMsg * msg);
void Can2_ReceiveMsgProcess(CanRxMsg * msg);

void Motor_3508_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void Motor_3508_EncoderProcess1(volatile Encoder *v, CanRxMsg * msg);
void Motor_2006_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void EncoderReset(volatile Encoder *v);

#endif
