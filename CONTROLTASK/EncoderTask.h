#ifndef __ENCODERTASK_H__
#define __ENCODERTASK_H__
#include <stm32f4xx.h>
#include "main.h"
/*部分宏定义*/
#define RATE_BUF_SIZE 6

/*CAN通信电机电调配置*/
/*CAN1部分*/
/*底盘电机电调配置（4*3508电机）*/
#define CAN_BUS1_CM1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS1_CM2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS1_CM3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS1_CM4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS1_LM1_FEEDBACK_MSG_ID           0x205
#define CAN_BUS1_LM2_FEEDBACK_MSG_ID           0x206

/*CAN2部分*/
#define CAN_BUS2_TM1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS2_TM2_FEEDBACK_MSG_ID           0x202
#define CAN_BUS2_TM3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_TM4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS2_TM5_FEEDBACK_MSG_ID           0x205
#define CAN_BUS2_TM6_FEEDBACK_MSG_ID           0x206
#define CAN_BUS2_TM7_FEEDBACK_MSG_ID           0x207
#define CAN_BUS2_TM8_FEEDBACK_MSG_ID           0x208

/*编码器信息结构体定义*///暂时先不改
/*说明：ecd编码器 raw原始 value数值*/
typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值,可以理解为速度
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	//保存RATE_BUF_SIZE次的diff
	int32_t round_cnt;										//记录圈数
	float filter_rate;											//速度,对diff識ATE_BUF_SIZE次平均
	double ecd_angle;											//角度
	double ecd_xtl_angle;
  float real_torque_current;          //实际转矩电流
}Encoder;


/*函数声明*/
void GetEncoderBias(volatile Encoder *v, CanRxMsg *msg);
void CanReceiveMsgProcess(CanRxMsg * msg);
void Can2_ReceiveMsgProcess(CanRxMsg * msg);

void Motor_3508_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void Motor_3508_EncoderProcess1(volatile Encoder *v, CanRxMsg * msg);
void Motor_2006_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void EncoderReset(volatile Encoder *v);

#endif
