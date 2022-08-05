#ifndef __MOTORTASK_H__
#define __MOTORTASK_H__
#include "stm32f4xx.h"
#include "main.h"

typedef enum
{
	up,//上升
	down,//下降
}PositionMode;//抬升机构位置模式

typedef enum
{
	out,//上升
	back,//下降
}ClawPositionMode;//手爪机构位置模式

typedef enum
{
	advance,//往前转
	backward,//往后转
}ConveyPositionMode;//传送带机构位置模式

/*函数声明*/
void motorParameterSettingTask(void);
void MotorExecution(void);
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq);
void Set_XM_Speed(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq);
void Set_XM_SpeedPlus(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq);
void Set_XM_SpeedPlus2(CAN_TypeDef *CANx, int16_t m1_iq, int16_t m2_iq, int16_t m3_iq, int16_t m4_iq);
#endif
