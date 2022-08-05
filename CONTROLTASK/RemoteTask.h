#ifndef __REMOTETASK_H__
#define __REMOTETASK_H__
#include "main.h"
#include <stm32f4xx.h>
/*遥控数据处理相关声明*/
#define MOUSE_Ro_RAMP_TICK_COUNT			50
#define MOUSE_LR_RAMP_TICK_COUNT			30
#define MOUSR_FB_RAMP_TICK_COUNT			30
#define MOUSR_MOUSEY_RAMP_TICK_COUNT			20//pitch

//remote data process
//typedef struct
//{
//    float pitch_angle_dynamic_ref;
//    float yaw_angle_dynamic_ref;       //不含累加
//    float yaw_angle_dynamic_ref_add;
//	  float pitch_angle_static_ref;
//    float yaw_angle_static_ref;
//    float pitch_speed_ref;
//    float yaw_speed_ref;
////	  float yaw_angle_dynamic_ref_add;   //含累加
//}Gimbal_Ref_t;
/*结构体数据类型定义*/
/*存储底盘行动数据的结构体，前后、左右、旋转速度*/
typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

/*存储遥控器数据的结构体*/
typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;

/*存储鼠标数据的结构体*/
typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;

/*存储键盘数据的结构体*/
typedef	__packed struct
{
	uint16_t v;
}Key;

/*该结构体包含遥控所有方式的数据存储结构体，遥控器、键鼠*/
typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

//typedef enum
//{
//	ZIXUAN_OFF=0,
//	ZIXUAN_ON=1,
//}zixuanState_e;
/*函数声明*/
void remoteDataProcess(uint8_t *pData);

#endif
