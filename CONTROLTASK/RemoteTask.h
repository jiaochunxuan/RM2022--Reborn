#ifndef __REMOTETASK_H__
#define __REMOTETASK_H__
#include "main.h"
#include <stm32f4xx.h>
/*ң�����ݴ����������*/
#define MOUSE_Ro_RAMP_TICK_COUNT			50
#define MOUSE_LR_RAMP_TICK_COUNT			30
#define MOUSR_FB_RAMP_TICK_COUNT			30
#define MOUSR_MOUSEY_RAMP_TICK_COUNT			20//pitch

//remote data process
//typedef struct
//{
//    float pitch_angle_dynamic_ref;
//    float yaw_angle_dynamic_ref;       //�����ۼ�
//    float yaw_angle_dynamic_ref_add;
//	  float pitch_angle_static_ref;
//    float yaw_angle_static_ref;
//    float pitch_speed_ref;
//    float yaw_speed_ref;
////	  float yaw_angle_dynamic_ref_add;   //���ۼ�
//}Gimbal_Ref_t;
/*�ṹ���������Ͷ���*/
/*�洢�����ж����ݵĽṹ�壬ǰ�����ҡ���ת�ٶ�*/
typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

/*�洢ң�������ݵĽṹ��*/
typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;

/*�洢������ݵĽṹ��*/
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

/*�洢�������ݵĽṹ��*/
typedef	__packed struct
{
	uint16_t v;
}Key;

/*�ýṹ�����ң�����з�ʽ�����ݴ洢�ṹ�壬ң����������*/
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
/*��������*/
void remoteDataProcess(uint8_t *pData);

#endif
