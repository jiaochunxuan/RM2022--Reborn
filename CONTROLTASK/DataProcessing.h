#ifndef __DATAPROCESSING_H__
#define __DATAPROCESSING_H__
#include <stm32f4xx.h>
#include "main.h"
/*遥控器数据处理*/
/*宏定义(可修改)*/
#define STICK_TO_CHASSIS_SPEED_FB_REF_FACT     2.0f//0.75
#define STICK_TO_CHASSIS_SPEED_LR_REF_FACT     2.0f
#define STICK_TO_ROTATE_ANGLE_INC_FACT         0.96f
#define REMOTE_CONTROLLER_STICK_OFFSET      1024u
/*变量声明*/

/*函数声明*/
void remoteDataProcessing(Remote *rc);

/*键鼠数据处理*/
/*宏定义*/
#define Key_W                  0x0001
#define Key_S                  0x0002
#define Key_A                  0x0004
#define Key_D                  0x0008
#define Key_SHIFT              0x0010
#define Key_CTRL               0x0020
#define Key_Q                  0x0040
#define Key_E                  0x0080
#define Key_R                  0x0100
#define Key_F                  0x0200
#define Key_G                  0x0400
#define Key_Z                  0x0800
#define Key_X                  0x1000
#define Key_C                  0x2000
#define Key_V                  0x4000
#define Key_B                  0x8000
#define HIGH_FORWARD_BACK_SPEED 			  700
#define HIGH_LEFT_RIGHT_SPEED   			  700
#define HIGH_ROTATE_SPEED   			      400

#define LOW_FORWARD_BACK_SPEED 			  500
#define LOW_LEFT_RIGHT_SPEED   			  600
#define LOW_ROTATE_SPEED   			      300
/*变量声明*/
/**
0x0001:w 0x0002:s 0x0004:a 0x0008:d 
0x0010:shift 0x0020:ctrl 0x0040:q 0x0080:e 
0x0100:r 0x0200:f 0x0400:g 0x0800:z 
0x1000:x 0x2000:c 0x4000:v 0x8000:b
**/

/*函数声明*/
void mouseKeyDataProcessing(Mouse *mouse, Key *key);
#endif
