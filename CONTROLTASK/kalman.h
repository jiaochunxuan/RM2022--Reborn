//对号

/** ******************************************************************************
  * @file    kalman.h                                                            *
  * @author  Liu heng                                                            *
  * @version V1.0.0                                                              *
  * @date    27-August-2013                                                      *
  * @brief   Hearder file for kalman filter                                      *
  *                                                                              *
  ********************************************************************************
  *          此代码可任意传播与使用，但请注明出处。                              *
  ********************************************************************************/
#ifndef _KALMAN_H
#define _KALMAN_H

#include "main.h"

typedef struct 
{
    float X_last; //上一时刻的最优结果
    float X_mid;  //当前时刻的预测结果
    float X_now;  //当前时刻的最优结果
    float P_mid;  //当前时刻预测结果的协方差
    float P_now;  //当前时刻最优结果的协方差
    float P_last; //上一时刻最优结果的协方差
    float kg;     //kalman增益
    float A;      //系统参数
    float Q;
    float R;
    float H;
}kalman;

extern kalman CR_yaw;
extern kalman CR_yaw_Feedforward;
extern kalman CR_pitch;


void kalmanCreate(kalman *p,float T_Q,float T_R);
float KalmanFilter(kalman* p,float dat);

#endif
