#include "main.h"
#include "PIDregulation.h"
/*���PID����*/
/*���̵��Chassis Motor,�ٶ�PID*/
PID_Regulation_t CM1SpeedPID = CHASSIS_MOTOR_CM_1_SPEED_PID_DEFAULT;
PID_Regulation_t CM2SpeedPID = CHASSIS_MOTOR_CM_2_SPEED_PID_DEFAULT;
PID_Regulation_t CM3SpeedPID = CHASSIS_MOTOR_CM_3_SPEED_PID_DEFAULT;
PID_Regulation_t CM4SpeedPID = CHASSIS_MOTOR_CM_4_SPEED_PID_DEFAULT;
//PID_Regulation_t Speed_Limit= CHASSIS_Speed_Limit_PID_DEFAULT;

/*̧�����Lifting Motor,λ��PID*/
PID_Regulation_t LM1PositionPID = CHASSIS_MOTOR_LM_1_POSITION_PID_DEFAULT;
PID_Regulation_t LM2PositionPID = CHASSIS_MOTOR_LM_2_POSITION_PID_DEFAULT;
PID_Regulation_t LM1SpeedPID = CHASSIS_MOTOR_LM_1_SPEED_PID_DEFAULT;
PID_Regulation_t LM2SpeedPID = CHASSIS_MOTOR_LM_2_SPEED_PID_DEFAULT;

/*��ת���TurnoverMotor,λ��PID*/
PID_Regulation_t TM1PositionPID = CHASSIS_MOTOR_TM_1_POSITION_PID_DEFAULT;
PID_Regulation_t TM2PositionPID = CHASSIS_MOTOR_TM_2_POSITION_PID_DEFAULT;
PID_Regulation_t TM1SpeedPID = CHASSIS_MOTOR_TM_1_SPEED_PID_DEFAULT;
PID_Regulation_t TM2SpeedPID = CHASSIS_MOTOR_TM_2_SPEED_PID_DEFAULT;
PID_Regulation_t TM5PositionPID = CHASSIS_MOTOR_TM_5_POSITION_PID_DEFAULT;
PID_Regulation_t TM6PositionPID = CHASSIS_MOTOR_TM_6_POSITION_PID_DEFAULT;
PID_Regulation_t TM5SpeedPID = CHASSIS_MOTOR_TM_5_SPEED_PID_DEFAULT;
PID_Regulation_t TM6SpeedPID = CHASSIS_MOTOR_TM_6_SPEED_PID_DEFAULT;
PID_Regulation_t TM7PositionPID = CHASSIS_MOTOR_TM_7_POSITION_PID_DEFAULT;
PID_Regulation_t TM8PositionPID = CHASSIS_MOTOR_TM_8_POSITION_PID_DEFAULT;
PID_Regulation_t TM7SpeedPID = CHASSIS_MOTOR_TM_7_SPEED_PID_DEFAULT;
PID_Regulation_t TM8SpeedPID = CHASSIS_MOTOR_TM_8_SPEED_PID_DEFAULT;

/*���ʹ����Convey Motor,�ٶ�PID*/
PID_Regulation_t TM3SpeedPID = CHASSIS_MOTOR_TM_3_SPEED_PID_DEFAULT;
PID_Regulation_t TM4SpeedPID = CHASSIS_MOTOR_TM_4_SPEED_PID_DEFAULT;
PID_Regulation_t TM3PositionPID = CHASSIS_MOTOR_TM_3_POSITION_PID_DEFAULT;
PID_Regulation_t TM4PositionPID = CHASSIS_MOTOR_TM_4_POSITION_PID_DEFAULT;

/*��������λ����ʵ��*/
float VAL_LIMITF(float val,float min,float max)
{
	if(val<=min)
	{
		val = min;
	}
	else if(val>=max)
	{
		val = max;
	}
	return val;
}

/*����ʵ��*/
void PID_Calc(PID_Regulation_t *pid)
{
	pid->err[0] = pid->ref - pid->fdb;
	
	if((pid->err[0] >= pid->diedzone) || (pid->err[0] < -pid->diedzone))
	{
		pid->componentKp = pid->kp * pid->err[0];
		pid->componentKp = VAL_LIMITF(pid->componentKp,-pid->componentKpMax,pid->componentKpMax);
//		if((pid->err[0] * pid->err[1] < 0))
//		{
//			pid->ki = 0;
//		}
		pid->componentKi+= pid->ki * pid->err[0];
		pid->componentKi = VAL_LIMITF(pid->componentKi,-pid->componentKiMax,pid->componentKiMax);
		
		
	//	pid->componentKd = pid->kd * ( pid->err[0] - pid->err[1] );
		//����Ϊ�Ľ��桪������ͨ�˲��Ĳ���ȫ΢��
		pid->componentKd = pid->kd * ( pid->err[0] - pid->err[1] )*0.7f+pid->componentKd*0.3f;
		pid->componentKd = VAL_LIMITF(pid->componentKd,-pid->componentKdMax,pid->componentKdMax);
		pid->output=pid->componentKp + pid->componentKi + pid->componentKd;
	}
	else
	{
	}
	
	pid->output = VAL_LIMITF(pid->output,-pid->outputMax,pid->outputMax);
	
	pid->err[1] = pid->err[0];
}
void PID_Calc_1(PID_Regulation_t *pid)
{
	int index=1;
	pid->err[0] = pid->ref - pid->fdb;
	
	if((pid->err[0] >= pid->diedzone) || (pid->err[0] < -pid->diedzone))
	{
		pid->componentKp = pid->kp * pid->err[0];
		pid->componentKp = VAL_LIMITF(pid->componentKp,-pid->componentKpMax,pid->componentKpMax);
//		if((pid->err[0] * pid->err[1] < 0))
//		{
//			pid->ki = 0;
//		}
//		if(abs(pid->err[0])>20) //���ַ���
//		{
//			index=0;
//		}
//		else
//		{
//			index=1;
//		}
		
		if(abs(pid->err[0])>200) //�����
		{
			index=0;
		}
		else if(pid->err[0]<180)
		{
			index=1;
		}
		else
		{
			index=(200-abs(pid->err[0]))/20;
		}
		
//		pid->componentKi+= index*pid->ki * pid->err[0];
		pid->componentKi+= index*(pid->ki * pid->err[0]+pid->ki * pid->err[1])/2; //���λ���
		pid->componentKi = VAL_LIMITF(pid->componentKi,-pid->componentKiMax,pid->componentKiMax);
		
		
	//	pid->componentKd = pid->kd * ( pid->err[0] - pid->err[1] );
		//����Ϊ�Ľ��桪������ͨ�˲��Ĳ���ȫ΢��
		pid->componentKd = pid->kd * ( pid->err[0] - pid->err[1] )*0.7f+pid->componentKd*0.3f;
		pid->componentKd = VAL_LIMITF(pid->componentKd,-pid->componentKdMax,pid->componentKdMax);
		pid->output=pid->componentKp + pid->componentKi + pid->componentKd;
	}
	else
	{
	}
	
	pid->output = VAL_LIMITF(pid->output,-pid->outputMax,pid->outputMax);
	
	pid->err[1] = pid->err[0];
}
//PID��λ
void PID_Reset(PID_Regulation_t *pid)
{
	pid->ref 		= 0;
	pid->fdb		= 0;
	
	pid->err[0] = 0;
	pid->err[1]	= 0;
	pid->componentKp = 0;
	pid->componentKi = 0;
	pid->componentKd = 0;
	pid->output			 = 0;
}
