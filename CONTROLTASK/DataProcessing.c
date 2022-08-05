#include "RemoteTask.h"
#include "DataProcessing.h"
#include "Workstate.h"
#include "key.h"
#include "PIDregulation.h"
/*数据处理文件将遥控设备发来的数据进行处理，转化为要执行的动作标志或动作状态，在电机执行函数中将动作标志或状态转化为电机参数输出*/
/*变量定义*/
extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern MiningTaskStatus miningTaskStatus;
extern Workstate workstate;
extern float Eular[2];//记录着陀螺仪YAW轴角度，在这里用于自转90°
extern int Direction_switching_sign;//键盘控制底盘方向切换标志量，为0时图传为前方，为1时爪子为正前方
extern int jiaodu;
/*函数定义*/
/*有符号短整型限位函数实现*/
int16_t VAL_LIMIT(int16_t val,int16_t min,int16_t max)
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
/**
函数：RemoteDataProcessing(Remote *rc)
功能：遥控器数据更新,初步更新在ChassisSpeedRef结构体中，在电机函数部分传递到电机的结构体中，再输出给电机。
**/

void remoteDataProcessing(Remote *rc)
{
	if(getWorkstate()!=PREPARE_STATE )//分别为前后、左右、俯仰、旋转
	{
		   ChassisSpeedRef.forward_back_ref   = -(rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT;
		   ChassisSpeedRef.left_right_ref     = -(rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_LR_REF_FACT; 
		   ChassisSpeedRef.rotate_ref       =  (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT;	
	}
	/*限位*/
//	ChassisSpeedRef.forward_back_ref = VAL_LIMIT(ChassisSpeedRef.forward_back_ref,-300,300);//（可修改）
//	ChassisSpeedRef.left_right_ref = VAL_LIMIT(ChassisSpeedRef.left_right_ref,-170,170);
}


/*键鼠部分*/
/*变量定义*/
uint8_t W_Flag=0;
uint8_t S_Flag=0;
uint8_t A_Flag=0;
uint8_t D_Flag=0;
uint8_t Q_Flag=0;
uint8_t E_Flag=0;
uint8_t SHIFT_Flag=0;
uint8_t CTRL_Flag=0;
uint8_t R_Flag=0;
uint8_t F_Flag=0;
uint8_t G_Flag=0;
uint8_t Z_Flag=0;
uint8_t X_Flag=0;
uint8_t C_Flag=0;
uint8_t V_Flag=0;
uint8_t B_Flag=0;
uint8_t Mouse_l_Flag=0;
uint8_t Mouse_r_Flag=0;

uint16_t forward_back_speed = HIGH_FORWARD_BACK_SPEED ;
uint16_t left_right_speed = HIGH_LEFT_RIGHT_SPEED;
uint16_t rotate_speed = HIGH_ROTATE_SPEED;
float LM_Ref=0;
float TM_Ref_0=0;
float TM_Ref_1=0;
float TM_Ref_2=0;
float TM_Ref_3=0;
float TM_Ref_4=0;
float TM_Ref_5=0;
float TM_Ref_6=0;

/**
函数：MouseKeyDataProcessing(Mouse *mouse, Key *key)
功能：键鼠数据更新
按键配置：WASD控制底盘前后左右（其余待定）
**/
void mouseKeyDataProcessing(Mouse *mouse, Key *key)
{

	if(getWorkstate()!=PREPARE_STATE)
	{
		if(Direction_switching_sign==0)//图传方向为正前方
		{
						/*WASD控制底盘*/
					if(key->v & Key_W)//W:前进
					{
		//			W_Flag=1;
		//			S_Flag=0;
						//forward_back_speed += (forward_back_speed < 700 ? 50 * (700 - forward_back_speed)/700:0);
						ChassisSpeedRef.forward_back_ref = -(forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));

					}
					else if(key->v & Key_S)//S:后退
					{
		//			W_Flag=0;
		//			S_Flag=1;
						//ChassisSpeedRef.left_right_ref = (left_right_speed );//* LRSpeedRamp.Calc(&LRSpeedRamp);
						//forward_back_speed += (forward_back_speed < 700 ? 300 * (700 - forward_back_speed)/700:0);
						ChassisSpeedRef.forward_back_ref = (forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));
					}
					else
					{
						ChassisSpeedRef.forward_back_ref = 0;
					}
					if(key->v & Key_A)//A:左移
					{
		//			A_Flag=1;
		//			D_Flag=0;
						//ChassisSpeedRef.forward_back_ref = -(forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));
						//left_right_speed += (left_right_speed < 700 ? 300 * (700 - left_right_speed)/700:0);
						ChassisSpeedRef.left_right_ref = (left_right_speed );//* LRSpeedRamp.Calc(&LRSpeedRamp);
					}
					else if(key->v & Key_D)//D:右移
					{
						//ChassisSpeedRef.forward_back_ref = (forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));
						//left_right_speed += (left_right_speed < 700 ? 300 * (700 - left_right_speed)/700:0);
						ChassisSpeedRef.left_right_ref = -(left_right_speed );//* LRSpeedRamp.Calc(&LRSpeedRamp);
					}
					else
					{
						ChassisSpeedRef.left_right_ref = 0;
					}
		}
		else//爪子为正前方
		{
								/*WASD控制底盘*/
					if(key->v & Key_W)//W:前进
					{
		//			W_Flag=1;
		//			S_Flag=0;
						//left_right_speed += (left_right_speed < 700 ? 300 * (700 - left_right_speed)/700:0);
						ChassisSpeedRef.left_right_ref = -(left_right_speed );//* LRSpeedRamp.Calc(&LRSpeedRamp);
					}
					else if(key->v & Key_S)//S:后退
					{
		//			W_Flag=0;
		//			S_Flag=1;
						//left_right_speed += (left_right_speed < 700 ? 300 * (700 - left_right_speed)/700:0);
						ChassisSpeedRef.left_right_ref = (left_right_speed );//* LRSpeedRamp.Calc(&LRSpeedRamp);
					}
					else
					{
						ChassisSpeedRef.left_right_ref = 0;
					}
					if(key->v & Key_A)//A:左移
					{
		//			A_Flag=1;
		//			D_Flag=0;
						//forward_back_speed += (forward_back_speed < 700 ? 300 * (700 - forward_back_speed)/700:0);
						ChassisSpeedRef.forward_back_ref = -(forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));
					}
					else if(key->v & Key_D)//D:右移
					{
						//forward_back_speed += (forward_back_speed < 700 ? 300 * (700 - forward_back_speed)/700:0);
						ChassisSpeedRef.forward_back_ref = (forward_back_speed );//* (FBSpeedRamp.Calc(&FBSpeedRamp));
					}
					else
					{
						ChassisSpeedRef.forward_back_ref = 0;
					}
		}
			if(key->v & Key_Q)  // Q:左旋
			{
//			Q_Flag=1;
//			E_Flag=0;
				ChassisSpeedRef.rotate_ref = -rotate_speed;//* RoSpeedRamp.Calc(&RoSpeedRamp);
			}
			else if(key->v & Key_E) //E:右旋
			{
//				Q_Flag=0;
//				E_Flag=1;
				ChassisSpeedRef.rotate_ref = rotate_speed;//* RoSpeedRamp.Calc(&RoSpeedRamp);
			}
			else if(mouse->x != 0) //鼠标控制转向
			{
				ChassisSpeedRef.rotate_ref = 1.8*mouse->x;//需要乘系数（待修改）
//			RoSpeedRamp.ResetCounter(&RoSpeedRamp);
			}
			else
			{
				ChassisSpeedRef.rotate_ref=0;
			}
	}
//G键进入采矿状态
			if(key->v & Key_G)
			{
				G_Flag=1;
			}
//B键将采矿装置初始化
			else if(key->v & Key_B)
			{
				B_Flag=1;
			}
//R键进入兑矿状态
			else if(key->v & Key_R)
			{
				R_Flag=1;
			}
//F键进入抓取车中矿石并放到台上状态
			else if(key->v & Key_F)
			{
				F_Flag=1;
			}
//V键进入无任务状态进行初始化，区别于STOP的点在于柔性恢复，并非急停
			else if(key->v & Key_V)
			{
				V_Flag=1;
			}
//SHIFT键进入推入矿石兑换状态‘
			else if(key->v & Key_SHIFT)
			{
				SHIFT_Flag=1;
				if(key->v & Key_C)
			{
				C_Flag=1;
			}
			}
//底盘
			else if(key->v & Key_C)
			{
				C_Flag=1;
			}
			else
			{
				G_Flag=0;
				B_Flag=0;
				R_Flag=0;
				F_Flag=0;
				V_Flag=0;
				C_Flag=0;
				SHIFT_Flag=0;
			}
			
////c取障碍块切换
//			uint8_t keyvalue_C;//保存键值X
//			keyvalue_C=KEY_Scan_C(0,key);
//			if(keyvalue_C)
//			{
//					C_Flag=!C_Flag;
//			}

//X救援切换
			uint8_t keyvalue_X;//保存键值X
			keyvalue_X=KEY_Scan_X(0,key);
			if(keyvalue_X)
			{
					X_Flag=!X_Flag;
			}
//Z图传切换
			uint8_t keyvalue_Z;//保存键值Z
			keyvalue_Z=KEY_Scan_Z(0,key);
			if(keyvalue_Z)
			{
					Z_Flag=!Z_Flag;
			}
			
//CTRL键控制大小资源岛采矿模式切换 	CTRL_Flag为0时为小资源岛模式，CTRL_Flag为1时为大资源岛模式
			uint8_t keyvalue_ctrl;//保存键值ctrl
			keyvalue_ctrl=KEY_Scan_ctrl(0,key);
			if((keyvalue_ctrl) && (miningTaskStatus==MINING_NOTASK_STATE))
			{
					CTRL_Flag=!CTRL_Flag;
			}
			
////C键自转90°
//			uint8_t keyvalue_C;//保存键值Z
//			keyvalue_C=KEY_Scan_C(0,key);
//			if(keyvalue_C)
//			{
//				C_Flag=1;//用此标志判断进入自转九十度模式，转过90°后此标志清零
//			}
			
//鼠标左键标志
if(mouse->press_l==0)
{
	Mouse_l_Flag=0;
}	
else if(mouse->press_l==1)
{
	Mouse_l_Flag=1;
}
//鼠标右键标志
if(mouse->press_r==0)
{
	Mouse_r_Flag=0;
}	
else if(mouse->press_r==1)
{
	Mouse_r_Flag=1;
}
//鼠标上下控制图传
if(mouse->y < 0)
{
	if(jiaodu<9300)
	{
	jiaodu+=2;
	}	
	else
	{
		jiaodu=9300;
	}
}
else if(mouse->y > 0)
{
	if(jiaodu>9050)
	{
	jiaodu-=2;
	}
	else
	{
		jiaodu=9050;
	}
	}
	
}
