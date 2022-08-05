#include "main.h"
#include "math.h"
#include "arm_math.h"  

/**
裁判系统数据处理
利用串口3接收数据
**/

//kalman_filter_t KALMAN1 = KALMAN1INIT;
//kalman_filter_init_t KALMAN2 = KALMAN2INIT;

extGameRobotState_t robotState;
extPowerHeatData_t  robotPowerHeat;
extShootData_t      robotShootData;
extBulletData_t     robotBulletData;
ext_game_status_t   robotGameStatus;
ext_robot_hurt_t    robotHurt;
ringBuffer_t buffer;

float chassisPowerError = 0;  //前后两次的功率差
float lastChassisPower = 0;   //上一次的功率
float chassisPowerBuffer = 0;//功率缓冲
uint16_t Heat42_Max = 0;//保证发出去子弹不超热量
uint16_t bulletCounter = 0;
uint16_t bulletNum = 0;
uint8_t GameProgress = 0;
int SpeedAdd = 0;
//uint16_t PowerMax =50;


/***
函数：void getRobotState(uint8_t *stateData)
功能：从裁判系统读取机器人状态
备注：stateData角标0从帧头开始计算
	  cmd_ID：0x0201
      共26字节，下标7为本机器人ID，下标8为机器人等级，下标9、10为机器人剩余血量，下标31、32为底盘功率限制上限
***/
void getRobotState(uint8_t *stateData)
{
	robotState.robot_id= stateData[7];//机器人id
	robotState.robotLevel = stateData[8];//机器人等级
	robotState.remainHP = stateData[9]|(stateData[10]<<8);//机器人剩余血量
	robotState.shooter_42_cooling_rate = stateData[25]|(stateData[26]<<8);//42mm枪口每秒冷却值
	robotState.shooter_42_cooling_limit = stateData[27]|(stateData[28]<<8);//42mm枪口热量上限
	robotState.shooter_42_speed_limit = stateData[29]|(stateData[30]<<8);//42mm枪口上限速度
	robotState.chassis_power_limit = stateData[31]|(stateData[32]<<8);//底盘功率上限
	
	if(robotState.remainHP == 0)
	{
		workState = NORMAL_STATE;
		
		autoaim_state = AUTOAIM_OFF;
		autoaim.keyCnt = 0;
		
		xtl_state = XTL_OFF;
		xtl.keyCnt = 0;
		
		flag_far_shoot = 0;
		Far_Shoot.keyCnt = 0;
		
		temp_CM7 = -CM7Encoder.ecd_angle;
		count_temp = 0;
		count_temp1 = 0;
	}
	
	if(robotState.shooter_42_speed_limit == 10)
	{
		Speed42MAX = 170;//10米每秒摩擦轮速度
		quickShootFlag = 0;
	}
	else
	{
		Speed42MAX = 280;//265;// + SpeedAdd*0.5;//16米每秒摩擦轮速度
		quickShootFlag = 1;
	}
	
	
	if(heat_temp == 1 )//b键爆发，不考虑热量限制发射
	{
		Heat42_Max = 10000;
	}
	else
	{
		Heat42_Max = robotState.shooter_42_cooling_limit;
	}

	
}
/***
函数：void getRobotPowerHeat(uint8_t *powerHeartData)
功能：从裁判系统读取实时功率热量数据
备注：cmd_ID：0x0202   50Hz频率周期发送
      共14字节data，下标11-14为底盘输出功率，下标15、16为底盘功率缓冲，下标21,22为42mm弹丸枪口热量
***/
char test[14]={0};
int icnt=0;
void getRobotPowerHeat(uint8_t *powerHeartData)
{
	uint32_t ChassisPower_temp;
	uint16_t ChassisPower_buffer;
	for(icnt=0;icnt<14;icnt++)
	{
		test[icnt]=powerHeartData[5+icnt];
	}
	ChassisPower_temp = powerHeartData[11]|(powerHeartData[12]<<8)|(powerHeartData[13]<<16)|(powerHeartData[14]<<24);
	robotPowerHeat.ChassisPower = Parameter_Transformation(ChassisPower_temp);//四字节二进制数转换为浮点数
	
	robotPowerHeat.ChassisPowerBuffer = powerHeartData[15]|(powerHeartData[16]<<8);
	//chassisPowerBuffer = Parameter_Transformation(ChassisPower_buffer);

	robotPowerHeat.shooter_42_Heat= powerHeartData[21]|(powerHeartData[22]<<8);//42mm枪口实时热量
	
}
/***
函数：void getRobotShootData(uint8_t *shootData)
功能：从裁判系统读取实时射击信息
备注：cmd_ID：0x0207
      共6字节，下标7为子弹类型，下标8为发射结构id，下标9为子弹射频，下标10-13为子弹射速
***/
float bullet_speed = 0.0;
void getRobotShootData(uint8_t *shootData)
{
	uint32_t Speed42mm_temp;
	static float last_Small_bulletSpeed;
	robotShootData.bulletType=shootData[7];
	robotShootData.ShootIDType=shootData[8];
	if((shootData[7]==2)&&(shootData[8]==3))
	{
		robotShootData.bulletFreq = shootData[9];//射频
		Speed42mm_temp = shootData[10]|(shootData[11]<<8)|(shootData[12]<<16)|(shootData[13]<<24);
		robotShootData.bulletSpeed = Parameter_Transformation(Speed42mm_temp);//彈速
		
		if(robotShootData.bulletSpeed != last_Small_bulletSpeed)
		bulletCounter ++;//實際發出彈丸的計數器
	}
	
	last_Small_bulletSpeed = robotShootData.bulletSpeed;
}

/***
函数：void getBulletRemain(uint8_t *bulletData)
功能：从裁判系统读取实时子弹数量
备注：cmd_ID：0x0208
      共6字节，下标9-10为剩余可发弹数
***/
void getBulletRemain(uint8_t *bulletData)
{
	robotBulletData.bullet_42_Num = bulletData[9]|(bulletData[10]<<8);
	bulletNum = robotBulletData.bullet_42_Num;
}

/***
函数：void getGameStatus(uint8_t *gamestatus)
功能：从裁判系统读取比赛状态
备注：cmd_ID：0x0001
      共11字节，下标7为比赛类型和当前比赛阶段
***/
void getGameStatus(uint8_t *gamestatus)
{
	robotGameStatus.game_type = (gamestatus[7]&0x0f);
	robotGameStatus.game_progress = ((gamestatus[7]&0xf0)>>4);
	GameProgress = robotGameStatus.game_progress;
}

/***
函数：void getRobotHurt(uint8_t *hurtData)
功能：从裁判系统读取实时机器人伤害
备注：cmd_ID：0x0206
      共1字节
***/
void getRobotHurt(uint8_t *hurtData)
{
	robotHurt.armor_id = (hurtData[7]&0x0f);
	robotHurt.hurt_type =((hurtData[7]&0xf0)>>4);
}

/*********************************************
函数：Parameter_Transformation
功能：四字节二进制数转换为浮点数
**********************************************/
float Parameter_Transformation(int32_t data)
{
  int temp1,temp4;
	long temp2;
	float temp3;
	//temp1是阶码
	//temp2是尾数
	//temp3是最后算好的数
	//temp4是电流尾数的每一位
	temp1=((data&0X7F800000)>>23)-127; 
	temp2= data&0X007FFFFF;
	for(int j=0;j<24;j++)
	{
		if(j==0)
		{ 
			temp3=(float)ldexp(1.0,temp1);
		}
		else
		{
		temp4=(temp2&(0x00400000>>(j-1)))>>(23-j);
		temp3=temp3+temp4*(float)ldexp(1.0,temp1-j);
		}
	}
	return temp3;
}
/***
函数：void RingBuffer_Write(uint8_t data)
功能：将数据data写入环形队列buffer.ringBuf中
备注：无
***/
void RingBuffer_Write(uint8_t data)
{
	buffer.ringBuf[buffer.tailPosition] = data;     //从尾部追加
	if(++buffer.tailPosition>=BUFFER_MAX)           //尾节点偏移
		buffer.tailPosition = 0;                      //大于数组最大长度，归零，形成环形队列
	if(buffer.tailPosition == buffer.headPosition)  //如果尾部节点追到头部节点，则修改头结点偏移位置丢弃早期数据
		if(++buffer.headPosition>=BUFFER_MAX)
		buffer.headPosition = 0;
}

/***
函数：u8 RingBuffer_Read(uint8_t *pdata)
功能：从环形队列buffer.ringBuf中读取数据到地址pdata中
备注：无
***/
u8 RingBuffer_Read(uint8_t *pdata)
{
	if(buffer.headPosition == buffer.tailPosition)  //如果头尾接触表示缓冲区为空
	{
		return 1;  //返回1，环形缓冲区是空的
	}
	else
	{
		*pdata = buffer.ringBuf[buffer.headPosition];  //如果缓冲区非空则取头节点值并偏移头节点
		if(++buffer.headPosition>=BUFFER_MAX)
			buffer.headPosition = 0;
		return 0;   //返回0，表示读取数据成功
	}
}


void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
	float Adata[4]={1,4,0,1};
	float Hdata[2]={1,0};
	float Qdata[4]={0.001,0,0,0.001};
	float Rdata[1]={2};
	float Pdata[4]={1,0,0,1};
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
	mat_init(&F->xhatminus,2,2,(float *)I->xhatminus_data);
	mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
	mat_init(&F->A,2,2,(float *)Adata);
	mat_init(&F->AT,2,2,(float *)I->AT_data);
	mat_init(&F->H,1,2,(float *)Hdata);
	mat_init(&F->Q,2,2,(float *)Qdata);
	mat_init(&F->K,2,1,(float *)I->K_data);
	mat_init(&F->z,1,1,(float *)I->z_data);
	mat_init(&F->R,1,1,(float *)Rdata);
	mat_init(&F->P,2,2,(float *)Pdata);
  mat_init(&F->HT,2,1,(float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
	mat_trans(&F->A, &F->AT);
}

float kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
	float idata[4]={1,0,0,1};
  mat TEMP,TEMP21,i;

  mat_init(&TEMP,2,2,(float *)TEMP_data);
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);
  mat_init(&i,2,2,(float *)idata);
  F->z.pData[0] = signal1;

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);
	

	
  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);
  mat_inv(&F->K, &F->P);
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);
  mat_sub(&F->z, &TEMP21, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP21);
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);
  mat_sub(&i, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];

  return F->filtered_value[0];
}


