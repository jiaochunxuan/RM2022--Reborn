#include "main.h"
#include "math.h"
#include "arm_math.h"  

/**
����ϵͳ���ݴ���
���ô���3��������
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

float chassisPowerError = 0;  //ǰ�����εĹ��ʲ�
float lastChassisPower = 0;   //��һ�εĹ���
float chassisPowerBuffer = 0;//���ʻ���
uint16_t Heat42_Max = 0;//��֤����ȥ�ӵ���������
uint16_t bulletCounter = 0;
uint16_t bulletNum = 0;
uint8_t GameProgress = 0;
int SpeedAdd = 0;
//uint16_t PowerMax =50;


/***
������void getRobotState(uint8_t *stateData)
���ܣ��Ӳ���ϵͳ��ȡ������״̬
��ע��stateData�Ǳ�0��֡ͷ��ʼ����
	  cmd_ID��0x0201
      ��26�ֽڣ��±�7Ϊ��������ID���±�8Ϊ�����˵ȼ����±�9��10Ϊ������ʣ��Ѫ�����±�31��32Ϊ���̹�����������
***/
void getRobotState(uint8_t *stateData)
{
	robotState.robot_id= stateData[7];//������id
	robotState.robotLevel = stateData[8];//�����˵ȼ�
	robotState.remainHP = stateData[9]|(stateData[10]<<8);//������ʣ��Ѫ��
	robotState.shooter_42_cooling_rate = stateData[25]|(stateData[26]<<8);//42mmǹ��ÿ����ȴֵ
	robotState.shooter_42_cooling_limit = stateData[27]|(stateData[28]<<8);//42mmǹ����������
	robotState.shooter_42_speed_limit = stateData[29]|(stateData[30]<<8);//42mmǹ�������ٶ�
	robotState.chassis_power_limit = stateData[31]|(stateData[32]<<8);//���̹�������
	
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
		Speed42MAX = 170;//10��ÿ��Ħ�����ٶ�
		quickShootFlag = 0;
	}
	else
	{
		Speed42MAX = 280;//265;// + SpeedAdd*0.5;//16��ÿ��Ħ�����ٶ�
		quickShootFlag = 1;
	}
	
	
	if(heat_temp == 1 )//b���������������������Ʒ���
	{
		Heat42_Max = 10000;
	}
	else
	{
		Heat42_Max = robotState.shooter_42_cooling_limit;
	}

	
}
/***
������void getRobotPowerHeat(uint8_t *powerHeartData)
���ܣ��Ӳ���ϵͳ��ȡʵʱ������������
��ע��cmd_ID��0x0202   50HzƵ�����ڷ���
      ��14�ֽ�data���±�11-14Ϊ����������ʣ��±�15��16Ϊ���̹��ʻ��壬�±�21,22Ϊ42mm����ǹ������
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
	robotPowerHeat.ChassisPower = Parameter_Transformation(ChassisPower_temp);//���ֽڶ�������ת��Ϊ������
	
	robotPowerHeat.ChassisPowerBuffer = powerHeartData[15]|(powerHeartData[16]<<8);
	//chassisPowerBuffer = Parameter_Transformation(ChassisPower_buffer);

	robotPowerHeat.shooter_42_Heat= powerHeartData[21]|(powerHeartData[22]<<8);//42mmǹ��ʵʱ����
	
}
/***
������void getRobotShootData(uint8_t *shootData)
���ܣ��Ӳ���ϵͳ��ȡʵʱ�����Ϣ
��ע��cmd_ID��0x0207
      ��6�ֽڣ��±�7Ϊ�ӵ����ͣ��±�8Ϊ����ṹid���±�9Ϊ�ӵ���Ƶ���±�10-13Ϊ�ӵ�����
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
		robotShootData.bulletFreq = shootData[9];//��Ƶ
		Speed42mm_temp = shootData[10]|(shootData[11]<<8)|(shootData[12]<<16)|(shootData[13]<<24);
		robotShootData.bulletSpeed = Parameter_Transformation(Speed42mm_temp);//����
		
		if(robotShootData.bulletSpeed != last_Small_bulletSpeed)
		bulletCounter ++;//���H�l�������Ӌ����
	}
	
	last_Small_bulletSpeed = robotShootData.bulletSpeed;
}

/***
������void getBulletRemain(uint8_t *bulletData)
���ܣ��Ӳ���ϵͳ��ȡʵʱ�ӵ�����
��ע��cmd_ID��0x0208
      ��6�ֽڣ��±�9-10Ϊʣ��ɷ�����
***/
void getBulletRemain(uint8_t *bulletData)
{
	robotBulletData.bullet_42_Num = bulletData[9]|(bulletData[10]<<8);
	bulletNum = robotBulletData.bullet_42_Num;
}

/***
������void getGameStatus(uint8_t *gamestatus)
���ܣ��Ӳ���ϵͳ��ȡ����״̬
��ע��cmd_ID��0x0001
      ��11�ֽڣ��±�7Ϊ�������ͺ͵�ǰ�����׶�
***/
void getGameStatus(uint8_t *gamestatus)
{
	robotGameStatus.game_type = (gamestatus[7]&0x0f);
	robotGameStatus.game_progress = ((gamestatus[7]&0xf0)>>4);
	GameProgress = robotGameStatus.game_progress;
}

/***
������void getRobotHurt(uint8_t *hurtData)
���ܣ��Ӳ���ϵͳ��ȡʵʱ�������˺�
��ע��cmd_ID��0x0206
      ��1�ֽ�
***/
void getRobotHurt(uint8_t *hurtData)
{
	robotHurt.armor_id = (hurtData[7]&0x0f);
	robotHurt.hurt_type =((hurtData[7]&0xf0)>>4);
}

/*********************************************
������Parameter_Transformation
���ܣ����ֽڶ�������ת��Ϊ������
**********************************************/
float Parameter_Transformation(int32_t data)
{
  int temp1,temp4;
	long temp2;
	float temp3;
	//temp1�ǽ���
	//temp2��β��
	//temp3�������õ���
	//temp4�ǵ���β����ÿһλ
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
������void RingBuffer_Write(uint8_t data)
���ܣ�������dataд�뻷�ζ���buffer.ringBuf��
��ע����
***/
void RingBuffer_Write(uint8_t data)
{
	buffer.ringBuf[buffer.tailPosition] = data;     //��β��׷��
	if(++buffer.tailPosition>=BUFFER_MAX)           //β�ڵ�ƫ��
		buffer.tailPosition = 0;                      //����������󳤶ȣ����㣬�γɻ��ζ���
	if(buffer.tailPosition == buffer.headPosition)  //���β���ڵ�׷��ͷ���ڵ㣬���޸�ͷ���ƫ��λ�ö�����������
		if(++buffer.headPosition>=BUFFER_MAX)
		buffer.headPosition = 0;
}

/***
������u8 RingBuffer_Read(uint8_t *pdata)
���ܣ��ӻ��ζ���buffer.ringBuf�ж�ȡ���ݵ���ַpdata��
��ע����
***/
u8 RingBuffer_Read(uint8_t *pdata)
{
	if(buffer.headPosition == buffer.tailPosition)  //���ͷβ�Ӵ���ʾ������Ϊ��
	{
		return 1;  //����1�����λ������ǿյ�
	}
	else
	{
		*pdata = buffer.ringBuf[buffer.headPosition];  //����������ǿ���ȡͷ�ڵ�ֵ��ƫ��ͷ�ڵ�
		if(++buffer.headPosition>=BUFFER_MAX)
			buffer.headPosition = 0;
		return 0;   //����0����ʾ��ȡ���ݳɹ�
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


