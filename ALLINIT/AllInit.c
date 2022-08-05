#include "AllInit.h"
#include "kalman.h"
#include "timer.h"
#include "PWM.h"
extern void TIM6_Init(void);
extern void TIM6_Start(void);
extern void USART1_Init(void);
extern void CAN1_Configuration(void);
extern void GPIO_PH_Init(void);
extern void GPIO_PE6_Init(void);
extern void CAN2_Configuration(void);
extern void GPIO_PF_Init(void);
extern void GPIO_PE_Init(void);
extern void GPIO_PD_Init(void);
//extern void PWM_PD12_Configuration(void);
//extern void PWM_PD13_Configuration(void);
extern void USART6_Configuration(void);
extern void USART7_Configuration(void);
extern void USART3_Init(void);
extern void UI_init(void);
extern void TIM5_Init(void);
extern void GPIO_PH_Init_1(void);
extern void USART7_Init(void);
kalman p1,p2,p_LM1,p_LM2;
//extern void PWM_PI0_Configuration(void);
//extern void PWM_PI2_Configuration(void);
void kalerman_init(void)
{
//	kalmanCreate(&p1,20,200);
//	kalmanCreate(&p2,0.001,0.001);
	kalmanCreate(&p_LM1,40,200);
	kalmanCreate(&p_LM2,40,200);
}

//void huizheng(void)
//{
//	Claw_Position_set(180,back);
//	
void allInit(void)
{
	//USART7_Configuration();
//	GPIO_PH_Init_1();
//	TIM5_Init();
	
	GPIO_PD_Init();//���ڹ���
	USART3_Init();//����ϵͳͨ��
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	USART7_Init();
	TIM6_Init();
	TIM6_Start();
	TIM2_Configuration();
	USART1_Init();//ң����
	CAN1_Configuration();
	CAN2_Configuration();
	GPIO_PH_Init();//����
	GPIO_PE6_Init();//��е��Ԯ����
	GPIO_PE_Init();
	GPIO_PF_Init();//����
//	GPIO_PE_Init();//ǰ�����
	//��Ԯ���
	PWM_PI5_Configuration();
	PWM_PI6_Configuration();
	//����ͷת��
	PWM_PD12_Configuration();
	USART6_Configuration();//������
	
//	UI_init();
	kalerman_init();
}
