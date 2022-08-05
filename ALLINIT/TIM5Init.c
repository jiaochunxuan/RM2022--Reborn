#include "TIM5Init.h"
/*��������*/
extern void controlTask(void);
u8 time5_capture_sta=0;
u16 time5_capture_val;

void TIM5_Init(void) //PA0 Echo��
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//ʹ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	
	//����TIM5
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period=5000-1;
    TIM_TimeBaseStructure.TIM_Prescaler=84-1;
    TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);

	//����TIM5 Channel1���벶��
    TIM_ICInitStructure.TIM_Channel=TIM_Channel_4;
    TIM_ICInitStructure.TIM_ICFilter=0x00;
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//�����ز���
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM5,&TIM_ICInitStructure);
	
	//����GPIO��PA0
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;//����ģʽ
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOI,&GPIO_InitStructure);
    GPIO_ResetBits(GPIOI,GPIO_Pin_0);

	//�����жϷ���
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
    NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM5,ENABLE);//ʹ��TIM5
    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);//��������ж�
    TIM_ITConfig(TIM5,TIM_IT_CC1,ENABLE);//�������ж�

}
void TIM5_IRQHandler(void)//�жϴ�����  ������Ҫע���жϷ��������ֵ�д����
{
    if((time5_capture_sta&0x80)==0)//��־λ���λΪ0��˵����û�гɹ����񵽸ߵ�ƽ����ʱ�����жϺ�����жϿɱ���Ӧ
    {
        if(TIM_GetITStatus(TIM5,TIM_IT_CC1)!=RESET)//���������ж�
        {
            if((time5_capture_sta&0x40)==0)//��־λ�Ĵθ�λΪ0��˵����û�в���һ�������أ�������˷����Ĳ����жϣ��ǲ�׽����һ��������
            {
                time5_capture_sta=0;
                time5_capture_val=0;
                TIM_SetCounter(TIM5,0);//���ü������е�ֵ�������ｫ��������ֵ��Ϊ0
                time5_capture_sta|=0x40;//�ѱ�־λ�Ĵθ�λ��1�����Ϊ�Ѿ�������һ�������أ��������Ҫ�û����㣬��Ϊtime5_capture_sta��������ĵ���λ�����ż���������Ĵ����������ڸı�������������λ�ʹθ�λ��ʱ�򣬲��ܸĶ�����ĵ���λ����
                TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
            else if(time5_capture_sta&0x40)//��־λ�θ�λΪ1��˵���ϴ��Ѿ�������һ�������أ���ô��η����Ĳ����жϾ��ǲ��񵽵��½���
            {
                time5_capture_sta|=0x80;//�ѱ�־λ�����λ��1������Ѿ��ɹ�������һ�θߵ�ƽ
                time5_capture_val=TIM_GetCapture1(TIM5);//��ȡ��ǰ�������е���ֵ
                TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising);//����Ϊ�����ز���Ϊ��һ�β�����׼��
            }
        }
        if(TIM_GetITStatus(TIM5,TIM_IT_Update)!=RESET)//����������ж�
        {
            if(time5_capture_sta&0x40)//�ж��Ƿ������֮ǰ�Ѿ������˸ߵ�ƽ����ʵ�ǿ϶��Ѿ���׽����һ�θߵ�ƽ
            {
                if((time5_capture_sta&0x3f)==0x3f)//�ߵ�ƽʱ��̫���ˣ��Ѿ������2��6�η�-1=63��
                {
                    time5_capture_sta|=0x80;//�ѱ�־λ�����λ��1������Ѿ��ɹ�������һ�θߵ�ƽ����ʱΪǿ�ƽ����ߵ�ƽ���
                    time5_capture_val=0xffff;//ǿ�ƽ����󣬽���������ֵ��Ϊ�����ʵ�������е�������Ѿ��������
                }
                else
                {
                    time5_capture_sta++;//����������û�г�����־λ�ĵ���λ���ܱ�ʾ�ķ�Χ����ô���ñ�־λֱ�Ӽ�1���Դ�����¼����Ĵ������������һֱ����ȥ��ɶʱ�򳬳��˱�־λ�ĵ���λ���ܱ�ʾ�ķ�Χ����ô��ǿ���˳���⣬ֱ����Ϊ�˴μ��ߵ�ƽ�Ѿ�������Ȼ��ֱ�ӿ�����һ�μ��
                }
            }
        }
    }
    //����жϱ�־λ
    TIM_ClearITPendingBit(TIM5,TIM_IT_CC1);
    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
}
