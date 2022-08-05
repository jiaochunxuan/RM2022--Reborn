#include "TIM5Init.h"
/*函数声明*/
extern void controlTask(void);
u8 time5_capture_sta=0;
u16 time5_capture_val;

void TIM5_Init(void) //PA0 Echo端
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//使能时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	
	//配置TIM5
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period=5000-1;
    TIM_TimeBaseStructure.TIM_Prescaler=84-1;
    TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);

	//配置TIM5 Channel1输入捕获
    TIM_ICInitStructure.TIM_Channel=TIM_Channel_4;
    TIM_ICInitStructure.TIM_ICFilter=0x00;
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升沿捕获
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM5,&TIM_ICInitStructure);
	
	//配置GPIO口PA0
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;//输入模式
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOI,&GPIO_InitStructure);
    GPIO_ResetBits(GPIOI,GPIO_Pin_0);

	//配置中断分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
    NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM5,ENABLE);//使能TIM5
    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);//允许溢出中断
    TIM_ITConfig(TIM5,TIM_IT_CC1,ENABLE);//允许捕获中断

}
void TIM5_IRQHandler(void)//中断处理函数  （尤其要注意中断服务函数名字的写法）
{
    if((time5_capture_sta&0x80)==0)//标志位最高位为0，说明还没有成功捕获到高电平，此时捕获中断和溢出中断可被响应
    {
        if(TIM_GetITStatus(TIM5,TIM_IT_CC1)!=RESET)//发生捕获中断
        {
            if((time5_capture_sta&0x40)==0)//标志位的次高位为0，说明还没有捕获到一次上升沿，所以这此发生的捕获中断，是捕捉到了一次上升沿
            {
                time5_capture_sta=0;
                time5_capture_val=0;
                TIM_SetCounter(TIM5,0);//设置计数器中的值，在这里将计数器的值设为0
                time5_capture_sta|=0x40;//把标志位的次高位置1，标记为已经捕获到了一次上升沿，这里必须要用或运算，因为time5_capture_sta这个变量的第六位保存着计数器溢出的次数，我们在改变这个变量的最高位和次高位的时候，不能改动后面的第六位数据
                TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
            else if(time5_capture_sta&0x40)//标志位次高位为1，说明上次已经捕获到了一次上升沿，那么这次发生的捕获中断就是捕获到的下降沿
            {
                time5_capture_sta|=0x80;//把标志位的最高位置1，标记已经成功捕获到了一次高电平
                time5_capture_val=TIM_GetCapture1(TIM5);//获取当前计数器中的数值
                TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising);//设置为上升沿捕获，为下一次捕获做准备
            }
        }
        if(TIM_GetITStatus(TIM5,TIM_IT_Update)!=RESET)//发生了溢出中断
        {
            if(time5_capture_sta&0x40)//判断是否在溢出之前已经捕获到了高电平，其实是肯定已经捕捉到了一次高电平
            {
                if((time5_capture_sta&0x3f)==0x3f)//高电平时间太长了，已经溢出了2的6次方-1=63次
                {
                    time5_capture_sta|=0x80;//把标志位的最高位置1，标记已经成功捕获到了一次高电平，此时为强制结束高电平检测
                    time5_capture_val=0xffff;//强制结束后，将计数器的值设为最大，其实程序运行到这里，它已经是最大了
                }
                else
                {
                    time5_capture_sta++;//如果溢出次数没有超出标志位的低六位所能表示的范围，那么就让标志位直接加1，以此来记录溢出的次数，溢出次数一直加下去，啥时候超出了标志位的低六位所能表示的范围，那么就强制退出检测，直接认为此次检测高电平已经结束，然后直接开启下一次检测
                }
            }
        }
    }
    //清除中断标志位
    TIM_ClearITPendingBit(TIM5,TIM_IT_CC1);
    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
}
