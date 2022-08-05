#include "CAN2.h"
/*函数声明*/
extern void Can2_ReceiveMsgProcess(CanRxMsg * msg);
/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/

void CAN2_Configuration(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	/* -------------- Configure GPIO & CAN1 ---------------------------------*/
	{
		GPIO_InitTypeDef       gpio;
		CAN_InitTypeDef        can2;
		CAN_FilterInitTypeDef  can2_filter;
		
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

		gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio);
		
		CAN_DeInit(CAN2);
		CAN_StructInit(&can2);

		can2.CAN_TTCM = DISABLE;
		can2.CAN_ABOM = DISABLE;    
		can2.CAN_AWUM = DISABLE;    
		can2.CAN_NART = DISABLE;    
		can2.CAN_RFLM = DISABLE;    
		can2.CAN_TXFP = ENABLE;     
		can2.CAN_Mode = CAN_Mode_Normal; 
		can2.CAN_SJW  = CAN_SJW_1tq;
		can2.CAN_BS1 = CAN_BS1_9tq;
		can2.CAN_BS2 = CAN_BS2_4tq;
		can2.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
		CAN_Init(CAN2, &can2);

		can2_filter.CAN_FilterNumber=14;
		can2_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
		can2_filter.CAN_FilterScale=CAN_FilterScale_32bit;
		can2_filter.CAN_FilterIdHigh=0x0000;
		can2_filter.CAN_FilterIdLow=0x0000;
		can2_filter.CAN_FilterMaskIdHigh=0x0000;
		can2_filter.CAN_FilterMaskIdLow=0x0000;
		can2_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
		can2_filter.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&can2_filter);

		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
		CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
	}
  /* -------------- Configure NVIC ----------------------------------------*/
	{
		NVIC_InitTypeDef       nvic;
		
		nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		
		nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 2;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
	}
}
/*************************************************************************
                          CAN2_TX_IRQHandler
描述：CAN2的发数中断函数
*************************************************************************/
void CAN2_TX_IRQHandler(void) //CAN TX
{
  if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
		
  }
}
/*************************************************************************
                          CAN2_RX0_IRQHandler
描述：CAN2的接收中断函数
*************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
			CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
			CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
			CAN_Receive(CAN2, CAN_FIFO0, &rx_message);  
		 //电机编码器数据处理
			Can2_ReceiveMsgProcess(&rx_message);
    }

}
