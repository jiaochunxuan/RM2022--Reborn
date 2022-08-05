#include "main.h"


int recieve=0;
extern uint8_t CTRL_Flag;
unsigned char buff[5];
int16_t jiesuan_data[3];
 int shu=0;
void USART7_Configuration(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE); 
	
	/* -------------- Configure GPIO & USART2 -------------------------------*/
	GPIO_InitTypeDef gpio;
	
	USART_InitTypeDef usart;
	DMA_InitTypeDef dma;
  NVIC_InitTypeDef nvic;	
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7);
	
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &gpio);
	
	USART_DeInit(UART7);
  USART_StructInit(&usart);////����ͨѶ���ö������Ｏ����

	USART_Init(UART7, &usart);
	
	USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);
	USART_ITConfig(UART7, USART_IT_TC, ENABLE);
	USART_Cmd(UART7, ENABLE);

	nvic.NVIC_IRQChannel = UART7_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 1;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 0;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	
	
}


void USART7_Init(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //ʹ��GPIODʱ��
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//ʹ��DMA1ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//ʹ��USART3ʱ��
 /* -------------- Configure GPIO & USART3 -------------------------------*/
	{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart;
		//����7��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART7); //PD8����ΪUSART3_TX
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART7); //PD9����ΪUSART3_RX
		//USART3�˿�����
		gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
		gpio.GPIO_Mode = GPIO_Mode_AF;//���ù���
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
		gpio.GPIO_OType = GPIO_OType_PP; //���츴�����
		gpio.GPIO_PuPd = GPIO_PuPd_UP; //����
		GPIO_Init(GPIOE,&gpio); //��ʼ��PD8,PD9

		//USART3 ��ʼ������
		USART_DeInit(UART7);
		USART_StructInit(&usart);
		usart.USART_BaudRate = 9600;//����������
		usart.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		usart.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		usart.USART_Parity = USART_Parity_No;//����żУ��λ
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;	//�շ�ģʽ
		USART_Init(UART7, &usart); //��ʼ������3
		
		USART_DMACmd(UART7, USART_DMAReq_Rx, ENABLE);//��������DMA���գ�λ���Ƿ���Ի�������������������������
		USART_ITConfig(UART7, USART_IT_IDLE, ENABLE);//��������ж�
		USART_Cmd(UART7, ENABLE);  //ʹ�ܴ���3
	}
	/* -------------- Configure DMA1_Stream1 --------------------------------*/
//	{
//		DMA_InitTypeDef dma;
//		DMA_DeInit(DMA1_Stream1);
//		DMA_StructInit(&dma);
//		dma.DMA_Channel = DMA_Channel_4;
//		dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
//		dma.DMA_Memory0BaseAddr = (uint32_t)&UART7_DMA1_RX_BUF[0][0];        //����DMA���ڴ��Ŀ��λ�ã���DMA����Ҫ��ȡ����д���λ��
//		dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//		dma.DMA_BufferSize = sizeof(UART7_DMA1_RX_BUF)/2;                    //���鳤��
//		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//		dma.DMA_Mode = DMA_Mode_Circular;
//		dma.DMA_Priority = DMA_Priority_Medium;//�����ȼ�
//		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;//û�ã���������������������������������
//		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//û�ã�����������������������������
//		dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;//û�ã�������������������������������������
//		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//û�ã�����������������������������
//		DMA_Init(DMA1_Stream1, &dma);
//		
//		//����Memory1,Memory0�ǵ�һ��ʹ�õ�Memory
//		DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)&USART3_DMA1_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
//		DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
//		DMA_Cmd(DMA1_Stream1, ENABLE);
//	}
	/* -------------- Configure NVIC ----------------------------------------*/
	{
		NVIC_InitTypeDef nvic;
		//Usart3 NVIC ����
		nvic.NVIC_IRQChannel = UART7_IRQn;//����1�ж�ͨ�� Ϊʲô�򿪵��Ǵ��ڵ��жϲ���DMA���жϣ�����������
		nvic.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
		nvic.NVIC_IRQChannelSubPriority =3;		//�����ȼ�0
		nvic.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&nvic);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	}
}
//float exchange(int bai,int shi,int ge,int fu1,int fu2,int fu3)
//{float result=0;
//	result=100*(bai)+10*(shi)+ge+0.1*(fu1)+0.01*(fu2)+0.001*(fu3);
//	return result;
//}
	

//void UART7_IRQHandler(void)
//{
//	static uint32_t usart7_this_time_rx_len = 0;
//	
////	if(j>4)j=0;
//	if(USART_GetFlagStatus(UART7,USART_FLAG_TC) != RESET)
//	{

//	USART_ClearFlag(UART7,USART_FLAG_TC);
//	}

//	
//		if(USART_GetFlagStatus(UART7,USART_FLAG_RXNE) != RESET)//USART_GetITStatus
//	{
//		static int  j=0;
//		USART_ClearFlag(UART7,USART_IT_RXNE);
//		recieve=USART_ReceiveData( UART7);
//	
//		
//		buff[shu]=recieve;
//		shu++;
//		if(shu>3)
//			shu=0;
//		
//		if(buff[0]=='R'&&buff[1]=='M')
//	    {
//			jiesuan_data[0] = (buff[3]<<8) | buff[2];
//		
//	    }
//		//USART_SendData( UART7,recieve);////////////���������̷���ȥ	
//		}

//}

void uart_test(void)
{
	USART_SendData( UART7, CTRL_Flag+48);
	//USART_SendData( UART7, CTRL_Flag);
	//while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET);	
//	USART_ClearFlag(UART7,USART_FLAG_TXE);
}

