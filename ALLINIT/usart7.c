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
  USART_StructInit(&usart);////所有通讯设置都在这里集成了

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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //使能GPIOD时钟
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//使能DMA1时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//使能USART3时钟
 /* -------------- Configure GPIO & USART3 -------------------------------*/
	{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart;
		//串口7对应引脚复用映射
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART7); //PD8复用为USART3_TX
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART7); //PD9复用为USART3_RX
		//USART3端口配置
		gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
		gpio.GPIO_Mode = GPIO_Mode_AF;//复用功能
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		gpio.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		gpio.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOE,&gpio); //初始化PD8,PD9

		//USART3 初始化设置
		USART_DeInit(UART7);
		USART_StructInit(&usart);
		usart.USART_BaudRate = 9600;//波特率设置
		usart.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		usart.USART_StopBits = USART_StopBits_1;//一个停止位
		usart.USART_Parity = USART_Parity_No;//无奇偶校验位
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;	//收发模式
		USART_Init(UART7, &usart); //初始化串口3
		
		USART_DMACmd(UART7, USART_DMAReq_Rx, ENABLE);//开启串口DMA接收，位置是否可以换？？？？？？？？？？？？
		USART_ITConfig(UART7, USART_IT_IDLE, ENABLE);//开启相关中断
		USART_Cmd(UART7, ENABLE);  //使能串口3
	}
	/* -------------- Configure DMA1_Stream1 --------------------------------*/
//	{
//		DMA_InitTypeDef dma;
//		DMA_DeInit(DMA1_Stream1);
//		DMA_StructInit(&dma);
//		dma.DMA_Channel = DMA_Channel_4;
//		dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
//		dma.DMA_Memory0BaseAddr = (uint32_t)&UART7_DMA1_RX_BUF[0][0];        //定义DMA在内存的目标位置，即DMA即将要读取或者写入的位置
//		dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//		dma.DMA_BufferSize = sizeof(UART7_DMA1_RX_BUF)/2;                    //数组长度
//		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//		dma.DMA_Mode = DMA_Mode_Circular;
//		dma.DMA_Priority = DMA_Priority_Medium;//中优先级
//		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;//没用？？？？？？？？？？？？？？？？？
//		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//没用？？？？？？？？？？？？？？？
//		dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;//没用？？？？？？？？？？？？？？？？？？？
//		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//没用？？？？？？？？？？？？？？？
//		DMA_Init(DMA1_Stream1, &dma);
//		
//		//配置Memory1,Memory0是第一个使用的Memory
//		DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)&USART3_DMA1_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
//		DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
//		DMA_Cmd(DMA1_Stream1, ENABLE);
//	}
	/* -------------- Configure NVIC ----------------------------------------*/
	{
		NVIC_InitTypeDef nvic;
		//Usart3 NVIC 配置
		nvic.NVIC_IRQChannel = UART7_IRQn;//串口1中断通道 为什么打开的是串口的中断不是DMA的中断？？？？？？
		nvic.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
		nvic.NVIC_IRQChannelSubPriority =3;		//子优先级0
		nvic.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&nvic);	//根据指定的参数初始化VIC寄存器、
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
//		//USART_SendData( UART7,recieve);////////////收完数立刻发出去	
//		}

//}

void uart_test(void)
{
	USART_SendData( UART7, CTRL_Flag+48);
	//USART_SendData( UART7, CTRL_Flag);
	//while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET);	
//	USART_ClearFlag(UART7,USART_FLAG_TXE);
}

