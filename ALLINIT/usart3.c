#include "main.h"
#include "usart3.h"
#include "fifo.h"
//DMA1数据流1 通道4

#define USART3_DMA_RX_BUF_LEN   108          //每级数组的长度
#define USART3_DMA_RX_buff 80           //应接受的数据量

extern unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
extern uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage,uint32_t dwLength,uint16_t wCRC);
uint8_t USART3_DMA1_RX_BUF[2][USART3_DMA_RX_BUF_LEN];
uint8_t CRC8_Ref_Value;
uint8_t CRC8_Solve_Value;
uint16_t CRC16_Ref_Value;    //收到的CRC16校验值
uint16_t CRC16_Solve_Value;  //计算得到的CRC16校验值

uint8_t Save_Element_Array[30];
uint16_t data_Length;
uint16_t Tail_Over_Zero_Value =0;   //尾指针通过零点
uint16_t Head_Over_Zero_Value =0;   //头指针通过零点
FIFO_S_t* UART_TranFifo;
uint8_t computer_tx_buf[28]={0};
ringBuffer_t buffer;
extGameRobotState_t robotState;

void USART3_Init(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//使能DMA1时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
 /* -------------- Configure GPIO & USART3 -------------------------------*/
	{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart;
		//串口3对应引脚复用映射
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //PD8复用为USART3_TX
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //PD9复用为USART3_RX
		//USART3端口配置
		gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		gpio.GPIO_Mode = GPIO_Mode_AF;//复用功能
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		gpio.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		gpio.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOD,&gpio); //初始化PD8,PD9

		//USART3 初始化设置
		USART_DeInit(USART3);
		USART_StructInit(&usart);
		usart.USART_BaudRate = 115200;//波特率设置
		usart.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		usart.USART_StopBits = USART_StopBits_1;//一个停止位
		usart.USART_Parity = USART_Parity_No;//无奇偶校验位
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;	//收发模式
		USART_Init(USART3, &usart); //初始化串口3
		
		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);//开启串口DMA接收，位置是否可以换？？？？？？？？？？？？
		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//开启相关中断
		USART_Cmd(USART3, ENABLE);  //使能串口3
	}
	/* -------------- Configure DMA1_Stream1 --------------------------------*/
	{
		DMA_InitTypeDef dma;
		DMA_DeInit(DMA1_Stream1);
		DMA_StructInit(&dma);
		dma.DMA_Channel = DMA_Channel_4;
		dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
		dma.DMA_Memory0BaseAddr = (uint32_t)&USART3_DMA1_RX_BUF[0][0];        //定义DMA在内存的目标位置，即DMA即将要读取或者写入的位置
		dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
		dma.DMA_BufferSize = sizeof(USART3_DMA1_RX_BUF)/2;                    //数组长度
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Circular;
		dma.DMA_Priority = DMA_Priority_Medium;//中优先级
		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;//没用？？？？？？？？？？？？？？？？？
		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//没用？？？？？？？？？？？？？？？
		dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;//没用？？？？？？？？？？？？？？？？？？？
		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//没用？？？？？？？？？？？？？？？
		DMA_Init(DMA1_Stream1, &dma);
		
		//配置Memory1,Memory0是第一个使用的Memory
		DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)&USART3_DMA1_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
		DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
		DMA_Cmd(DMA1_Stream1, ENABLE);
	}
	/* -------------- Configure NVIC ----------------------------------------*/
	{
		NVIC_InitTypeDef nvic;
		//Usart3 NVIC 配置
		nvic.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道 为什么打开的是串口的中断不是DMA的中断？？？？？？
		nvic.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
		nvic.NVIC_IRQChannelSubPriority =3;		//子优先级0
		nvic.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&nvic);	//根据指定的参数初始化VIC寄存器、
	}
//	UART_TranFifo = FIFO_S_Create(100);  
//  if(!UART_TranFifo)
//   {
//       // while(1);  avoid while in program
//	 }
}
void USART3_IRQHandler(void)
{
	int i,j;
	static uint32_t usart3_this_time_rx_len = 0;
//	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)//发送寄存器空中断
//	{   
//		
//		//USART_ClearITPendingBit(USART3,USART_IT_TXE);
//		if(!FIFO_S_IsEmpty(UART_TranFifo))
//		{
//		uint16_t data = (uint16_t)FIFO_S_Get(UART_TranFifo);
//		USART_SendData(USART3, data);
//		}
//		else
//		{
//		USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
//		}  
//	}	 else
   if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)      //接收到数据
	{
		//clear the idle pending flag 
		(void)USART3->SR;
		(void)USART3->DR;

		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == 0)
		{
			DMA_Cmd(DMA1_Stream1, DISABLE);
			usart3_this_time_rx_len = USART3_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);
			DMA1_Stream1->NDTR = (uint16_t)USART3_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			DMA1_Stream1->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA1_Stream1, ENABLE);
      for(i=0;i<usart3_this_time_rx_len;i++)
			{
				RingBuffer_Write(USART3_DMA1_RX_BUF[0][i]);
			}
		}
		else //Target is Memory1
		{
			DMA_Cmd(DMA1_Stream1, DISABLE);
			usart3_this_time_rx_len = USART3_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);
			DMA1_Stream1->NDTR = (uint16_t)USART3_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
			DMA1_Stream1->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA1_Stream1, ENABLE);
      for(i=0;i<usart3_this_time_rx_len;i++)
			{
				RingBuffer_Write(USART3_DMA1_RX_BUF[1][i]);
			}
		}
		while(buffer.tailPosition!=buffer.headPosition)
		{
			if(buffer.tailPosition-buffer.headPosition>=0) 
				Tail_Over_Zero_Value=0;   //未过零点
			else                                           
				Tail_Over_Zero_Value=100; //通过零点
			
			if(buffer.headPosition>=96&&buffer.headPosition<=99)  
				Head_Over_Zero_Value = 100;//读五个元素头指针过零点
			else                                                  
				Head_Over_Zero_Value = 0;  //读五个元素头指针未过零点
				
			for(j=0;j<5;j++)   //取出帧头
			{
				RingBuffer_Read(Save_Element_Array+j);
			}
			CRC8_Ref_Value   = Save_Element_Array[4];
			CRC8_Solve_Value = Get_CRC8_Check_Sum(Save_Element_Array,4,0xff);
			if(CRC8_Ref_Value == CRC8_Solve_Value)  //帧头通过CRC8校验
			{
				data_Length = Save_Element_Array[1]|Save_Element_Array[2]<<8;//本该用两个变量，为了方便用1个				
				if(buffer.tailPosition+Tail_Over_Zero_Value-(Head_Over_Zero_Value+buffer.headPosition-5)>=5+2+data_Length+2)//没看懂这个关系由来
				{
					for(j=0;j<data_Length+2+2;j++)
					{
						RingBuffer_Read(Save_Element_Array+5+j);
					}
					CRC16_Ref_Value   = Save_Element_Array[5+2+data_Length+2-2]|Save_Element_Array[5+2+data_Length+2-1]<<8;
					CRC16_Solve_Value = Get_CRC16_Check_Sum(Save_Element_Array,7+data_Length+2-2,0xffff);
					if(CRC16_Ref_Value == CRC16_Solve_Value)  //通过CRC16校验
					{
						/*在这里进行数据定位，Save_Element_Array[5]和[6]表示cmd_id*/
						if(Save_Element_Array[5]==0x01&&Save_Element_Array[6]==0x02)
						{
							getRobotState(Save_Element_Array);
						}
					}
				}
				else
				{
					buffer.headPosition = Head_Over_Zero_Value+buffer.headPosition-5;
					break;
				}
			}
	  }
	}
}

/*
void UART3_PrintBlock(uint8_t* pdata, uint8_t len)//数组发送
{
//	 uint8_t i = 0;
//    for(i = 0; i < len+9; i++)
//    {
//			USART_SendData(USART3,pdata[i]);
//			while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==0);
//		}
}
*/

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

void UART3_PrintBlock1(void)
{
	 uint8_t i = 0;
    for(i = 0; i < 28; i++)
    {
			USART_SendData(USART3,computer_tx_buf[i]);
			while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==0);
	}
}

/***
函数：void getRobotState(uint8_t *stateData)
功能：从裁判系统读取机器人状态
备注：stateData角标0从帧头开始计算
	  cmd_ID：0x0201
      共26字节，下标7为本机器人ID，下标8为机器人等级
***/
void getRobotState(uint8_t *stateData)
{
	robotState.robot_id= stateData[7];//机器人id
	robotState.robotLevel = stateData[8];//机器人等级
}
