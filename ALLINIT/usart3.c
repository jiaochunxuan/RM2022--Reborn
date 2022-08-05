#include "main.h"
#include "usart3.h"
#include "fifo.h"
//DMA1������1 ͨ��4

#define USART3_DMA_RX_BUF_LEN   108          //ÿ������ĳ���
#define USART3_DMA_RX_buff 80           //Ӧ���ܵ�������

extern unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
extern uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage,uint32_t dwLength,uint16_t wCRC);
uint8_t USART3_DMA1_RX_BUF[2][USART3_DMA_RX_BUF_LEN];
uint8_t CRC8_Ref_Value;
uint8_t CRC8_Solve_Value;
uint16_t CRC16_Ref_Value;    //�յ���CRC16У��ֵ
uint16_t CRC16_Solve_Value;  //����õ���CRC16У��ֵ

uint8_t Save_Element_Array[30];
uint16_t data_Length;
uint16_t Tail_Over_Zero_Value =0;   //βָ��ͨ�����
uint16_t Head_Over_Zero_Value =0;   //ͷָ��ͨ�����
FIFO_S_t* UART_TranFifo;
uint8_t computer_tx_buf[28]={0};
ringBuffer_t buffer;
extGameRobotState_t robotState;

void USART3_Init(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIODʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//ʹ��DMA1ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
 /* -------------- Configure GPIO & USART3 -------------------------------*/
	{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart;
		//����3��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //PD8����ΪUSART3_TX
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //PD9����ΪUSART3_RX
		//USART3�˿�����
		gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		gpio.GPIO_Mode = GPIO_Mode_AF;//���ù���
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
		gpio.GPIO_OType = GPIO_OType_PP; //���츴�����
		gpio.GPIO_PuPd = GPIO_PuPd_UP; //����
		GPIO_Init(GPIOD,&gpio); //��ʼ��PD8,PD9

		//USART3 ��ʼ������
		USART_DeInit(USART3);
		USART_StructInit(&usart);
		usart.USART_BaudRate = 115200;//����������
		usart.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		usart.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		usart.USART_Parity = USART_Parity_No;//����żУ��λ
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;	//�շ�ģʽ
		USART_Init(USART3, &usart); //��ʼ������3
		
		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);//��������DMA���գ�λ���Ƿ���Ի�������������������������
		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//��������ж�
		USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3
	}
	/* -------------- Configure DMA1_Stream1 --------------------------------*/
	{
		DMA_InitTypeDef dma;
		DMA_DeInit(DMA1_Stream1);
		DMA_StructInit(&dma);
		dma.DMA_Channel = DMA_Channel_4;
		dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
		dma.DMA_Memory0BaseAddr = (uint32_t)&USART3_DMA1_RX_BUF[0][0];        //����DMA���ڴ��Ŀ��λ�ã���DMA����Ҫ��ȡ����д���λ��
		dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
		dma.DMA_BufferSize = sizeof(USART3_DMA1_RX_BUF)/2;                    //���鳤��
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Circular;
		dma.DMA_Priority = DMA_Priority_Medium;//�����ȼ�
		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;//û�ã���������������������������������
		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//û�ã�����������������������������
		dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;//û�ã�������������������������������������
		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//û�ã�����������������������������
		DMA_Init(DMA1_Stream1, &dma);
		
		//����Memory1,Memory0�ǵ�һ��ʹ�õ�Memory
		DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)&USART3_DMA1_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
		DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
		DMA_Cmd(DMA1_Stream1, ENABLE);
	}
	/* -------------- Configure NVIC ----------------------------------------*/
	{
		NVIC_InitTypeDef nvic;
		//Usart3 NVIC ����
		nvic.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ�� Ϊʲô�򿪵��Ǵ��ڵ��жϲ���DMA���жϣ�����������
		nvic.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
		nvic.NVIC_IRQChannelSubPriority =3;		//�����ȼ�0
		nvic.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&nvic);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
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
//	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)//���ͼĴ������ж�
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
   if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)      //���յ�����
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
				Tail_Over_Zero_Value=0;   //δ�����
			else                                           
				Tail_Over_Zero_Value=100; //ͨ�����
			
			if(buffer.headPosition>=96&&buffer.headPosition<=99)  
				Head_Over_Zero_Value = 100;//�����Ԫ��ͷָ������
			else                                                  
				Head_Over_Zero_Value = 0;  //�����Ԫ��ͷָ��δ�����
				
			for(j=0;j<5;j++)   //ȡ��֡ͷ
			{
				RingBuffer_Read(Save_Element_Array+j);
			}
			CRC8_Ref_Value   = Save_Element_Array[4];
			CRC8_Solve_Value = Get_CRC8_Check_Sum(Save_Element_Array,4,0xff);
			if(CRC8_Ref_Value == CRC8_Solve_Value)  //֡ͷͨ��CRC8У��
			{
				data_Length = Save_Element_Array[1]|Save_Element_Array[2]<<8;//����������������Ϊ�˷�����1��				
				if(buffer.tailPosition+Tail_Over_Zero_Value-(Head_Over_Zero_Value+buffer.headPosition-5)>=5+2+data_Length+2)//û���������ϵ����
				{
					for(j=0;j<data_Length+2+2;j++)
					{
						RingBuffer_Read(Save_Element_Array+5+j);
					}
					CRC16_Ref_Value   = Save_Element_Array[5+2+data_Length+2-2]|Save_Element_Array[5+2+data_Length+2-1]<<8;
					CRC16_Solve_Value = Get_CRC16_Check_Sum(Save_Element_Array,7+data_Length+2-2,0xffff);
					if(CRC16_Ref_Value == CRC16_Solve_Value)  //ͨ��CRC16У��
					{
						/*������������ݶ�λ��Save_Element_Array[5]��[6]��ʾcmd_id*/
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
void UART3_PrintBlock(uint8_t* pdata, uint8_t len)//���鷢��
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
������void getRobotState(uint8_t *stateData)
���ܣ��Ӳ���ϵͳ��ȡ������״̬
��ע��stateData�Ǳ�0��֡ͷ��ʼ����
	  cmd_ID��0x0201
      ��26�ֽڣ��±�7Ϊ��������ID���±�8Ϊ�����˵ȼ�
***/
void getRobotState(uint8_t *stateData)
{
	robotState.robot_id= stateData[7];//������id
	robotState.robotLevel = stateData[8];//�����˵ȼ�
}
