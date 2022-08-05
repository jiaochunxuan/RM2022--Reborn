#include "EncoderTask.h"
/*变量定义*/
static uint32_t can_count = 0;
static uint32_t can2_count = 0;
//底盘电机编码器
volatile Encoder CM1Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CM2Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CM3Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CM4Encoder		 	= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
//抬升电机编码器
volatile Encoder LM1Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder LM2Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
//翻转电机编码器
volatile Encoder TM1Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder TM2Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder TM3Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder TM4Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder TM5Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder TM6Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder TM7Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder TM8Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
extern int pre;
/*函数实现*/
void GetEncoderBias(volatile Encoder *v, CanRxMsg *msg)
{
	v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
	v->raw_value= v->ecd_bias;
	v->ecd_value = v->ecd_bias;
}

void CanReceiveMsgProcess(CanRxMsg * msg)
{
    can_count++;
		switch(msg->StdId)
		{
			case CAN_BUS1_CM1_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM1Encoder ,msg):Motor_3508_EncoderProcess(&CM1Encoder,msg);       //获取到编码器的初始偏差值    
//				wwdg_flag |=0x0001;
			}break;
			case CAN_BUS1_CM2_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM2Encoder,msg):Motor_3508_EncoderProcess(&CM2Encoder,msg);
//				wwdg_flag |=0x0002;
			}break;
			case CAN_BUS1_CM3_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM3Encoder ,msg):Motor_3508_EncoderProcess(&CM3Encoder ,msg); 
//				wwdg_flag |=0x0004;
			}break;
			case CAN_BUS1_CM4_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM4Encoder,msg):Motor_3508_EncoderProcess(&CM4Encoder ,msg);
//				wwdg_flag |=0x0008;
			}break;
			case CAN_BUS1_LM1_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&LM1Encoder,msg):Motor_3508_EncoderProcess(&LM1Encoder,msg); 
//				wwdg_flag |=0x0010;
			}break;
//			
			case CAN_BUS1_LM2_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&LM2Encoder,msg):Motor_3508_EncoderProcess(&LM2Encoder,msg); 
//				wwdg_flag |=0x0020;
			}break;
			
//			case CAN_BUS1_TM4_FEEDBACK_MSG_ID:  
//			{
//				//(can_count<=50) ? GetEncoderBias(&TM4Encoder ,msg):Motor_2006_EncoderProcess(&TM4Encoder,msg);
//			}break;
			
			default:
			{
			}
	}
}

void Can2_ReceiveMsgProcess(CanRxMsg * msg)
{
	  can2_count++;
		switch(msg->StdId)
		{
			case CAN_BUS2_TM1_FEEDBACK_MSG_ID:  
			{
				(can2_count<=50) ? GetEncoderBias(&TM1Encoder,msg):Motor_3508_EncoderProcess(&TM1Encoder,msg);       //获取到编码器的初始偏差值    
			}break;
			case CAN_BUS2_TM2_FEEDBACK_MSG_ID:  
			{
				(can2_count<=50) ? GetEncoderBias(&TM2Encoder ,msg):Motor_3508_EncoderProcess(&TM2Encoder,msg);
			}break;
			case CAN_BUS2_TM3_FEEDBACK_MSG_ID:  
			{
				(can2_count<=50) ? GetEncoderBias(&TM3Encoder ,msg):Motor_2006_EncoderProcess(&TM3Encoder,msg);
			}break;	
			case CAN_BUS2_TM4_FEEDBACK_MSG_ID:  
			{
				(can2_count<=50) ? GetEncoderBias(&TM4Encoder ,msg):Motor_2006_EncoderProcess(&TM4Encoder,msg);
			}break;
			case CAN_BUS2_TM5_FEEDBACK_MSG_ID:  
			{
				(can2_count<=50) ? GetEncoderBias(&TM5Encoder ,msg):Motor_3508_EncoderProcess(&TM5Encoder,msg);
			}break;
			case CAN_BUS2_TM6_FEEDBACK_MSG_ID:  
			{
				(can2_count<=50) ? GetEncoderBias(&TM6Encoder ,msg):Motor_3508_EncoderProcess(&TM6Encoder,msg);
			}break;
			case CAN_BUS2_TM7_FEEDBACK_MSG_ID:  
			{
				(can2_count<=50) ? GetEncoderBias(&TM7Encoder ,msg):Motor_3508_EncoderProcess(&TM7Encoder,msg);
			}break;
			case CAN_BUS2_TM8_FEEDBACK_MSG_ID:  
			{
				(can2_count<=50) ? GetEncoderBias(&TM8Encoder ,msg):Motor_3508_EncoderProcess(&TM8Encoder,msg);
			}break;		
			default:
			{
			}
	 }
		//include_round_engle();
	 
	 if(pre == 1)
	 {
	 	GetEncoderBias(&TM1Encoder,msg);
		GetEncoderBias(&TM2Encoder,msg);
		 
		TM1Encoder.last_raw_value=0;
		TM1Encoder.diff=0;
		TM1Encoder.buf_count=0;
		TM1Encoder.ecd_raw_rate=0;
		TM1Encoder.round_cnt=0;
		TM1Encoder.filter_rate=0.0;
		TM1Encoder.ecd_angle=0.0;
		TM1Encoder.ecd_xtl_angle=0.0;
		TM1Encoder.real_torque_current=0.0; 
		 
		TM2Encoder.last_raw_value=0;
		TM2Encoder.diff=0;
		TM2Encoder.buf_count=0;
		TM2Encoder.ecd_raw_rate=0;
		TM2Encoder.round_cnt=0;
		TM2Encoder.filter_rate=0.0;
		TM2Encoder.ecd_angle=0.0;
		TM2Encoder.ecd_xtl_angle=0.0;
		TM2Encoder.real_torque_current=0.0; 
		
		pre = 0;
	 }
	 
	 if(pre == 2)
	 {
	 	GetEncoderBias(&TM5Encoder,msg);
		GetEncoderBias(&TM6Encoder,msg);
		 
		TM5Encoder.last_raw_value=0;
		TM5Encoder.diff=0;
		TM5Encoder.buf_count=0;
		TM5Encoder.ecd_raw_rate=0;
		TM5Encoder.round_cnt=0;
		TM5Encoder.filter_rate=0.0;
		TM5Encoder.ecd_angle=0.0;
		TM5Encoder.ecd_xtl_angle=0.0;
		TM5Encoder.real_torque_current=0.0; 
		 
		TM6Encoder.last_raw_value=0;
		TM6Encoder.diff=0;
		TM6Encoder.buf_count=0;
		TM6Encoder.ecd_raw_rate=0;
		TM6Encoder.round_cnt=0;
		TM6Encoder.filter_rate=0.0;
		TM6Encoder.ecd_angle=0.0;
		TM6Encoder.ecd_xtl_angle=0.0;
		TM6Encoder.real_torque_current=0.0; 
		
		pre = 0;
	 }

}

void Motor_3508_EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;
  int16_t temp_filter=0;	
	int16_t torque_current = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//保存diff
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (double)(v->raw_value - v->ecd_bias)*(360/19.0)/8192 + v->round_cnt * 360/19.0;//减速比19:1
	
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	//v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);	
	temp_filter = ((msg->Data[2]<<8)|msg->Data[3]);
	v->filter_rate = temp_filter/19;
	torque_current=((msg->Data[4]<<8)|msg->Data[5]);
	v->real_torque_current = torque_current;
}

void Motor_3508_EncoderProcess1(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;
  int16_t temp_filter=0;	
	int16_t torque_current = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//保存diff
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (double)(v->raw_value - v->ecd_bias)*(360/14.0)/8192 + v->round_cnt * 360/14.0;//减速比19:1
	
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	//v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);	
	temp_filter = ((msg->Data[2]<<8)|msg->Data[3]);
	v->filter_rate = temp_filter/19;
	torque_current=((msg->Data[4]<<8)|msg->Data[5]);
	v->real_torque_current = torque_current;
}


void Motor_2006_EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;   
  int16_t temp_filter=0;		
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	temp_filter = (msg->Data[2]<<8)|msg->Data[3];
	v->filter_rate = temp_filter/36;
	v->diff = v->raw_value - v->last_raw_value;
	
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//保存diff
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (double)(v->raw_value - v->ecd_bias)*(360/36.0)/8192 + v->round_cnt * 360/36.0;//减速比36:1
	
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
//	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);	
}

void EncoderReset(volatile Encoder *v)
{
//	v->buf_count=0;
//	v->diff=0;
//	v->ecd_angle=0;
//	v->ecd_bias=0;
//	v->ecd_raw_rate=0;
	v->ecd_value=0;
//	v->ecd_xtl_angle=0;
//	v->filter_rate=0;
//	v->last_raw_value=0;
//	v->rate_buf[0]=0;
//	v->rate_buf[1]=0;
//	v->rate_buf[2]=0;
//	v->rate_buf[3]=0;
//	v->rate_buf[4]=0;
//	v->rate_buf[5]=0;
//	v->raw_value=0;
//	v->real_torque_current=0;
	v->round_cnt=0;
}
