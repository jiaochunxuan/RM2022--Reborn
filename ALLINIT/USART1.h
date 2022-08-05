#ifndef __USART1_H__
#define __USART1_H__
#include "main.h"
/*串口1初始化声明*/

#define REMOTE_DMA_RX_BUF_LEN 30u

#define RC_FRAME_LENGTH   18u

extern int remote_error_time;
void USART1_Init(void);

#endif
