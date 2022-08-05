#ifndef __USART6_H__
#define __USART6_H__

#include <stm32f4xx.h>
#define IMU_DMA_RX_BUF_LEN 72u
#define IMU_FRAME_LENGTH   41u 

extern uint8_t IMU_DMA_RX_BUF[2][IMU_DMA_RX_BUF_LEN];
extern int sendflag ;
extern int16_t Gyro[3];
extern float Eular[3];
extern  uint8_t angle[12];
extern uint32_t usart6_this_time_rx_len;
void USART6_Configuration(void);
void IMUDataProcess(uint8_t *pData);

#endif
