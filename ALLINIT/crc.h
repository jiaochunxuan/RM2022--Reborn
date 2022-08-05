#ifndef __CRC_H__
#define __CRC_H__
#include "stm32f4xx.h"

extern unsigned char CRC8_INIT;
extern uint16_t CRC_INIT;

extern uint8_t  computer_tx_picture_buf[70];

void Append_CRC8_Check_Sum(unsigned char*pchMessage,unsigned int dwLength);
void Append_CRC16_Check_Sum(uint8_t* pchMessage,uint32_t dwLength);
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage,uint32_t dwLength,uint16_t wCRC);
#endif
