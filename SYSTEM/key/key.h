#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
#include "RemoteTask.h"

uint8_t KEY_Scan_ctrl(uint8_t mode,Key* key);//����ɨ�躯��_ctrl
uint8_t KEY_Scan_Z(uint8_t mode,Key* key);//����ɨ�躯��_Z
uint8_t KEY_Scan_X(uint8_t mode,Key* key);//����ɨ�躯��_X
uint8_t KEY_Scan_C(uint8_t mode,Key* key);//����ɨ�躯��_C

#endif
