#include "key.h"
#include "delay.h" 
#include "DataProcessing.h"

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY2按下 
//4，WKUP按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
uint8_t KEY_Scan_ctrl(uint8_t mode,Key* key)
{	 
	static uint8_t key_up_ctrl=1;//按键按松开标志
	if(mode)key_up_ctrl=1;  //支持连按		  
	if(key_up_ctrl&&(key->v & Key_CTRL))
	{
		key_up_ctrl=0;
		if(key->v & Key_CTRL)return 1;
	}else if(!(key->v & Key_CTRL))key_up_ctrl=1; 	    
 	return 0;// 无按键按下
}

uint8_t KEY_Scan_Z(uint8_t mode,Key* key)
{	 
	static uint8_t key_up_Z=1;//按键按松开标志
	if(mode)key_up_Z=1;  //支持连按		  
	if(key_up_Z&&(key->v & Key_Z))
	{
		key_up_Z=0;
		if(key->v & Key_Z)return 1;
	}else if(!(key->v & Key_Z))key_up_Z=1; 	    
 	return 0;// 无按键按下
}

uint8_t KEY_Scan_X(uint8_t mode,Key* key)
{
	static uint8_t key_up_X=1;//按键按松开标志
	if(mode)key_up_X=1;  //支持连按
	if(key_up_X&&(key->v & Key_X))
	{
		key_up_X=0;
		if(key->v & Key_X)return 1;
	}else if(!(key->v & Key_X))key_up_X=1;
 	return 0;// 无按键按下
}

uint8_t KEY_Scan_C(uint8_t mode,Key* key)
{
	static uint8_t key_up_C=1;//按键按松开标志
	if(mode)key_up_C=1;  //支持连按
	if(key_up_C&&(key->v & Key_C))
	{
		key_up_C=0;
		if(key->v & Key_C)return 1;
	}else if(!(key->v & Key_X))key_up_C=1;
 	return 0;// 无按键按下
}
