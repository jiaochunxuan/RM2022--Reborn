#include "key.h"
#include "delay.h" 
#include "DataProcessing.h"

//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY2���� 
//4��WKUP���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>WK_UP!!
uint8_t KEY_Scan_ctrl(uint8_t mode,Key* key)
{	 
	static uint8_t key_up_ctrl=1;//�������ɿ���־
	if(mode)key_up_ctrl=1;  //֧������		  
	if(key_up_ctrl&&(key->v & Key_CTRL))
	{
		key_up_ctrl=0;
		if(key->v & Key_CTRL)return 1;
	}else if(!(key->v & Key_CTRL))key_up_ctrl=1; 	    
 	return 0;// �ް�������
}

uint8_t KEY_Scan_Z(uint8_t mode,Key* key)
{	 
	static uint8_t key_up_Z=1;//�������ɿ���־
	if(mode)key_up_Z=1;  //֧������		  
	if(key_up_Z&&(key->v & Key_Z))
	{
		key_up_Z=0;
		if(key->v & Key_Z)return 1;
	}else if(!(key->v & Key_Z))key_up_Z=1; 	    
 	return 0;// �ް�������
}

uint8_t KEY_Scan_X(uint8_t mode,Key* key)
{
	static uint8_t key_up_X=1;//�������ɿ���־
	if(mode)key_up_X=1;  //֧������
	if(key_up_X&&(key->v & Key_X))
	{
		key_up_X=0;
		if(key->v & Key_X)return 1;
	}else if(!(key->v & Key_X))key_up_X=1;
 	return 0;// �ް�������
}

uint8_t KEY_Scan_C(uint8_t mode,Key* key)
{
	static uint8_t key_up_C=1;//�������ɿ���־
	if(mode)key_up_C=1;  //֧������
	if(key_up_C&&(key->v & Key_C))
	{
		key_up_C=0;
		if(key->v & Key_C)return 1;
	}else if(!(key->v & Key_X))key_up_C=1;
 	return 0;// �ް�������
}
