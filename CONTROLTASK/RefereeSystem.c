#include "RefereeSystem.h"

//����ϵͳ��ʱ����
/*****************
����:JudgementHandle(uint8_t *pData)
���ܣ�����ϵͳ���ݴ���
*****************/
//void JudgementHandle(uint8_t *pData)
//{
//  GameInformationData.CommandCodeID = (int16_t)pData[5];
//	GameInformationData.TimeRemain = ((int32_t)pData[7]<<24)|((int32_t)pData[8]<<16)|((int32_t)pData[9]<<8)|((int32_t)pData[10]);
//	GameInformationData.BloodRemain = ((int16_t)pData[12]<<8)|((int16_t)pData[11]);                                                                                                                                                                           
//	GameInformationData.Voltage = ((int32_t)pData[16]<<24)|((int32_t)pData[15]<<16)|((int32_t)pData[14]<<8)|((int32_t)pData[13]);
//	GameInformationData.Current = ((int32_t)pData[20]<<24)|((int32_t)pData[18]<<19)|((int32_t)pData[18]<<8)|((int32_t)pData[17]);
//	GameInformationData.RemainPower = ((int32_t)pData[41]<<24)|((int32_t)pData[40]<<16)|((int32_t)pData[39]<<8)|((int32_t)pData[38]);
//	GameInformationData.ecd_Voltage = Parameter_Transformation(GameInformationData.Voltage);
//	GameInformationData.ecd_Current = Parameter_Transformation(GameInformationData.Current);
//	GameInformationData.ChassisPower = GameInformationData.ecd_Voltage*GameInformationData.ecd_Current;
//}

