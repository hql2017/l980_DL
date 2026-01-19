
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 

#include "CAN_modbusRTU_bsp.h"
#include "fdcan.h"
#include "tim.h"
#include "main.h"

   /**
  * @brief CAN_modbusRTU_init
  * @param  void
  * @note   
  * @retval None
  */
void CAN_modbusRTU_init(void)
{
     
}
/************************************************************************//**
  * @brief  CAN_crc16Num
  * @param   
  * @note    CRC校验函数,LSB
  * @retval None
  ****************************************************************************/
static unsigned short int CAN_crc16Num(unsigned char *pData,unsigned  int length)
{
	uint16_t crc = 0xFFFF;	
	for (int i = 0; i < length; i++)
	{
		crc ^= pData[i];			
		for (int j = 0; j < 8; j++)
		{
			if (crc & 1)
			{
				crc >>= 1;
				crc ^= 0xA001;			
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	return crc;
}

/************************************************************************//**
  * @brief  CAN_transmitPackage
  * @param   
  * @note    
  * @retval None
  ****************************************************************************/
 void CAN_transmitPackage(unsigned char function,unsigned char code,unsigned char *data)
 {
   unsigned char transmitBuff[8];
   transmitBuff[0]=function;
   if(function==RTU_CODE_LONG_BYTES_PACKAGE)
   {
    unsigned char tansNum=0;
    transmitBuff[1]=code;
    while(tansNum<code)
    {
      memcpy(&transmitBuff[2],data,5);
      transmitBuff[7]=tansNum;
      data+=5;
      tansNum++;
      APP_CAN_SEND_DATA(transmitBuff,8,CAN_MASTER_ID);
      HAL_Delay(1);
    }    
   }
   else 
   {
    transmitBuff[1]=code|L980_CODE_MASK; 
    memcpy(&transmitBuff[2],data,4);
    transmitBuff[6]=(CAN_crc16Num(transmitBuff,4)>>8)&0xFF;
    transmitBuff[7]=CAN_crc16Num(transmitBuff,4)&0xFF;    
    APP_CAN_SEND_DATA(transmitBuff,8,CAN_MASTER_ID);
   }
   
 } 
 /************************************************************************//**
  * @brief  CAN_heart_ack
  * @param   
  * @note    
  * @retval None
  ****************************************************************************/
void CAN_packe_ack(void)
{
  unsigned char sendBuff[8];
  sendBuff[0]=1; 
  sendBuff[1]=2; 
  sendBuff[2]=3;
  sendBuff[3]=4;
  CAN_transmitPackage(RTU_CODE_R_SINGLE_REG,L980_REG_HEART_STATUS,sendBuff);
}
 /************************************************************************//**
  * @brief  L980_appDataParaphrase
  * @param   
  * @note    can 数据处理
  * @retval None
  ****************************************************************************/
 void CAN_appLongPackageDataParaphrase(unsigned char *data,unsigned char Len)
 {

 }
 /************************************************************************//**
  * @brief  CAN_LongPackageHandle
  * @param   
  * @note    can 数据处理
  * @retval None
  ****************************************************************************/
 void CAN_LongPackageHandle( can_long_package *pPkt)
 {
  unsigned char data[8],Len;
    //包组合，校验数据
    CAN_appLongPackageDataParaphrase(data,Len);
 }
  /************************************************************************//**
  * @brief  L980_appSingleReadAck
  * @param    
  * @note   单通道读
  * @retval None
  ****************************************************************************/
 void L980_appSingleReadAck(unsigned char reg,unsigned char *data)
 {
  switch(reg)
  {
    case  L980_NONE_CMD:
    break;
    case L980_REG_HEART_STATUS:
        CAN_packe_ack();
    break;
  }
 }
 /************************************************************************//**
  * @brief  L980_appSingleWriteAck
  * @param    
  * @note   单通道写
  * @retval None
  ****************************************************************************/
 void L980_appSingleWriteAck(unsigned char reg,unsigned char *data)
 {



 }
  /************************************************************************//**
  * @brief  L980_appMutipleReadAck
  * @param    
  * @note   多通道读
  * @retval None
  ****************************************************************************/
 void L980_appMutipleRegReadAck(unsigned char startReg,unsigned char offeset,unsigned char *data)
 {

 }
 /************************************************************************//**
  * @brief  L980_appMutipleRegWriteAck
  * @param    
  * @note   多通道写
  * @retval None
  ****************************************************************************/
 void L980_appMutipleRegWriteAck(unsigned char startReg,unsigned char offeset,unsigned char *data)
 {


 }
 /************************************************************************//**
  * @brief  L980_appDataParaphrase
  * @param    
  * @note   L980应用数据解析
  * @retval None
  ****************************************************************************/
 void L980_appDataParaphrase(L980_can_app_package *pPkt)
 {
    unsigned short int crcValue= (pPkt->crcH<<8)|pPkt->crcL;
    if(crcValue!=CAN_crc16Num((unsigned char *)&pPkt,6)) return ;
    switch(pPkt->functionCode)
    {
      case RTU_CODE_NONE:
      break;
      case RTU_CODE_R_SINGLE_REG:             
        L980_appSingleReadAck(pPkt->laser980Reg,pPkt->data);
      break;
      case RTU_CODE_W_SINGLE_REG:
        L980_appSingleWriteAck(pPkt->laser980Reg,pPkt->data);
      break;
      case RTU_CODE_R_MULTIPLE_REG:
        L980_appMutipleRegReadAck(pPkt->laser980Reg,2,pPkt->data);
      break;
      case RTU_CODE_W_MULTIPLE_REG:        
        L980_appMutipleRegWriteAck(pPkt->laser980Reg,2,pPkt->data);
      break;       
      default:
      break;
    } 
 }
/************************************************************************//**
  * @brief  CAN_receivePackageHandle
  * @param  buff:数据缓存
  * @note   标准帧，固定长度8bytes
  * @retval None
  ****************************************************************************/
 void CAN_receivePackageHandle(unsigned char *buff,unsigned char Len)
 {     
    if((buff[0])==RTU_CODE_LONG_BYTES_PACKAGE) 
    {
      CAN_LongPackageHandle((can_long_package*)buff);
    } 
    else    
    {      
      L980_appDataParaphrase((L980_can_app_package *)buff);
    }
 }


