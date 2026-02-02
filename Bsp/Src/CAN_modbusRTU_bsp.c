
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 

#include "CAN_modbusRTU_bsp.h"
#include "fdcan.h"
#include "tim.h"
#include "main.h"

U_L980_STATUS  u_s_l980;
U_L980_CONFIG_PARAM u_l980;
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
static unsigned short int CAN_crc16Num(unsigned char *pData, int length)
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
  * @brief  CAN_r_w_ack
  * @param   packNum:总包数
  * @note   读、写回应 
  * @retval None
  ****************************************************************************/
 void CAN_r_w_ack(unsigned char reg,unsigned char dataLen,unsigned char *data)
 {
  unsigned char sendBuff[128],packNum,len;  
    len= dataLen  +4;
    if(len>128)  return ;   
    sendBuff[0]=reg|L980_REG_WRITE_MASK; 
    sendBuff[1]=dataLen;
    memcpy(&sendBuff[2],data,dataLen);
    sendBuff[dataLen+3]=(CAN_crc16Num(sendBuff,dataLen+2)>>8)&0xFF;
    sendBuff[dataLen+4]=CAN_crc16Num(sendBuff,dataLen+2)&0xFF;
    if(dataLen>4)
    {
      packNum=(len)>>3;///8;
      if((len)%8!=0)
      {
        packNum+=1;      
      }
      CAN_RTU_transmitPackage(RTU_CODE_LONG_BYTES_PACKAGE,packNum,sendBuff);
    }
    else
    { 
      packNum=1;
      CAN_RTU_transmitPackage(RTU_CODE_SINGLE_PACKAGE,packNum,sendBuff);
    }
 }
/************************************************************************//**
  * @brief  CAN_RTU_transmitPackage
  * @param   packNum:总包数
  * @note    
  * @retval None
  ****************************************************************************/
 void CAN_RTU_transmitPackage(unsigned char function,unsigned char packNum,unsigned char *data)
 {
  if(function==RTU_CODE_LONG_BYTES_PACKAGE)
  {//分包
    unsigned char transmitBuff[8];
    unsigned char tansNum=0; 
    transmitBuff[0]= packNum;//总包数  
    while(tansNum<packNum)
    {
      memcpy(&transmitBuff[1],data,6);
      transmitBuff[7]=tansNum;
      data+=6;
      tansNum++;      
      APP_CAN_SEND_DATA(transmitBuff,8,CAN_RTU_SLAVE_ID+RTU_CODE_LONG_BYTES_PACKAGE);
      HAL_Delay(1);
    }    
  }
  else 
  {       
    APP_CAN_SEND_DATA(data,8,CAN_RTU_SLAVE_ID);
  }  
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
  * @brief  L980_appReadAck
  * @param    
  * @note   单通道读
  * @retval None
  ****************************************************************************/
 void L980_appReadAck(unsigned char reg,unsigned char *data)
 {
    unsigned char transmitBuff[8];
    switch(reg)
    {   
      case L980_REG_HEART_STATUS:          
          CAN_r_w_ack(reg,sizeof(L980_STATUS),u_s_l980.data);
        break;
      case L980_REG_PULSE_COUNT_AND_TIME:           
        
          CAN_r_w_ack(reg,4,(unsigned char *)&u_s_l980.sta.laserUseTimeS);
        break;
        case L980_REG_ENERGE_PARAM:        
          transmitBuff[0]=u_l980.set_param.energeSet&0xFF;
          transmitBuff[1]=(u_l980.set_param.energeSet>>8)&0xFF;
          transmitBuff[3]=u_l980.set_param.energeCaliSet&0xFF;
          transmitBuff[4]=(u_l980.set_param.energeCaliSet>>8)&0xFF;
          CAN_r_w_ack(reg,4,transmitBuff);
      break;    
      case L980_REG_AUXILIARY_BULB:           
        transmitBuff[0]=u_l980.set_param.auxLedBulbDutySet;
        transmitBuff[1]=u_l980.set_param.auxLedBulbFreqSet;         
          CAN_r_w_ack(reg,2,transmitBuff);
        break;   
      case L980_REG_MOTOR_POSITION: 
            transmitBuff[0]= u_l980.set_param.positionSet&0xFF;
            transmitBuff[1]=(u_l980.set_param.positionSet>>8)&0xFF;          
            transmitBuff[2]=0;//电机运行状态
            transmitBuff[3]=0;    
            CAN_r_w_ack(reg,4,transmitBuff);      
        break;
      case L980_REG_LASER_TEMPRATURE: 
            CAN_r_w_ack(reg,2,(unsigned char *)&u_l980.set_param.positionSet);
        break;
      case L980_REG_COUNTDOWN_TIMERS:
            if((u_s_l980.sta.staByte&L980_STA_TIMERS_BIT3)==L980_STA_TIMERS_BIT3) 
            {
              transmitBuff[0]=1;              
            }
            else 
            {             
              transmitBuff[0]=0;  
            }       
            transmitBuff[1]=0;
            transmitBuff[2]= u_l980.set_param.timerSet&0xFF;
            transmitBuff[3]=( u_l980.set_param.timerSet>>8)&0xFF;
            CAN_r_w_ack(reg,4,transmitBuff);
        break;
      case L980_REG_SYNC_CONFIG:     
          CAN_r_w_ack(reg,sizeof(L980_SET_PARAM),u_l980.data);         
        break;        
      default:
        break;
    }
 }
 /************************************************************************//**
  * @brief  L980_appWriteAck
  * @param    
  * @note   单通道写
  * @retval None
  ****************************************************************************/
 void L980_appWriteAck(unsigned char reg,unsigned char *data)
 {
  unsigned char transmitBuff[8];
  switch(reg)
  {
    case L980_REG_ENERGE_PARAM:  
          u_l980.set_param.energeSet=(data[1]<<8)|data[0];
          u_l980.set_param.energeCaliSet=(data[3]<<8)|data[2];         
          DEBUG_PRINTF("energe Set OK=%dmj cli=%d\r\n",u_l980.set_param.energeSet,u_l980.set_param.energeCaliSet);         
          CAN_r_w_ack(reg,4,data);
      break;  
    case L980_REG_CTR_PRO_HOT:   
          if(data[0]==1)
          {
            u_l980.set_param.energeSet=(data[3]<<8)|data[2];
            laser_ctr_param.pro_hot=1;
            DEBUG_PRINTF("980 prohot energe=%d\r\n",u_l980.set_param.energeSet);           
          }
          else
          {  
            laser_ctr_param.pro_hot=0;
            DEBUG_PRINTF("980 prohot close,wait status fresh\r\n");
          } 
          app_laser_prohot_semo();    
          CAN_r_w_ack(reg,4,data);
      break;    
    case L980_REG_JT_CTR_STOP:
          if(data[0]==1)
          {          
            laser_ctr_param.JT_laser_out=1;
            DEBUG_PRINTF("980 pulse out \r\n"); 
          }
          else
          {
            laser_ctr_param.JT_laser_out=0;
            DEBUG_PRINTF("980 stop pulse out  \r\n");  
          }  
          CAN_r_w_ack(reg,2,data);
      break;
    case L980_REG_PULSE_COUNT_AND_TIME:
          u_s_l980.sta.laserUseTimeS=(data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0];
          DEBUG_PRINTF("980 laser use time  change OK\r\n");
          u_sys_param.sys_config_param.laser_use_timeS=u_s_l980.sta.laserUseTimeS;        
          CAN_r_w_ack(reg,4,(unsigned char *)&u_s_l980.sta.laserUseTimeS);
      break;    
    case L980_REG_AUXILIARY_BULB:       
        u_l980.set_param.auxLedBulbDutySet=data[0];
        u_l980.set_param.auxLedBulbFreqSet=data[1];
        DEBUG_PRINTF("980 set led ok duty=%d f=%d\r\n", u_l980.set_param.auxLedBulbDutySet,u_l980.set_param.auxLedBulbFreqSet); 
        CAN_r_w_ack(reg,2,data);
      break;   
    case L980_REG_MOTOR_POSITION:
        u_l980.set_param.positionSet=(data[1]<<8)|data[0];//
        unsigned short int temp;
        temp=(data[3]<<8)|data[2];      
        app_motor_move_to_sem(temp);       
        CAN_r_w_ack(reg,4,data);             
      break;
    case L980_REG_LASER_TEMPRATURE:
          u_l980.set_param.targetTempratureSet=(data[1]<<8)|data[0];//
          DEBUG_PRINTF("980 set work emprature OK=%d℃r\n",u_l980.set_param.targetTempratureSet); 
          CAN_r_w_ack(reg,2, (unsigned char *)&u_l980.set_param.targetTempratureSet);    
      break;
    case L980_REG_COUNTDOWN_TIMERS:
          if(data[0]!=0) 
          {
            u_s_l980.sta.staByte|=L980_STA_TIMERS_BIT3;
          }
          else 
          {
            u_s_l980.sta.staByte&=(~(L980_STA_TIMERS_BIT3));
          } 
          u_l980.set_param.timerSet=(data[3]<<8)|data[2];          
          DEBUG_PRINTF("980 counter timer set OK= %ds\r\n",u_l980.set_param.timerSet);
          CAN_r_w_ack(reg,2, data);       
      break;
      case  L980_REG_SYNC_CONFIG:
          memcpy(u_l980.data,data,sizeof(L980_SET_PARAM)); 
          transmitBuff[0]=1;//ok
          transmitBuff[1]=0;
          CAN_r_w_ack(reg,2, transmitBuff); 
          DEBUG_PRINTF("980 config param set OK\r\n");
      break;
    default:
      break;
  }
 }

 /************************************************************************//**
  * @brief  L980_appRegDataParaphrase
  * @param    
  * @note   L980应用数据解析
  * @retval None
  ****************************************************************************/
 void L980_appRegDataParaphrase(L980_can_app_package *pPkt,unsigned char functionCode)
 {   
    if(functionCode == L980_REG_WRITE_MASK)
    {       
      L980_appWriteAck(pPkt->laser980Reg,pPkt->data);       
    }  
    else 
    {     
      L980_appReadAck(pPkt->laser980Reg,pPkt->data);
    }
 }
/************************************************************************//**
  * @brief  CAN_receivePackageHandle
  * @param  buff:数据缓存
  * @note   标准帧，固定长度8bytes
  * @retval None
  ****************************************************************************/
 static unsigned char canAppDataBuff[72];
 static L980_can_app_package pL980;
 void CAN_receivePackageHandle(unsigned char *data,unsigned char packageType)
 {  
    unsigned short int crcValue;  
    unsigned char len;
    unsigned char functionCode;
    if(packageType==RTU_CODE_SINGLE_PACKAGE) 
    { 
      len = data[1]+2;
      crcValue=data[len]<<8|data[len+1]; 
      if(crcValue!=CAN_crc16Num(data,len)) return; 
      functionCode=data[0]&L980_REG_WRITE_MASK;      
      pL980.laser980Reg = data[0]&L980_REG_MASK;
      pL980.packLen = data[1];   
      pL980.data = &data[2];     
      pL980.crcH = data[pL980.packLen+2];  
      pL980.crcL = data[pL980.packLen+3]; 
      L980_appRegDataParaphrase(&pL980,functionCode);
    } 
    else  
    {
      can_long_package *pLpack = (can_long_package*)data;
      if(pLpack->packageNum>(pLpack->currentNum))
      {       
        memcpy(&canAppDataBuff[pLpack->currentNum*6],pLpack->data,6); 
        if(pLpack->packageNum==(pLpack->currentNum+1))
        {//重组结束   
          functionCode=canAppDataBuff[0]&L980_REG_WRITE_MASK;
          if(functionCode==REG_AUX_REG)
          {         
            //CAN_LongPackageHandle(canAppDataBuff);
          }
          else
          {
            len = data[1]+2;
            crcValue=data[len]<<8|data[len+1]; 
            if(crcValue!=CAN_crc16Num(data,len)) return;              
            pL980.laser980Reg=canAppDataBuff[0]&L980_REG_MASK;
            pL980.packLen=canAppDataBuff[1];   
            pL980.data=&canAppDataBuff[2];     
            pL980.crcH= canAppDataBuff[pL980.packLen+2];  
            pL980.crcL= canAppDataBuff[pL980.packLen+3];  
            L980_appRegDataParaphrase(&pL980,functionCode);          
          } 
        }
      }
    }
 }
#if 0 
// 主设备
/************************************************************************//**
  * @brief  L980_appReadReq
  * @param  reg:起始地址 
  *         len:读取长度
  * @note   读请求
  * @retval None
  ****************************************************************************/
 void L980_appReadReq(unsigned char reg,unsigned char len)
 {  
    unsigned char sendBuff[8];  
    sendBuff[0]=reg; 
    sendBuff[1]=1; 
    sendBuff[2]=len;
    sendBuff[3]=(CAN_crc16Num(sendBuff,3)>>8)&0xFF;
    sendBuff[4]=CAN_crc16Num(sendBuff,3)&0xFF;  
    sendBuff[5]=0; 
    sendBuff[6]=0;
    sendBuff[7]=0;    
    CAN_RTU_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,sendBuff);
 }
 /************************************************************************//**
  * @brief  L980_appWriteReg
  * @param    
  * @note   写数据
  * @retval None
  ****************************************************************************/
 void L980_appWriteReg(unsigned char reg,unsigned char len,unsigned char *data)
 {  
    unsigned char sendBuff[128],packNum;    
    if(len>128)  return ;   
    sendBuff[0]=reg|L980_REG_WRITE_MASK; 
    sendBuff[1]=len;
    memcpy(&sendBuff[2],data,len);
    sendBuff[len+3]=(CAN_crc16Num(sendBuff,len+2)>>8)&0xFF;
    sendBuff[len+4]=CAN_crc16Num(sendBuff,len+2)&0xFF;
    if(len>4)
    {
      packNum=(len+4)>>3;///8;
      if((len+4)%8!=0)
      {
        packNum+=1;      
      }
      CAN_RTU_transmitPackage(RTU_CODE_LONG_BYTES_PACKAGE,packNum,sendBuff);
    }
    else
    { 
      packNum=1;
      CAN_RTU_transmitPackage(RTU_CODE_SINGLE_PACKAGE,packNum,sendBuff);
    }
 }
  #endif
