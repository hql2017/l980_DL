
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 

#include "CAN_modbusRTU_bsp.h"
#include "fdcan.h"
#include "tim.h"
#include "main.h"


L980_STATUS l980_sta;
L980_CONFIG_PARAM u_l980;
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
    unsigned char transmitBuff[8],readBytes;
    readBytes=data[0];
    switch(reg)
    {   
      case L980_REG_HEART_STATUS:
          {   
            l980_sta.realtemprature=(data[1]<<8)|data[0];    
            l980_sta.staByte=(data[3]<<8)|data[2]; 
            l980_sta.realPosition=(data[5]<<8)|data[4]; 
            l980_sta.energeFeedback=(data[7]<<8)|data[6];
            l980_sta.laserUseTimeS=(data[11]<<24)|(data[10]<<16)|(data[9]<<8)|data[8];         
          }
        break;
      case L980_REG_PULSE_COUNT_AND_TIME:
            {
              l980_sta.laserUseTimeS=(data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0];
            }
        break;
        case L980_REG_ENERGE_PARAM:
          {
            u_l980.set_param.energeSet=(data[1]<<8)|data[0];   
            u_l980.set_param.energeCaliSet=(data[3]<<8)|data[2];            
          }
      break;    
      case L980_REG_AUXILIARY_BULB:  
            u_l980.set_param.auxLedBulbDutySet=data[0];
            u_l980.set_param.auxLedBulbFreqSet=data[1];
        break;   
      case L980_REG_MOTOR_POSITION:  
            u_l980.set_param.positionSet=(data[1]<<8)|data[0];
            unsigned short int temp;
            temp=(data[3]<<8)|data[2];
            if(temp==0)
            {
              DEBUG_PRINTF("980 motor not move!\r\n");  //          
            }        
            else if(temp>L980_MAX_MOTOR_DISTANCE_UM)
            {
              DEBUG_PRINTF("980 motor is moving find zero\r\n");        
            }
            else
            {
              DEBUG_PRINTF("980 motor move to =%dμm\r\n",temp); 
            }                          
        break;
      case L980_REG_LASER_TEMPRATURE:
            u_l980.set_param.targetTempratureSet=data[0];
        break;
      case L980_REG_COUNTDOWN_TIMERS:
            if(data[0]!=0) 
            {
              l980_sta.staByte|=L980_STA_TIMERS_BIT3;
            }
            else 
            {//~L980_STA_TIMERS_BIT3=0xF7
              l980_sta.staByte&=0xF7;
            }        
            u_l980.set_param.timerSet=(data[3]<<8)|data[2];
        break;
      case L980_REG_SYNC_CONFIG:  
          memcpy(u_l980.data,data,sizeof(L980_SET_PARAM));          
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
 extern void app_laser_prohot_semo(void);
 void L980_appWriteAck(unsigned char reg,unsigned char *data)
 {
  unsigned char transmitBuff[8];
  switch(reg)
  {
    case L980_REG_ENERGE_PARAM:  
          u_l980.set_param.energeSet=(data[1]<<8)|data[0];
          u_l980.set_param.energeCaliSet=(data[3]<<8)|data[2];         
          DEBUG_PRINTF("energe Set OK=%dmj cli=%d\r\n",u_l980.set_param.energeSet,u_l980.set_param.energeCaliSet);
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
      break;    
    case L980_REG_JT_CTR_STOP:
          if(data[0]==1)
          {          
            DEBUG_PRINTF("980 pulse out \r\n"); 
          }
          else   DEBUG_PRINTF("980 stop pulse out  \r\n");             
      break;
    case L980_REG_PULSE_COUNT_AND_TIME:
           if(data[0]==1)
          {
            DEBUG_PRINTF("980 laser use time  change OK\r\n");
            u_sys_param.sys_config_param.laser_use_timeS=l980_sta.laserUseTimeS;
          }
          else
          {
            l980_sta.laserUseTimeS= u_sys_param.sys_config_param.laser_use_timeS;
            DEBUG_PRINTF("980 laser use time  change fail\r\n");
          }     
      break;    
    case L980_REG_AUXILIARY_BULB:       
        u_l980.set_param.auxLedBulbDutySet=data[0];
        u_l980.set_param.auxLedBulbFreqSet=data[1];
        DEBUG_PRINTF("980 set led ok duty=%d f=%d\r\n", u_l980.set_param.auxLedBulbDutySet,u_l980.set_param.auxLedBulbFreqSet); 
      break;   
    case L980_REG_MOTOR_POSITION:
        u_l980.set_param.positionSet=(data[1]<<8)|data[0];//
        unsigned short int temp;
        temp=(data[3]<<8)|data[2];
        if(temp==0)
        {
          DEBUG_PRINTF("980 motor stop move!\r\n");  //立即停止           
        }        
        else if(temp>L980_MAX_MOTOR_DISTANCE_UM)
        {
          DEBUG_PRINTF("980 motor find zero\r\n");        
        }
        else
        {
          DEBUG_PRINTF("980 motor move to =%dμm\r\n",temp); 
        }              
      break;
    case L980_REG_LASER_TEMPRATURE:
          u_l980.set_param.targetTempratureSet=(data[1]<<8)|data[0];//
          DEBUG_PRINTF("980 set work emprature OK=%d℃r\n",u_l980.set_param.targetTempratureSet);     
      break;
    case L980_REG_COUNTDOWN_TIMERS:
          if(data[0]!=0) 
          {
            l980_sta.staByte|=L980_STA_TIMERS_BIT3;
          }
          else 
          {
            l980_sta.staByte&=(~(L980_STA_TIMERS_BIT3));
          } 
          u_l980.set_param.timerSet=(data[3]<<8)|data[2];          
          DEBUG_PRINTF("980 counter timer set OK= %ds\r\n",u_l980.set_param.timerSet);         
      break;
      case  L980_REG_SYNC_CONFIG:
          memcpy(u_l980.data,data,sizeof(L980_SET_PARAM)); 
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
      DEBUG_PRINTF(" CAN_RTU_w reg\r\n ");
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
    }
    else packNum=1;
    CAN_RTU_transmitPackage(RTU_CODE_SINGLE_PACKAGE,packNum,sendBuff);
 }
