
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 
#include "stdlib.h"

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
    sendBuff[0]=reg; 
    sendBuff[1]=dataLen;
    memcpy(&sendBuff[2],data,dataLen);
    sendBuff[dataLen+3]=(CAN_crc16Num(sendBuff,dataLen+2)>>8)&0xFF;
    sendBuff[dataLen+4]=CAN_crc16Num(sendBuff,dataLen+2)&0xFF;
    if(dataLen>4)
    {
      packNum=(len)/6;
      unsigned char t_len=(len)%6;
      if(t_len!=0)
      {
        packNum+=1; 
        memset(&sendBuff[packNum*6-6+t_len] ,0 ,6-t_len );//剩余补0
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
      case L980_REG_ENERGE_CALI_PARAM:
        {  
          unsigned short int num=(u_s_l980.sta.useEnerge/5);
          num%=41;
          transmitBuff[0]=u_s_l980.sta.useEnerge&0xFF;
          transmitBuff[1]=(u_s_l980.sta.useEnerge>>8)&0xFF;
          transmitBuff[3]=u_sys_param.sys_config_param.e_cali[num]&0xFF;
          transmitBuff[4]=(u_sys_param.sys_config_param.e_cali[num]>>8)&0xFF;
          CAN_r_w_ack(reg,4,transmitBuff);
        }
      break;    
      case L980_REG_AUXILIARY_BULB:           
        transmitBuff[0]=u_l980.set_param.auxLedBulbDutySet;
        transmitBuff[1]=u_l980.set_param.auxLedBulbFreqSet;         
        CAN_r_w_ack(reg,2,transmitBuff);
        break;   
      case L980_REG_MOTOR_POSITION: 
            transmitBuff[0]= u_l980.set_param.positionSet&0xFF;
            transmitBuff[1]=(u_l980.set_param.positionSet>>8)&0xFF; 
            transmitBuff[2]=u_s_l980.sta.realPosition&0xFF;//电机实时位置
            transmitBuff[3]=(u_s_l980.sta.realPosition>>8)&0xFF;            
            CAN_r_w_ack(reg,4,transmitBuff);      
        break;
      case L980_REG_LASER_TEMPRATURE: 
            CAN_r_w_ack(reg,2,(unsigned char *)&u_l980.set_param.targetTempratureSet);
        break;
      case L980_REG_COUNTDOWN_TIMERS:             
            transmitBuff[0]= u_l980.set_param.timerSet&0xFF;
            transmitBuff[1]=( u_l980.set_param.timerSet>>8)&0xFF;
            CAN_r_w_ack(reg,2,transmitBuff);
        break;
      case L980_REG_SYNC_CONFIG:     
          CAN_r_w_ack(reg,sizeof(L980_SET_PARAM),u_l980.data);         
        break; 
        case L980_REG_TEC_CTR:          
         // transmitBuff[0]= u_s_l980.sta.tec_switch&0xFF;
         // transmitBuff[1]=( u_s_l980.sta.tec_switch>>8)&0xFF;
          CAN_r_w_ack(reg|L980_REG_WRITE_MASK,2, (unsigned char *)&u_s_l980.sta.tec_switch);
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
    case L980_REG_ENERGE_CALI_PARAM:  
        {  
          unsigned short int num=(u_s_l980.sta.useEnerge/5);
          num%=41;
          u_s_l980.sta.useEnerge=(data[1]<<8)|data[0];
          u_sys_param.sys_config_param.e_cali[num]=(data[3]<<8)|data[2];              
          DEBUG_PRINTF("energe Set OK=%dmj cli=%d\r\n",u_s_l980.sta.useEnerge,u_sys_param.sys_config_param.e_cali[num]);         
          CAN_r_w_ack(reg|L980_REG_WRITE_MASK,4,data);
        }
      break;  
    case L980_REG_CTR_PRO_HOT: 
        u_s_l980.sta.useEnerge=(data[1]<<8)|data[0];
        u_s_l980.sta.dacValue=(data[3]<<8)|data[2];
        if(u_s_l980.sta.dacValue>0)
        {
          laser_ctr_param.pro_hot=1;
          DEBUG_PRINTF("980 prohot energe=%d DAC=%d\r\n", u_s_l980.sta.useEnerge,u_s_l980.sta.dacValue); 
        }
        else
        {
          laser_ctr_param.pro_hot=0;
          DEBUG_PRINTF("980 exit prohot finish ,wait status fresh\r\n");
        }
        app_laser_prohot_semo();    
        CAN_r_w_ack(reg|L980_REG_WRITE_MASK,4,data);
      break;    
    case L980_REG_JT_CTR_STOP:
          if(data[0]!=0&&((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)==L980_STA_PROHOT_BIT1))
          {     
            laser_ctr_param.JT_laser_out=1;
            transmitBuff[0]=1;
            transmitBuff[1]=0;
            DEBUG_PRINTF("980 pulse out \r\n"); 
          }  
          else 
          {
            laser_ctr_param.JT_laser_out=0;
            transmitBuff[0]=0;
            transmitBuff[0]=0;
            DEBUG_PRINTF("980 pusle stop\r\n");  
          }
          CAN_r_w_ack(reg|L980_REG_WRITE_MASK,2,transmitBuff);          
      break;
    case L980_REG_PULSE_COUNT_AND_TIME:
           u_sys_param.sys_config_param.laser_use_timeS=(data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0];
          DEBUG_PRINTF("980 laser use time  change OK\r\n");
          u_s_l980.sta.laserUseTimeS=u_sys_param.sys_config_param.laser_use_timeS;        
          CAN_r_w_ack(reg|L980_REG_WRITE_MASK,4,(unsigned char *)&u_s_l980.sta.laserUseTimeS);
      break;    
    case L980_REG_AUXILIARY_BULB:       
        u_l980.set_param.auxLedBulbDutySet=data[0];
        u_l980.set_param.auxLedBulbFreqSet=data[1];        
        DEBUG_PRINTF("980 set led ok duty=%d f=%d\r\n", u_l980.set_param.auxLedBulbDutySet,u_l980.set_param.auxLedBulbFreqSet); 
        CAN_r_w_ack(reg|L980_REG_WRITE_MASK,2,data);
      break;   
    case L980_REG_MOTOR_POSITION:
        {
          unsigned short int setTemp=(data[1]<<8)|data[0];
          laser_ctr_param.motor_active=(data[3]<<8)|data[2];
          if(laser_ctr_param.motor_active>L980_MAX_MOTOR_DISTANCE_UM)
          {
            laser_ctr_param.motor_active=L980_MAX_MOTOR_DISTANCE_UM;         
          }
          if(setTemp!=0)
          {
            if(setTemp>L980_MAX_MOTOR_DISTANCE_UM)  u_l980.set_param.positionSet=L980_MAX_MOTOR_DISTANCE_UM;
            else u_l980.set_param.positionSet=setTemp;
          }
          app_motor_move_to_sem(laser_ctr_param.motor_active);
          CAN_r_w_ack(reg|L980_REG_WRITE_MASK,4,data); 
        }            
      break;
    case L980_REG_LASER_TEMPRATURE:
          u_l980.set_param.targetTempratureSet=(data[1]<<8)|data[0];//
          DEBUG_PRINTF("980 set work emprature OK=%.1f℃r\n",u_l980.set_param.targetTempratureSet*0.1); 
          CAN_r_w_ack(reg,2, (unsigned char *)&u_l980.set_param.targetTempratureSet);    
      break;
    case L980_REG_COUNTDOWN_TIMERS:
          u_l980.set_param.timerSet=(data[3]<<8)|data[2];          
          u_sys_param.sys_config_param.laTimerSet= abs(u_l980.set_param.timerSet);          
          DEBUG_PRINTF("980 counter timer set OK= %ds\r\n",u_l980.set_param.timerSet);
          CAN_r_w_ack(reg|L980_REG_WRITE_MASK,2, data);       
      break;
      case  L980_REG_SYNC_CONFIG: 
          memcpy(u_l980.data,data,sizeof(L980_SET_PARAM)); 
          transmitBuff[0]=1;//ok
          transmitBuff[1]=0;
          CAN_r_w_ack(reg|L980_REG_WRITE_MASK,2, transmitBuff); 
          DEBUG_PRINTF("980 config param set OK\r\n");
      break;
      case L980_REG_ENGINEER_MODE:
          if(data[0]!=0)
          {
            u_s_l980.sta.reserveByte=1;
            DEBUG_PRINTF("980 in engineer mode! please call manufacturer\r\n");
          }
         else 
         {
          u_s_l980.sta.reserveByte=0;
          DEBUG_PRINTF("980 exit engineer mode!\r\n");
         }
         CAN_r_w_ack(reg|L980_REG_WRITE_MASK,2, data); 
      break;
    case L980_REG_TEC_CTR:
          u_s_l980.sta.tec_switch=(data[3]<<8)|data[2];
          if(u_s_l980.sta.tec_switch!=0)
          {
           
            DEBUG_PRINTF("980 tec enable\r\n");
          }
          else 
          {
            
            DEBUG_PRINTF("980 tec disable\r\n");
          }
        CAN_r_w_ack(reg|L980_REG_WRITE_MASK,2, data);
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
  * @param  data:数据缓存；packageType 长包、短包
  * @note   标准帧，固定长度8bytes
  * @retval None
  ****************************************************************************/

 void CAN_receivePackageHandle(unsigned char *data,unsigned char packageType)
 {  
    static unsigned char canAppDataBuff[128];
    static L980_can_app_package pL980;
    unsigned short int crcValue;  
    unsigned char len;
    unsigned char functionCode;
    if(packageType==RTU_CODE_SINGLE_PACKAGE) 
    { 
      len = data[1]+4;
      crcValue=(data[len-2]<<8)|data[len-1];  
      if(crcValue!=CAN_crc16Num(data,len-2)) return;     
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
      if(data[0]>16)   return;//too long
      if((data[0])>data[7])
      {       
        memcpy(&canAppDataBuff[data[7]*6],&data[1],6);         
        if(data[0]==(data[7]+1))
        {//重组结束 
          len = canAppDataBuff[1]+4;
          if(len>128) return ;
          crcValue=(canAppDataBuff[len-2]<<8)|canAppDataBuff[len-1];
          if(crcValue!=CAN_crc16Num(canAppDataBuff,len-2)) return;        
          functionCode=canAppDataBuff[0]&L980_REG_WRITE_MASK;
          pL980.laser980Reg=canAppDataBuff[0]&L980_REG_MASK;
          if(pL980.laser980Reg==REG_AUX_REG)
          {         
            //CAN_LongPackageHandle(canAppDataBuff);
          }
          else
          {   
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
