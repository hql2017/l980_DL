
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
  * @brief  CAN_transmitPackage
  * @param   packNum:总包数
  * @note    
  * @retval None
  ****************************************************************************/
 void CAN_transmitPackage(unsigned char function,unsigned char packNum,unsigned char *data)
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
      APP_CAN_SEND_DATA(transmitBuff,8,CAN_MASTER_ID+RTU_CODE_LONG_BYTES_PACKAGE);
      HAL_Delay(1);
    }    
  }
  else 
  {       
    APP_CAN_SEND_DATA(data,8,CAN_MASTER_ID);
  }  
   
 } 
 /************************************************************************//**
  * @brief  CAN_pack_w_ack
  * @param   reg： 指令位置 flag:1成功 ；失败
  * @note    写回应：1成功 ；0失败
  * @retval None
  ****************************************************************************/
void CAN_pack_w_ack(unsigned char reg,unsigned char flag)
{
  unsigned char transmitBuff[8];
  transmitBuff[0] = reg; 
  transmitBuff[1] = 1; 
  transmitBuff[2] = flag; 
  transmitBuff[3] = (CAN_crc16Num(transmitBuff,3)>>8)&0xFF;
  transmitBuff[7] = CAN_crc16Num(transmitBuff,3)&0xFF;
  transmitBuff[4] =0;
  transmitBuff[5] = 0;            
  transmitBuff[6] = 0;
  transmitBuff[7] = 0;  
  CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
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
 void CAN_LongPackageHandle( unsigned char *data)
 {    
    unsigned char len=data[1];
    CAN_appLongPackageDataParaphrase(data,len);
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
      case L980_NO_OPTION:       
        break;
      case L980_CTR_CMD:       
        break;
      case L980_REG_HEART_STATUS:
          {            
            transmitBuff[0] = reg; 
            transmitBuff[1] = readBytes; 
            transmitBuff[2] = (laser_980_sta.sys_run_status_value>>24)&0xFF; 
            transmitBuff[3] = (laser_980_sta.sys_run_status_value>>16)&0xFF; 
            transmitBuff[4] = (laser_980_sta.sys_run_status_value>>8)&0xFF;
            transmitBuff[5] = (laser_980_sta.sys_run_status_value)&0xFF;            
            transmitBuff[6] = (CAN_crc16Num(transmitBuff,6)>>8)&0xFF;
            transmitBuff[7] = CAN_crc16Num(transmitBuff,6)&0xFF;  
            CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
          }
        break;
      case L980_REG_ENERGE_PARAM:
          {            
            transmitBuff[0] = reg; 
            transmitBuff[1] = readBytes; 
            transmitBuff[2] = (laser_ctr_param.energe>>8)&0xFF; 
            transmitBuff[3] = (laser_ctr_param.energe)&0xFF; 
            transmitBuff[4] = (laser_980_sta.laser_real_energe>>8)&0xFF;
            transmitBuff[5] = (laser_980_sta.laser_real_energe)&0xFF;            
            transmitBuff[6] = (CAN_crc16Num(transmitBuff,6)>>8)&0xFF;
            transmitBuff[7] = CAN_crc16Num(transmitBuff,6)&0xFF;  
            CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
          }
        break;      
      case L980_REG_CTR_TEST_MODE:
        {            
          transmitBuff[0] = reg; 
          transmitBuff[1] = readBytes;              
          transmitBuff[2] = laser_980_sta.laser_test_ctr_status;                       
          transmitBuff[3] = (CAN_crc16Num(transmitBuff,3)>>8)&0xFF;
          transmitBuff[4] = CAN_crc16Num(transmitBuff,3)&0xFF;  
          transmitBuff[5]=0;
          transmitBuff[6]=0;
          transmitBuff[7]=0;
          CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
        }
        break;
      case L980_REG_CTR_PRO_HOT:
          {            
            transmitBuff[0] = reg; 
            transmitBuff[1] = readBytes; 
            transmitBuff[2] = laser_ctr_param.pro_hot;        
            transmitBuff[3] = laser_980_sta.laser_prohot_status;                       
            transmitBuff[4] = (CAN_crc16Num(transmitBuff,4)>>8)&0xFF;
            transmitBuff[5] = CAN_crc16Num(transmitBuff,4)&0xFF;  
            transmitBuff[6]=0;
            transmitBuff[7]=0;
            CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
          }
        break;    
      case L980_REG_JT_CTR_STOP:
        {            
          transmitBuff[0] = reg; 
          transmitBuff[1] = readBytes;           
          transmitBuff[2] = laser_980_sta.laser_980_out_status;                       
          transmitBuff[3] = (CAN_crc16Num(transmitBuff,3)>>8)&0xFF;
          transmitBuff[4] = CAN_crc16Num(transmitBuff,3)&0xFF;  
          transmitBuff[5]=0;
          transmitBuff[6]=0;
          transmitBuff[7]=0;
          CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
        }
        break;
      case L980_REG_PULSE_COUNT_AND_TIME:
          {            
            transmitBuff[0] = reg; 
            transmitBuff[1] = readBytes; 
            transmitBuff[2] = (u_sys_param.sys_config_param.laser_use_timeS>>24)&0xFF; 
            transmitBuff[3] = (u_sys_param.sys_config_param.laser_use_timeS>>16)&0xFF; 
            transmitBuff[4] = (u_sys_param.sys_config_param.laser_use_timeS>>8)&0xFF;
            transmitBuff[5] = (u_sys_param.sys_config_param.laser_use_timeS)&0xFF;            
            transmitBuff[6] = (CAN_crc16Num(transmitBuff,6)>>8)&0xFF;
            transmitBuff[7] = CAN_crc16Num(transmitBuff,6)&0xFF;  
            CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
          }
        break;
      case L980_REG_PHOTODIOD:
          {            
            transmitBuff[0] = reg; 
            transmitBuff[1] = readBytes; 
            transmitBuff[2] = (laser_980_sta.real_photodiod_value>>8)&0xFF;
            transmitBuff[3] = (laser_980_sta.real_photodiod_value)&0xFF;
            transmitBuff[4] = (CAN_crc16Num(transmitBuff,4)>>8)&0xFF;
            transmitBuff[5] = CAN_crc16Num(transmitBuff,4)&0xFF;       
            transmitBuff[6]=0;
            transmitBuff[7]=0;
            CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
          }
        break;
      case L980_REG_AUXILIARY_BULB:
          {            
            transmitBuff[0] = reg; 
            transmitBuff[1] = readBytes; 
            transmitBuff[2] = laser_980_sta.auxiliary_bulb_staus;
            transmitBuff[3] = (CAN_crc16Num(transmitBuff,3)>>8)&0xFF;
            transmitBuff[4] = CAN_crc16Num(transmitBuff,3)&0xFF;  
            transmitBuff[5]=0;
            transmitBuff[6]=0;
            transmitBuff[7]=0;
            CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
          }
        break;
      case L980_REG_ENERGE_CALIBRATION:
          {            
            transmitBuff[0] = reg; 
            transmitBuff[1] = readBytes; 
            transmitBuff[2] = (u_sys_param.sys_config_param.e_cali[data[2]]>>8)&0xFF; 
            transmitBuff[3] = (u_sys_param.sys_config_param.e_cali[data[2]])&0xFF; 
            transmitBuff[4] = (CAN_crc16Num(transmitBuff,4)>>8)&0xFF;
            transmitBuff[5] = CAN_crc16Num(transmitBuff,4)&0xFF; 
            transmitBuff[6] = readBytes; 
            transmitBuff[7] = 0; 
            CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
          }
        break;
      case L980_REG_MOTOR_POSITION:
          {            
            transmitBuff[0] = reg; 
            transmitBuff[1] = readBytes; 
            transmitBuff[2] = (u_sys_param.sys_config_param.motor_positon_um_set>>8)&0xFF; 
            transmitBuff[3] = (u_sys_param.sys_config_param.motor_positon_um_set>>8)&0xFF;  
            transmitBuff[4] = (laser_980_sta.real_motor_positon_um>>8)&0xFF;
            transmitBuff[5] = (laser_980_sta.real_motor_positon_um)&0xFF;            
            transmitBuff[6] = (CAN_crc16Num(transmitBuff,6)>>8)&0xFF;
            transmitBuff[7] = CAN_crc16Num(transmitBuff,6)&0xFF;  
            CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
          }
        break;
      case L980_REG_LASER_TEMPRATURE:
          {            
            transmitBuff[0] = reg; 
            transmitBuff[1] = readBytes; 
            transmitBuff[2] = (u_sys_param.sys_config_param.cool_temprature_target>>8)&0xFF; //set  T
            transmitBuff[3] = (u_sys_param.sys_config_param.cool_temprature_target>>8)&0xFF;  
            transmitBuff[4] = (laser_980_sta.real_laser_temprature>>8)&0xFF;//real T
            transmitBuff[5] = (laser_980_sta.real_laser_temprature)&0xFF;            
            transmitBuff[6] = (CAN_crc16Num(transmitBuff,6)>>8)&0xFF;
            transmitBuff[7] = CAN_crc16Num(transmitBuff,6)&0xFF;  
            CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
          }
        break;
      case L980_REG_COUNTDOWN_TIMERS:
          {            
            transmitBuff[0] = reg; 
            transmitBuff[1] = readBytes; 
            transmitBuff[2] = (u_sys_param.sys_config_param.count_timer_s_set>>8)&0xFF; //set timers
            transmitBuff[3] = (u_sys_param.sys_config_param.count_timer_s_set>>8)&0xFF;  
            transmitBuff[4] = (laser_980_sta.real_timers>>8)&0xFF;//real timers
            transmitBuff[5] = (laser_980_sta.real_timers)&0xFF;            
            transmitBuff[6] = (CAN_crc16Num(transmitBuff,6)>>8)&0xFF;
            transmitBuff[7] = CAN_crc16Num(transmitBuff,6)&0xFF;  
            CAN_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,transmitBuff);
          }
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
    case L980_NO_OPTION:       
      break;
    case L980_CTR_CMD:       
      break;
    case L980_REG_HEART_STATUS:
        {  
          CAN_pack_w_ack( reg,1);
        }
      break;
    case L980_REG_ENERGE_PARAM:
        {            
          CAN_pack_w_ack( reg,1);
        }
      break;      
    case L980_REG_CTR_TEST_MODE:
      {            
        CAN_pack_w_ack( reg,1);
      }
      break;
    case L980_REG_CTR_PRO_HOT:
        {            
          CAN_pack_w_ack( reg,1);
        }
        
      break;    
    case L980_REG_JT_CTR_STOP:
        {   
          laser_ctr_param.laser_ctr_JT=data[0];    
          CAN_pack_w_ack( reg,1);
        }
      break;
    case L980_REG_PULSE_COUNT_AND_TIME:
        {    
          u_sys_param.sys_config_param.laser_use_timeS= (data[0]<<24)|(data[1]<<16)|(data[2]<<8)|data[3];           
          CAN_pack_w_ack( reg,1);
        }
      break;
    case L980_REG_PHOTODIOD:
        {      
          laser_980_sta.real_photodiod_value=(data[0]<<8)|data[1];   
          CAN_pack_w_ack( reg,1);
        }
      break;
    case L980_REG_AUXILIARY_BULB:
        {            
          laser_980_sta.auxiliary_bulb_staus=data[0];
          CAN_pack_w_ack( reg,1);         
        }
      break;
    case L980_REG_ENERGE_CALIBRATION:
        {  
          u_sys_param.sys_config_param.e_cali[data[0]] = (data[1]<<8)|data[2];        
          CAN_pack_w_ack( reg,1); 
        }
      break;
    case L980_REG_MOTOR_POSITION:
        {            
          u_sys_param.sys_config_param.motor_positon_um_set=(data[0]<<8)|data[1];
          CAN_pack_w_ack( reg,1); 
        }
      break;
    case L980_REG_LASER_TEMPRATURE:
        {            
          u_sys_param.sys_config_param.cool_temprature_target=(data[0]<<8)|data[1];
          CAN_pack_w_ack( reg,1); 
        }
      break;
    case L980_REG_COUNTDOWN_TIMERS:
        {            
          u_sys_param.sys_config_param.count_timer_s_set=(data[0]<<8)|data[1];
          if(data[2]!=0)//开启
          {
            laser_980_sta.real_timers=u_sys_param.sys_config_param.count_timer_s_set;
          }
          else laser_980_sta.real_timers=0;//停止
          CAN_pack_w_ack( reg,1);           
        }
      break;
    default:
      break;
  }


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
  * @brief  L980_appRegDataParaphrase
  * @param    
  * @note   L980应用数据解析
  * @retval None
  ****************************************************************************/
 void L980_appRegDataParaphrase(L980_can_app_package *pPkt,unsigned char functionCode)
 {   
    if(functionCode == L980_REG_WRITE_MASK)
    {   
      DEBUG_PRINTF(" CAN_R reg\r\n ");
      L980_appReadAck(pPkt->laser980Reg,pPkt->data);
    }  
    else 
    {
      DEBUG_PRINTF(" CAN_w reg\r\n ");
      L980_appWriteAck(pPkt->laser980Reg,pPkt->data);
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
      pL980.laser980Reg = data[0]&0x7F;
      pL980.packLen = data[1];   
      pL980.data = &data[2];     
      pL980.crcH = data[pL980.packLen+2];  
      pL980.crcL = data[pL980.packLen+3]; 
      L980_appRegDataParaphrase(&pL980,L980_REG_WRITE_MASK);
    } 
    else  
    {
      can_long_package *pLpack = (can_long_package*)data;
      if(pLpack->packageNum>(pLpack->currentNum))
      {       
        memcpy(&canAppDataBuff[pLpack->currentNum*6],pLpack->data,6); 
        if( pLpack->packageNum==(pLpack->currentNum+1))
        {//重组结束   
          functionCode=canAppDataBuff[0]&L980_REG_WRITE_MASK;
          if(functionCode==REG_AUX_REG)
          {         
            CAN_LongPackageHandle(canAppDataBuff);
          }
          else
          {
            len = data[1]+2;
            crcValue=data[len]<<8|data[len+1]; 
            if(crcValue!=CAN_crc16Num(data,len)) return;              
            pL980.laser980Reg=canAppDataBuff[0]&0x7F;
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


