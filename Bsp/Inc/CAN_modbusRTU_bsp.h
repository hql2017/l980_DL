
/*
 * 
 *  
 *  Created on: Dec 24, 2025
 *      Author: Hql2017
 */
#ifndef CAN_MODBUSRTU__H_
#define  CAN_MODBUSRTU__H_


/**
 * R:read 
 * W:write 
 * 非标准modbusRTU功能码
 * ************/
typedef enum {
	RTU_CODE_NONE=0,	
	RTU_CODE_R_SINGLE_REG,		
	RTU_CODE_R_MULTIPLE_REG,	
	RTU_CODE_W_SINGLE_REG, 	
	RTU_CODE_W_MULTIPLE_REG,
	RTU_CODE_LONG_BYTES_PACKAGE //超长数据包
}rtu_function_code;
#define L980_CODE_MASK  0x80
typedef enum {
	L980_NONE_CMD=0,
	L980_REG_HEART_STATUS,  		//心跳 状态
	L980_REG_ENERGE_PARAM, 			//		     
	L980_REG_CTR_TEST_MODE,			//光纤激活
	L980_REG_CTR_PRO_HOT,			// 
	L980_REG_CTR_JT_PRESS,			// 
	L980_REG_CTR_STOP,				//
	L980_REG_PULSE_COUNT_AND_TIME,	//激光运行时间（S） data2:timeS_byte3 data3:timeS_byte2  data3:timeS_byte1 data4:timeS_byte0
	L980_REG_PHOTODIOD,				//光电二极管
	L980_REG_AUXILIARY_BULB,    	//指示光 data1 :duty  data2: max duty data3:freq_H data4:freq_L
	L980_REG_ENERGE_CALIBRATION,	//data1 :cali_p_H  data2:cali_p_L
	L980_REG_MOTOR_POSITION,		//激光切换
	L980_REG_COOL_TEMPRATURE,		//冷却系统温度(10倍241:24.1℃) data1:set_T_H data2:set_T_L data3:real_T_H data4:real_T_L
	L980_REG_LASER_COUNTDOWN_TIMER,//倒计时（S）data1:on/off data2:timeS_byte1 data3:timeS_byte0 		
	L980_REG_POSITION              //偏光镜，位置。data1 :um_H data2 : um_L 	
}L980_cmd;
/** 
 * CAN数据帧解析
 * functionCode=RTU_CODE_LONG_BYTES_PACKAGE
 * pPkt：point package struct
 * ************/
typedef struct {    
	unsigned char functionCode;//functionCode
    unsigned char laser980Reg;
    unsigned char *data;//4bytes
	unsigned char crcH;//crcH
    unsigned char crcL;	//crcL	
}__attribute__((packed)) L980_can_app_package;

typedef struct {    
	unsigned char functionCode;//functionCode
    unsigned char packageNum;   //总包数
    unsigned char *data;        //5bytes	
    unsigned char currentNum;	//当前包序号	
}__attribute__((packed)) can_long_package;
void CAN_modbusRTU_init(void);
void CAN_receivePackageHandle(unsigned char *buff,unsigned char Len);
void CAN_transmitPackage(unsigned char function,unsigned char code,unsigned char *data);
#endif /*  CAN_MODBUSRTU__H_ */


