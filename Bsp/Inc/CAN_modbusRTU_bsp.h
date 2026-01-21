
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
	RTU_CODE_SINGLE_PACKAGE=0,//单包
	RTU_CODE_LONG_BYTES_PACKAGE  //分包发送
}rtu_function_code;
#define L980_CODE_MASK  0x80
#define L980_REG_WRITE_MASK  0x80// write:bit7=1 ;read:bit7=0。
typedef enum {
	L980_NO_OPTION=0,
	L980_CTR_CMD,
	L980_REG_CTR_TEST_MODE,			//w光纤激活
	L980_REG_CTR_PRO_HOT,			//w准备
	L980_REG_JT_CTR_STOP,				//w启停	
	L980_REG_HEART_STATUS,  		//r只读
	L980_REG_COUNTDOWN_TIMERS,      //r/w倒计时	
	L980_REG_ENERGE_PARAM, 			//r/w	
	L980_REG_PULSE_COUNT_AND_TIME,	//r/w激光运行时间（S） data2:timeS_byte3 data3:timeS_byte2  data3:timeS_byte1 data4:timeS_byte0
	L980_REG_AUXILIARY_BULB,    	//r/w指示光 data1 :duty  data2: max duty data3:freq_H data4:freq_L
	L980_REG_PHOTODIOD,				//r/w光电二极管	
	L980_REG_ENERGE_CALIBRATION,	//r/w data1 :cali_p_H  data2:cali_p_L
	L980_REG_MOTOR_POSITION,		//r/W偏光片位置。data0 :um_H data1 : um_L 	
	L980_REG_LASER_TEMPRATURE,		//r/w激光器温度(10倍241:24.1℃) data1:set_T_H data2:set_T_L data3:real_T_H data4:real_T_L
	REG_AUX_REG,                    //r/w其他数据
}L980_cmd;
/** 
 * CAN数据帧解析
 * functionCode=RTU_CODE_LONG_BYTES_PACKAGE
 * pPkt：point package struct
 * 大端模式处理发送数据
 * ************/
typedef struct { 
    unsigned char laser980Reg;
	unsigned char packLen;//数据部分长度
    unsigned char *data;//bytes
	unsigned char crcH;//crcH
    unsigned char crcL;	//crcL	
}__attribute__((packed)) L980_can_app_package;

typedef struct {   
    unsigned char packageNum;   //总包数
    unsigned char *data;        //6bytes	
    unsigned char currentNum;	//当前包序号	
}__attribute__((packed)) can_long_package;
void CAN_modbusRTU_init(void);
void CAN_receivePackageHandle(unsigned char *data,unsigned char packageType);
void CAN_transmitPackage(unsigned char function,unsigned char code,unsigned char *data);
#endif /*  CAN_MODBUSRTU__H_ */


