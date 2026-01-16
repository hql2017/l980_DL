/*
 * 
 *  
 *  Created on: Dec 24, 2025
 *      Author: Hql2017
 */

#ifndef CAN_MODBUSRTU__H_
#define  CAN_MODBUSRTU__H_

typedef enum {
	L980_NONE_CMD=0,
	L980_CODE_STATUS_QUERY, //1状态查询
	L980_CODE_ENERGE_PARAM, //2
	L980_CODE_CTR_TEST_MODE,//3光纤激活
	L980_CODE_PRO_HOT,//4
	L980_CODE_PULSE_COUNT_AND_TIME,//5	
	L980_CODE_PHOTODIOD,//6光电二极管
	L980_CODE_ENERGE_CALIBRATION,//7
	L980_CODE_MOTOR_POSITION,//8,激光切换
	L980_CODE_COOL_TEMPRATURE,//9冷却系统温度	
	L980_CODE_LASER_RUN_TIMER//10,倒计时	
}L980_cmd;

/*  data package  code+datalen+data+crc //标准数据帧
    crclen=  code+data;
 *  datalen = code+data;0
 
 */
typedef struct {
    unsigned char head;
    unsigned char datalen;
    unsigned char *data;
    unsigned char end;		
}L980_can_app_package;
void CAN_modbusRTU_init(void);
#endif /*  CAN_MODBUSRTU__H_ */


