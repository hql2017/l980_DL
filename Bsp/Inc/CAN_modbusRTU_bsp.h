
/*
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
#define L980_MAX_MOTOR_DISTANCE_UM    32000  //32mm,最远运动距离
#define L980_MAX_PROHOT_WAIT_TIME  20*SYS_1_SECOND_TICKS//20s
#define L980_CAN_FRAME_TIMEOUT  200
#define L980_CAN_MINI_TIME_MS  50//安全包间隔
#define L980_CODE_MASK  0x80
#define L980_REG_WRITE_MASK  0x80// write:bit7=1 ;read:bit7=0。
#define L980_REG_MASK   0x7F
typedef enum {		
	RTU_CODE_SINGLE_PACKAGE=0,//单包
	RTU_CODE_LONG_BYTES_PACKAGE  //分包发送
}rtu_function_code;

typedef enum {
	L980_NO_OPTION=0,
	L980_REG_HEART_STATUS,  		//r只读		
	L980_REG_CTR_PRO_HOT,			//w准备(4Bytes) (unsigned short 0)=ctr cmd; (unsigned short 1) energe
	L980_REG_JT_CTR_STOP,			//w启停	(2Bytes) (unsigned short 0)=ctr cmd;
	L980_REG_COUNTDOWN_TIMERS,      //r/w倒计时(4Bytes)	（unsigned short 0）=ctr cmd ;(unsigned short 1) timers(180S)
	L980_REG_ENERGE_PARAM, 			//r/w(4Bytes)	(unsigned short 0)energe,(unsigned short 1)energeCali(校准值) ;r:realValue ;w:setValue
	L980_REG_PULSE_COUNT_AND_TIME,	//r/w激光运行时间(4bytes）（unsigned int 0） r:realTime;w:setTime.
	L980_REG_AUXILIARY_BULB,    	//r/w指示光(2Bytes) byte0=duty;byte1=freq; r:realValue;w:setValue
	L980_REG_MOTOR_POSITION,		//r/W偏光片位置(4Bytes)(μm)(unsigned short 0)set position。(unsigned short 1) ctr cmd(0：stop ;其他位置:>L980_MAX_MOTOR_DISTANCE_UM找0)	
	L980_REG_LASER_TEMPRATURE,		//r/w激光器温度(2Bytes)(10倍241:24.1℃) (short int 0) r:realT;w:setT.
	L980_REG_MULTIPLE_READ,         //多通道读
	L980_REG_MULTIPLE_WRITE,        //多通道写
	L980_REG_SYNC_CONFIG,           //同步配置数据
	REG_AUX_REG,                    //r/w其他数据
}L980_cmd;

#define L980_STATUS_BYTE_MASK   0x0F//状态掩码
#define L980_STA_HEART_BIT0     0x01//1，ok(参数同步成功);0 参数同步失败;
#define L980_STA_PROHOT_BIT1    0x01<<1//1，ok;0 准备失败。
#define L980_STA_PULSEOUT_BIT2  0x01<<2//1，输出ok;
#define L980_STA_TIMERS_BIT3    0x01<<3 //1，使能;//倒计时

#define L980_ERROR_HALF_MASK   0xF0//错误掩码,状态字高4位
#define L980_ERR_TEMPRATURE_BIT4     0x01<<4//1，高温;0 正常
#define L980_ERR_ENERGE_BIT5         0x01<<5//1，激光能量过高
#define L980_ERR_LADER_POEER_BIT6    0x01<<6//1，激光电源异常
#define L980_ERR_MOTOR_BIT7          0x01<<7//1，丝杆移动异常
/** 
 * CAN数据帧解析
 * 读指令数据结构：
 * reg+(datalen=1)+(Nbytes 需要读取的数据长度)+crc
 * 读返回：reg+(datalen=N)+(Nbytes数据)+crc
 * 写指令数据结构：
 * reg+(datalen=N)+(Nbytes数据)+crc
 * pPkt：point package struct
 * 小端模式数据
 * “校验码高字节在前”
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

typedef struct {	 
	short int   realtemprature;   			//10倍温度值(-40~150) 
	unsigned short int    staByte;				//heart; bit1 准备就绪;bit2 脉冲输出;bit3 倒计时 1正在运行;
	unsigned short int    realPosition;         //实际位置			
	unsigned short int    energeFeedback;    	//能量计采集值
	unsigned  int    laserUseTimeS;   			//激光运行时间
}__attribute__((packed)) L980_STATUS;			//状态数据
extern L980_STATUS  l980_sta;

typedef struct {	 	
	short int   targetTempratureSet;    //温度值
	unsigned char   auxLedBulbDutySet;    //指示灯占空比
	unsigned char   auxLedBulbFreqSet;    //指示灯频率
	unsigned short int    positionSet;   //位置 
	unsigned short int    timerSet;   //倒计时
	unsigned short int    energeSet;   //能量值	
	unsigned short int    energeCaliSet;   //能量校准	
}__attribute__((packed)) L980_SET_PARAM;
typedef union 
{
	L980_SET_PARAM  set_param;
	unsigned char data[sizeof(L980_SET_PARAM)];
}L980_CONFIG_PARAM;
extern L980_CONFIG_PARAM u_l980;

void CAN_modbusRTU_init(void);
void CAN_receivePackageHandle(unsigned char *data,unsigned char packageType);
void CAN_RTU_transmitPackage(unsigned char function,unsigned char code,unsigned char *data);
void L980_appWriteReg(unsigned char reg,unsigned char len,unsigned char *data);
void L980_appReadReq(unsigned char reg,unsigned char len);
extern unsigned  int can_count;//test
#endif /*  CAN_MODBUSRTU__H_ */


