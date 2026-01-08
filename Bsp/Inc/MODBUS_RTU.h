
���л�����
#include "DMA.h"
#include "MODBUS_RTU.h"



extern u8 control_change;


MODBUS_RagTypeDef


uint16_t MODBUS_Crc16(uint8_t *ptr, uint16_t len);  //CRCУ��

//0x03��MODBUS_Read_Rag��
void MODBUS_0x06_Back(u16 add,int data);//0x06������ȷ�ķ���ֵ
void MODBUS_0x10_Back(u16 add,u16 num);//0x10������ȷ�ķ���ֵ
//////////////////////////////////////////
void MODBUS_BackError(u8 Function,u8 error);//MODBUSָ����󷵻�
int MODBUS_ReadAdd_check(u16 add); //������ַ�Ƿ���Ч ��Ч����0
int MODBUS_WriteAdd_check(u16 add); //���д��ַ�Ƿ���Ч ��Ч����0
int MODBUS_Read_one_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add);//�������Ĵ�����ʹ��ʱ�����Ⱦ���MODBUS_Add_check����ַ֮���ٵ��ñ���������Ȼ�����ַ����'e'����ֵ
void MODBUS_write_One_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,int data);//д�뵥���Ĵ���
unsigned char RBUFF_analysis(MODBUS_RagTypeDef *Re_MODBUS_Rag,u8 *RBUFF,u16 len); //MODBUSЭ�����ִ�У��ɲ��ҵ�ַ�͹����룬���ǽ�β������CRCУ��


u8 MODBUS_Read_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 num);//0x03
u8 MODBUS_0x06write_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 data);//0x06
u8 MODBUS_0x10write_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 num,u8 *data);//0x10

**************************************/

#ifndef  __MODBUS_RTU_H__
#define __MODBUS_RTU_H__
//#include "MODBUS_RTU.h"
#include "sys.h"
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
#include "includes.h"     //UCOS 
#endif
///////////////////////�궨��/////////////////////////
//	#define	Addr_Error_sta                 	0x0101	//	����״̬	0x03	0x0-0xFFFF	//λ1 :�����ֹ���//λ2 :�����ֹ�ѹ//λ3 :�����ֱ���������//λ4 :������Ƿѹ//λ5 :�����ֹ���//λ6 :ת���ֱ���//λ7 :������ͨѶ��ʱ //λ8 :������ͨѶ��ʱ

#define Local_address 0xAA  //MODBUSЭ���еı�����ַ
#define BUFF_LEN_MAX USART1_DMA_RBUFF_LENTH     //�ɽ��ܵ��ָ��
/*�豸��Ϣ*/		//	Ѳ�����
/*�豸��Ϣ*/	 	//
#define	Addr_ID                       	0x0001	//	�豸��ʶ  	0x03	--	
#define	Addr_Edition                   	0x0002	//	�豸�汾�� 	0x03		
#define	Addr_Device_info_end           	0x0003	//	�豸��Ϣ��β	0x03		
/*����״̬*/		//
#define	Addr_Error_sta                	0x0101	//	����״̬   	0x03	0x0-0xFFFF	
#define	Addr_Error_sta_end            	0x0102	//	����״̬��β  	0x03	������	
/*ϵͳ״̬*/		//
#define	Addr_Sys_Sta                  	0x0201	//	ϵͳ״̬    	0x03	0x0-0xFFFF	//λ15:FLASH����״̬
#define	Addr_L_Speed                   	0x0202	//	����ת��    	0x03		
#define	Addr_R_Speed                   	0x0203	//	����ת��    	0x03	0/����	0:ֹͣ 1:����
#define	Addr_Retain_1                  	0x0204	//	����	0x03		
#define	Addr_L_distance_H              	0x0205	//	������̸�λ	0x03		
#define	Addr_L_distance_L              	0x0206	//	������̵�λ	0x03		
#define	Addr_R_distance_H              	0x0207	//	������̸�λ	0x03		
#define	Addr_R_distance_L             	0x0208	//	������̵�λ	0x03		
#define	Addr_Angle                     	0x0209	//	��ǰ�Ƕ�	0x03		
#define	Addr_Battery_V                 	0x020A	//	��ص���	0x03		
#define	Addr_Battery_I                	0x020B	//	����ܵ���	0x03		
#define	Addr_FLASH_OK                 	0x020C	//	FLASH�������ݿɶ�	0x03		
#define	Addr_Encoder1                 	0x020D	//	������1��ǰ��ֵ	0x03		
#define	Addr_Encoder2                 	0x020E	//	������2��ǰ��ֵ	0x03		
#define	Addr_Encoder3                 	0x020F	//	������3��ǰ��ֵ	0x03		
#define	Addr_Encoder4                 	0x0210	//	������4��ǰ��ֵ	0x03		
#define	Addr_Sys_Sta_end              	0x0211	//	ϵͳ״̬��β	0x03		
/*�˶�ϵͳ����*/		//
#define	Addr_Start                     	0x0301	//	����ֹͣ        	0x03,0x06,0x10		
#define	Addr_Angle_Set                	0x0302	//	ת��Ƕ�ֵ(*100)	0x03,0x06,0x10		
#define	Addr_L_Speed_Set               	0x0303	//	�趨����ת��    	0x03,0x06,0x10		
#define	Addr_R_Speed_Set               	0x0304	//	�趨����ת��    	0x03,0x06,0x10		
#define	Addr_Angle_Right_front_Set           	0x0305	//	�趨��ǰ�ֽǶ�	0x03,0x06,0x10		
#define	Addr_Angle_Right_rear_Set          	0x0306	//	�趨�Һ��ֽǶ�	0x03,0x06,0x10		
#define	Addr_Angle_Left_front_Set           	0x0307	//	�趨��ǰ�ֽǶ�	0x03,0x06,0x10		
#define	Addr_Angle_Left_rear_Set          	0x0308	//	�趨����ֽǶ�	0x03,0x06,0x10
#define	Addr_Turn_Active            	0x0309	//	ת����	0x03,0x06,0x10		
#define	Addr_Reset_Distance            	0x030A	//	�������	0x03,0x06,0x10
#define	Addr_Motion_sys_end            	0x030B	//	�˶�ϵͳ������β	0x03,0x06,0x10		
/*�˶����ò���*/		//
#define	Addr_Limit_L_1                	0x0401	//	ת��1����λ	0x03,0x06,0x10		
#define	Addr_Limit_R_1                 	0x0402	//	ת��1����λ	0x03,0x06,0x10		
#define	Addr_Turn1_0Encoder           	0x0403	//	ת��1��0�ȱ���ֵ	0x03,0x06,0x10		
#define	Addr_Limit_L_2                	0x0404	//	ת��2����λ	0x03,0x06,0x10		
#define	Addr_Limit_R_2                	0x0405	//	ת��2����λ	0x03,0x06,0x10		
#define	Addr_Turn2_0Encoder           	0x0406	//	ת��2��0�ȱ���ֵ	0x03,0x06,0x10		
#define	Addr_Limit_L_3                	0x0407	//	ת��3����λ	0x03,0x06,0x10		
#define	Addr_Limit_R_3                	0x0408	//	ת��3����λ	0x03,0x06,0x10		
#define	Addr_Turn3_0Encoder           	0x0409	//	ת��3��0�ȱ���ֵ	0x03,0x06,0x10		
#define	Addr_Limit_L_4                	0x040A	//	ת��4����λ	0x03,0x06,0x10		
#define	Addr_Limit_R_4                	0x040B	//	ת��4����λ	0x03,0x06,0x10		
#define	Addr_Turn4_0Encoder           	0x040C	//	ת��4��0�ȱ���ֵ	0x03,0x06,0x10		
#define	Addr_Sports_confi_end         	0x040D	//	�˶����ò�����β	0x03,0x06,0x10		
/*ϵͳ���ܲ���*/		//
#define	Addr_USART_com_OUT_TIME        	0x0501	//	ͨѶ���ڳ�ʱʱ��	0x03,0x06,0x10	0~65535	Ĭ��30��*10ms�� 0������ʱ
#define	Addr_Printf_En                 	0x0502	//	����ϸ����ʾ	0x03,0x06,0x10		
#define	Addr_Retain_3                  	0x0503	//	����	0x03,0x06,0x10		
#define	Addr_Retain_4                  	0x0504	//	����	0x03,0x06,0x10		
#define	Addr_Systemrestart             	0x0505	//	ϵͳ����	0x03,0x06,0x10		
#define	Addr_system_function_end       	0x0506	//	ϵͳ���ܲ�����β	0x03,0x06,0x10		
/*���ò����洢�Ĵ���*/		//
#define	Addr_FLASH_Motor_function_Update    	0xFF01	//	�����˶����ò���	0x06,0x10	1	д��󱣴��˶����ò���
#define	Addr_FLASH_Sys_function_Update       	0xFF02	//	����ϵͳ���ܲ���	0x06,0x10	1	д��󱣴�ϵͳ���ܲ���
#define	Addr_FLASH_Motor_function_default    	0xFF11	//	�˶�ϵͳ�����ָ�Ĭ��ֵ	0x06,0x10	1	�˶�ϵͳ�����ָ�Ĭ��ֵ
#define	Addr_FLASH_Sys_function_default      	0xFF12	//	ϵͳ���ܲ����ָ�Ĭ��ֵ	0x06,0x10	1	ϵͳ���ܲ����ָ�Ĭ��ֵ
#define	Addr_FLASH_Read_Motor_function      	0xFF21	//	��ȡ�˶�ϵͳ����	0x06,0x10	1	��FLASH��ȡ��ȡ�˶�ϵͳ����
#define	Addr_FLASH_Read_Sys_function         	0xFF22	//	��ȡϵͳ���ܲ���	0x06,0x10	1	��FLASH��ȡ��ȡϵͳ���ܲ���
#define	Addr_Conf_storage_end               	0xFF23	//	���ò����洢�Ĵ�����β	0x06,0x10		

///////////////////�쳣��//////////////////////////////////
#define UNkwon_ERROR       0x00      //δ֪����     
#define FunctionCode_ERROR 0x01      //���������
#define Addr_ERROR 0x02              //��ַ����
#define DATA_ERROR 0x03              //���ݴ���
#define CRC_ERROR 0x04               //У�����
#define ACT_ERROR 0x05               //ָ��ִ��ʧ��

///////////////////////ͨ�Žṹ��/////////////////////////
/*
MOD_Error_sta
λ0: ������ͨѶ��ʱ
λ1: ������ͨѶ��ʱ
λ2: �����ֱ���
λ3: ת���ֱ���
λ4:�����ֹ���
λ5:�����ֹ�ѹ
λ6:�����ֱ���������
λ7:������Ƿѹ
λ8:�����ֹ���
λ9:
λ10:
λ11:
λ12:

MOD_sys_sta
λ0:�д���
λ1:�˶�״̬
λ2:
λ3:
λ4:
λ5:
λ6:
λ15:  ������FLASH��ͬ��
*/
/*���ݽṹ*/
typedef struct
{
    /*�豸��Ϣ*/
    short	MOD_ID                       	;//	�豸��ʶ  	0x03	--
    short	MOD_Edition                   	;//	�豸�汾�� 	0x03
    short	MOD_Device_info_end           	;//	�豸��Ϣ��β	0x03
    /*����״̬*/
    short	MOD_Error_sta                	;//	����״̬   	0x03	0x0-0xFFFF
    short	MOD_Error_sta_end            	;//	����״̬��β  	0x03	������
    /*ϵͳ״̬*/
    short	MOD_Sys_Sta                  	;//	ϵͳ״̬    	0x03	0x0-0xFFFF	//λ15:FLASH����״̬
    short	MOD_L_Speed                   	;//	����ת��    	0x03
    short	MOD_R_Speed                   	;//	����ת��    	0x03	0/����	0:ֹͣ 1:����
    short	MOD_Retain_1                  	;//	����	0x03
    short	MOD_L_distance_H              	;//	������̸�λ	0x03
    short	MOD_L_distance_L              	;//	������̵�λ	0x03
    short	MOD_R_distance_H              	;//	������̸�λ	0x03
    short	MOD_R_distance_L             	;//	������̵�λ	0x03
    short	MOD_Angle                     	;//	��ǰ�Ƕ�	0x03
    short	MOD_Battery_V                 	;//	��ص���	0x03
    short	MOD_Battery_I                	;//	����ܵ���	0x03
    short	MOD_FLASH_OK                 	;//	FLASH�������ݿɶ�	0x03
    short	MOD_Encoder1                 	;//	������1��ǰ��ֵ	0x03
    short	MOD_Encoder2                 	;//	������2��ǰ��ֵ	0x03
    short	MOD_Encoder3                 	;//	������3��ǰ��ֵ	0x03
    short	MOD_Encoder4                 	;//	������4��ǰ��ֵ	0x03
    short	MOD_Sys_Sta_end              	;//	ϵͳ״̬��β	0x03
    /*�˶�ϵͳ����*/
    short	MOD_Start                     	;//	����ֹͣ        	0x03,0x06,0x10
    short	MOD_Angle_Set                	;//	ת��Ƕ�ֵ(*100)	0x03,0x06,0x10
    short	MOD_L_Speed_Set               	;//	�趨����ת��    	0x03,0x06,0x10
    short	MOD_R_Speed_Set               	;//	�趨����ת��    	0x03,0x06,0x10
    short	MOD_Angle_Right_front_Set           	;//	�趨��ǰ�ֽǶ�	0x03,0x06,0x10
    short	MOD_Angle_Right_rear_Set          	;//	�趨�Һ��ֽǶ�	0x03,0x06,0x10
    short	MOD_Angle_Left_front_Set           	;//	�趨��ǰ�ֽǶ�	0x03,0x06,0x10
    short	MOD_Angle_Left_rear_Set          	;//	�趨����ֽǶ�	0x03,0x06,0x10
    short	MOD_Turn_Active            	;//	ת�ֶ���ִ��	0x03,0x06,0x10
    short	MOD_Reset_Distance            	;//	�������	0x03,0x06,0x10
    short	MOD_Motion_sys_end            	;//	�˶�ϵͳ������β	0x03,0x06,0x10
    /*�˶����ò���*/
    short	MOD_Limit_L_1                	;//	ת��1����λ	0x03,0x06,0x10
    short	MOD_Limit_R_1                 	;//	ת��1����λ	0x03,0x06,0x10
    short	MOD_Turn1_0Encoder           	;//	ת��1��0�ȱ���ֵ	0x03,0x06,0x10
    short	MOD_Limit_L_2                	;//	ת��2����λ	0x03,0x06,0x10
    short	MOD_Limit_R_2                	;//	ת��2����λ	0x03,0x06,0x10
    short	MOD_Turn2_0Encoder           	;//	ת��2��0�ȱ���ֵ	0x03,0x06,0x10
    short	MOD_Limit_L_3                	;//	ת��3����λ	0x03,0x06,0x10
    short	MOD_Limit_R_3                	;//	ת��3����λ	0x03,0x06,0x10
    short	MOD_Turn3_0Encoder           	;//	ת��3��0�ȱ���ֵ	0x03,0x06,0x10
    short	MOD_Limit_L_4                	;//	ת��4����λ	0x03,0x06,0x10
    short	MOD_Limit_R_4                	;//	ת��4����λ	0x03,0x06,0x10
    short	MOD_Turn4_0Encoder           	;//	ת��4��0�ȱ���ֵ	0x03,0x06,0x10
    short	MOD_Sports_confi_end         	;//	�˶����ò�����β	0x03,0x06,0x10
    /*ϵͳ���ܲ���*/
    short	MOD_USART_com_OUT_TIME        	;//	ͨѶ���ڳ�ʱʱ��	0x03,0x06,0x10	0~65535	Ĭ��30��*10ms�� 0������ʱ
    short	MOD_Printf_En                 	;//	����ϸ����ʾ	0x03,0x06,0x10
    short	MOD_Retain_3                  	;//	����	0x03,0x06,0x10
    short	MOD_Retain_4                  	;//	����	0x03,0x06,0x10
    short	MOD_Systemrestart             	;//	ϵͳ����	0x03,0x06,0x10
    short	MOD_system_function_end       	;//	ϵͳ���ܲ�����β	0x03,0x06,0x10
    /*���ò����洢�Ĵ���*/
    short	MOD_FLASH_Motor_function_Update    	;//	�����˶����ò���	0x06,0x10	1	д��󱣴��˶����ò���
    short	MOD_FLASH_Sys_function_Update       	;//	����ϵͳ���ܲ���	0x06,0x10	1	д��󱣴�ϵͳ���ܲ���
    short	MOD_FLASH_Motor_function_default    	;//	�˶�ϵͳ�����ָ�Ĭ��ֵ	0x06,0x10	1	�˶�ϵͳ�����ָ�Ĭ��ֵ
    short	MOD_FLASH_Sys_function_default      	;//	ϵͳ���ܲ����ָ�Ĭ��ֵ	0x06,0x10	1	ϵͳ���ܲ����ָ�Ĭ��ֵ
    short	MOD_FLASH_Read_Motor_function      	;//	��ȡ�˶�ϵͳ����	0x06,0x10	1	��FLASH��ȡ��ȡ�˶�ϵͳ����
    short	MOD_FLASH_Read_Sys_function         	;//	��ȡϵͳ���ܲ���	0x06,0x10	1	��FLASH��ȡ��ȡϵͳ���ܲ���
    short	MOD_Conf_storage_end               	;//	���ò����洢�Ĵ�����β	0x06,0x10

} MODBUS_RagTypeDef;
///////////////////�ɱ�����FLASH�Ĳ���Ĭ��ֵ//////////////////////////////
#define USART_com_OUT_TIME_default           	50             //ͨѶ���ڳ�ʱʱ��Ĭ����ֵ *10ms
#define distance_clear_after_read_default    	0x0             //�������ȡ���Ƿ���������
#define Stop_Entime_default             	100		//	��������ʱʱ��
#define MOD_Turn_er_Clear_default        	0x0   //	ת���ֱ����������

#define Limit_L_default                  	12200	//	ת������λ	 
#define Limit_R_default                 	4200	//	ת������λ 
#define Encoder_Middle_default         	 	8200	//	�������м�λ��
#define PID_P_default                   	200				//ת��PIDĬ�ϲ���P
#define PID_I_default                    	500				//ת��PIDĬ�ϲ���I
#define	PID_D_default											30				//ת��PIDĬ�ϲ���D

///////////////////FLASH��ַ//////////////////////////////////////////////
//ÿ��������short��u16 ռ�����ڴ��ַ��  0x08070000=str[0]  0x08070002=str[1],�ҵ�ַ��дʱ������ż��
//#define Addr_Flash_Move_Sys         0x08070000    //Ԥ��ʮ����u16����  //�˶�ϵͳ����
#define Addr_Flash_Move             0x08070032    //Ԥ��ʮ����u16����  //�˶����ò���
#define Addr_Flash_Sys_com          0x08070064    //Ԥ��ʮ����u16����  //ϵͳ���ܲ���
#define Addr_Flash_Flash            0x08070096    // ���ò����洢�Ĵ���
//���ò����洢�Ĵ���
///////////////////////��������/////////////////////////
extern u8 control_change;//ָʾ�в������ı���
//λ0��Start
//λ1��L_PWM
//λ2��R_PWM
///////////////////////��������/////////////////////////

unsigned char RBUFF_analysis(MODBUS_RagTypeDef *Re_MODBUS_Rag,u8 *RBUFF,u16 len); //MODBUSЭ�����ִ�У��ɲ��ҵ�ַ�͹����룬���ǽ�β������CRCУ��
/////////////R/////////////////////
u8 MODBUS_Read_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 num);//0x03
u8 MODBUS_0x06write_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 data);//0x06
u8 MODBUS_0x10write_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 num,u8 *data);//0x10
////////////T/////////////////////////
//0x03��MODBUS_Read_Rag��
void MODBUS_0x06_Back(u16 add,int data);//0x06������ȷ�ķ���ֵ
void MODBUS_0x10_Back(u16 add,u16 num);//0x10������ȷ�ķ���ֵ
//////////////////////////////////////////
void MODBUS_BackError(u8 Function,u8 error);//MODBUSָ����󷵻�
int MODBUS_Addr_check(u16 *Addr,u8 num,u16 addr);//�Ӷ����ַ�����ڼ�������ַ�Ƿ���������
int MODBUS_ReadAddr_check(u16 add); //������ַ�Ƿ���Ч ��Ч����0
int MODBUS_WriteAddr_check(u16 add); //���д��ַ�Ƿ���Ч ��Ч����0

u32 MODBUS_Read_one_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add);//�������Ĵ�����ʹ��ʱ�����Ⱦ���MODBUS_Add_check����ַ֮���ٵ��ñ���������Ȼ�����ַ����'e'����ֵ
u8  MODBUS_write_One_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,short data);//д�뵥���Ĵ���

void MODBUS_Rag_Init(MODBUS_RagTypeDef *MODBUS_Raginit);//MODBUS������ʼ��

void MODBUS_FLASH_Write(MODBUS_RagTypeDef *MODBUS_Raginit,u8 com);    //�����ֽṹ���еĲ���д��falsh
void MODBUS_FLASH_Read(MODBUS_RagTypeDef *MODBUS_Raginit,u8 com);     //��flash�ж������ֽṹ�����
void MODBUS_FLASH_default(MODBUS_RagTypeDef *MODBUS_Raginit,u8 com);  //�ظ�Ĭ�����ò�����Flash
uint16_t MODBUS_Crc16(uint8_t *ptr, uint16_t len);  //CRCУ��
#endif
/*-----------------file end-----------------------*/
