#include "MODBUS_RTU.h"
#include "sys.h"

MODBUS_RagTypeDef

uint16_t MODBUS_Crc16(uint8_t *ptr, uint16_t len);  //CRCУ��


void MODBUS_0x06_Back(u16 add,int data);//0x06������ȷ�ķ���ֵ
void MODBUS_0x10_Back(u16 add,u16 num);//0x10������ȷ�ķ���ֵ

void MODBUS_BackError(u8 Function,u8 error);//MODBUSָ����󷵻�
int MODBUS_ReadAdd_check(u16 add); //������ַ�Ƿ���Ч ��Ч����0
int MODBUS_WriteAdd_check(u16 add); //���д��ַ�Ƿ���Ч ��Ч����0
int MODBUS_Read_one_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add);//�������Ĵ�����ʹ��ʱ�����Ⱦ���MODBUS_Add_check����ַ֮���ٵ��ñ���������Ȼ�����ַ����'e'����ֵ
void MODBUS_write_One_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,int data);//д�뵥���Ĵ���
unsigned char RBUFF_analysis(MODBUS_RagTypeDef *Re_MODBUS_Rag,u8 *RBUFF,u16 len); //MODBUSЭ�����ִ�У��ɲ��ҵ�ַ�͹����룬���ǽ�β������CRCУ��

/////////////R/////////////////////
u8 MODBUS_Read_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 num);//0x03
u8 MODBUS_0x06write_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 data);//0x06
u8 MODBUS_0x10write_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 num,u8 *data);//0x10

**************************************/
#include "MODBUS_RTU.h"
#include "DMA.h"
#include "USART_BASE.h"
#include "Main.h"
#include "stmflash.h"
//////////////////��ַ�б�////////////////////////////////////
u16 WriteAddr[]= {
    Addr_Start,Addr_Motion_sys_end,
    Addr_Limit_L_1,Addr_Sports_confi_end,
    Addr_USART_com_OUT_TIME,Addr_system_function_end,
    Addr_FLASH_Motor_function_Update,Addr_Conf_storage_end
};

u8 WriteAddrnum=4;

u16 ReadAddr[]= {
    Addr_ID,Addr_Device_info_end ,
    Addr_Error_sta,Addr_Error_sta_end,
    Addr_Sys_Sta,Addr_Sys_Sta_end,
    Addr_Start,Addr_Motion_sys_end,
    Addr_Limit_L_1,Addr_Sports_confi_end,
    Addr_USART_com_OUT_TIME,Addr_system_function_end,
};
u8 ReadAddrnum=6;
/*-------------------------------------------
//��������MODBUS_Rag_Init
//������ MODBUS������ʼ��
//������MODBUS_RagTypeDef *MODBUS_Raginit   �ṹ��
//����ֵ����
//���������б�����

---------------------------------------------*/
void MODBUS_Rag_Init(MODBUS_RagTypeDef *MODBUS_Raginit)
{
    MODBUS_Raginit->	MOD_ID                       	=0x0000	;//	�豸��ʶ  	0x03	--
    MODBUS_Raginit->	MOD_Edition                   	=0x0000	;//	�豸�汾�� 	0x03
    MODBUS_Raginit->	MOD_Device_info_end           	=0x0000	;//	�豸��Ϣ��β	0x03
    /*����״̬*/
    MODBUS_Raginit->	MOD_Error_sta                	=0x0000	;//	����״̬   	0x03	0x0-0xFFFF
    MODBUS_Raginit->	MOD_Error_sta_end            	=0x0000	;//	����״̬��β  	0x03	������
    /*ϵͳ״̬*/
    MODBUS_Raginit->	MOD_Sys_Sta                  	=0x0000	;//	ϵͳ״̬    	0x03	0x0-0xFFFF	//λ15:FLASH����״̬
    MODBUS_Raginit->	MOD_L_Speed                   	=0x0000	;//	����ת��    	0x03
    MODBUS_Raginit->	MOD_R_Speed                   	=0x0000	;//	����ת��    	0x03	0/����	0:ֹͣ 1:����
    MODBUS_Raginit->	MOD_Retain_1                  	=0x0000	;//	����	0x03
    MODBUS_Raginit->	MOD_L_distance_H              	=0x0000	;//	������̸�λ	0x03
    MODBUS_Raginit->	MOD_L_distance_L              	=0x0000	;//	������̵�λ	0x03
    MODBUS_Raginit->	MOD_R_distance_H              	=0x0000	;//	������̸�λ	0x03
    MODBUS_Raginit->	MOD_R_distance_L             	=0x0000	;//	������̵�λ	0x03
    MODBUS_Raginit->	MOD_Angle                     	=0x0000	;//	��ǰ�Ƕ�	0x03
    MODBUS_Raginit->	MOD_Battery_V                 	=0x0000	;//	��ص���	0x03
    MODBUS_Raginit->	MOD_Battery_I                	=0x0000	;//	����ܵ���	0x03
    MODBUS_Raginit->	MOD_FLASH_OK                 	=0x0000	;//	FLASH�������ݿɶ�	0x03
    MODBUS_Raginit->	MOD_Encoder1                 	=0x0000	;//	������1��ǰ��ֵ	0x03
    MODBUS_Raginit->	MOD_Encoder2                 	=0x0000	;//	������2��ǰ��ֵ	0x03
    MODBUS_Raginit->	MOD_Encoder3                 	=0x0000	;//	������3��ǰ��ֵ	0x03
    MODBUS_Raginit->	MOD_Encoder4                 	=0x0000	;//	������4��ǰ��ֵ	0x03
    MODBUS_Raginit->	MOD_Sys_Sta_end              	=0x0000	;//	ϵͳ״̬��β	0x03
    /*�˶�ϵͳ����*/
    MODBUS_Raginit->	MOD_Start                     	=0x0000	;//	����ֹͣ        	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Angle_Set                	=0x0000	;//	ת��Ƕ�ֵ(*100)	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_L_Speed_Set               	=0x0000	;//	�趨����ת��    	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_R_Speed_Set               	=0x0000	;//	�趨����ת��    	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Angle_Right_front_Set           	=0x0001	;//	�趨��ǰ�ֽǶ�	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Angle_Right_rear_Set          	=0x0001	;//	�趨�Һ��ֽǶ�	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Angle_Left_front_Set           	=0x0001	;//	�趨��ǰ�ֽǶ�	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Angle_Left_rear_Set          	=0x0001	;//	�趨����ֽǶ�
    MODBUS_Raginit->	MOD_Turn_Active            	=0x0000	;//	ת�ֶ���ִ��	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Reset_Distance            	=0x0000	;//	�������	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Motion_sys_end            	=0x0000	;//	�˶�ϵͳ������β	0x03,0x06,0x10
    /*�˶����ò���*/
    MODBUS_Raginit->	MOD_Limit_L_1                	=0x0000	;//	ת��1����λ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Limit_R_1                 =0x0000	;//	ת��1����λ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Turn1_0Encoder           	=0x04D1	;//	ת��1��0�ȱ���ֵ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Limit_L_2                	=0x0000	;//	ת��2����λ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Limit_R_2                	=0x0000	;//	ת��2����λ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Turn2_0Encoder           	=0x03FC	;//	ת��2��0�ȱ���ֵ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Limit_L_3                	=0x0000	;//	ת��3����λ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Limit_R_3                	=0x0000	;//	ת��3����λ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Turn3_0Encoder           	=0x053E	;//	ת��3��0�ȱ���ֵ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Limit_L_4                	=0x0000	;//	ת��4����λ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Limit_R_4                	=0x0000	;//	ת��4����λ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Turn4_0Encoder           	=0x045A	;//	ת��4��0�ȱ���ֵ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Sports_confi_end         	=0x0000	;//	�˶����ò�����β	0x03,0x06,0x10
    /*ϵͳ���ܲ���*/
    MODBUS_Raginit->	MOD_USART_com_OUT_TIME        	=20	;//	ͨѶ���ڳ�ʱʱ��	0x03,0x06,0x10	0~65535	Ĭ��30��*10ms�� 0������ʱ
    MODBUS_Raginit->	MOD_Printf_En                  	=0x1	;//	����ϸ����ʾ	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Retain_3                  	=0x0000	;//	����	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Retain_4                  	=0x0000	;//	����	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_Systemrestart             	=0x0000	;//	ϵͳ����	0x03,0x06,0x10
    MODBUS_Raginit->	MOD_system_function_end       	=0x0000	;//	ϵͳ���ܲ�����β	0x03,0x06,0x10
    /*���ò����洢�Ĵ���*/
    MODBUS_Raginit->	MOD_FLASH_Motor_function_Update    	=0x0000	;//	�����˶����ò���	0x06,0x10	1	д��󱣴��˶����ò���
    MODBUS_Raginit->	MOD_FLASH_Sys_function_Update       	=0x0000	;//	����ϵͳ���ܲ���	0x06,0x10	1	д��󱣴�ϵͳ���ܲ���
    MODBUS_Raginit->	MOD_FLASH_Motor_function_default    	=0x0000	;//	�˶�ϵͳ�����ָ�Ĭ��ֵ	0x06,0x10	1	�˶�ϵͳ�����ָ�Ĭ��ֵ
    MODBUS_Raginit->	MOD_FLASH_Sys_function_default      	=0x0000	;//	ϵͳ���ܲ����ָ�Ĭ��ֵ	0x06,0x10	1	ϵͳ���ܲ����ָ�Ĭ��ֵ
    MODBUS_Raginit->	MOD_FLASH_Read_Motor_function      	=0x0000	;//	��ȡ�˶�ϵͳ����	0x06,0x10	1	��FLASH��ȡ��ȡ�˶�ϵͳ����
    MODBUS_Raginit->	MOD_FLASH_Read_Sys_function         	=0x0000	;//	��ȡϵͳ���ܲ���	0x06,0x10	1	��FLASH��ȡ��ȡϵͳ���ܲ���
    MODBUS_Raginit->	MOD_Conf_storage_end               	=0x0000	;//	���ò����洢�Ĵ�����β	0x06,0x10

}
/*-------------------------------------------
//��������MODBUS_Read_one_Rag    �������Ĵ�����ʹ��ʱ�����Ⱦ���MODBUS_Add_check����ַ֮���ٵ��ñ���������Ȼ�����ַ����'e'����ֵ
//���������������е�ַ��飬�����ַ��ֱ�ӷ���'e',�����д����飬�������ǲ��ᴥ����
//������MODBUS_RagTypeDef *Re_MODBUS_Rag   ���������ݵĻ�����
,u16 add    ��Ҫ�ļĴ�����ַ
//����ֵ����ȡ�Ĵ������ݣ���Ҫע��abcdeСд��ĸ���п��ܳ�����
//���������б���
MODBUS_ReadAdd_check ����ַ�Ƿ���Ч
---------------------------------------------*/
u32 MODBUS_Read_one_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add)
{
    u8 add_place;//��ʶ��ַ��ʲô������
    add_place=MODBUS_ReadAddr_check(add);
    if(add_place)
    {
        switch(add_place)
        {
        case 1: { //�豸��Ϣ
            switch(add)
            {
            case	Addr_ID:	{
                return Re_MODBUS_Rag->	MOD_ID;   //	�豸��ʶ
                case	Addr_Edition                   	: {
                    return Re_MODBUS_Rag->	MOD_Edition                   	;   //	�豸�汾��
                }
                case	Addr_Device_info_end           	: {
                    return Re_MODBUS_Rag->	MOD_Device_info_end           	;   //	�豸��Ϣ��β
                }

            }
            default : {
                return 0x000010000;   //��ַ����////�쳣��Addr_ERROR   �����������²������
            }
            }
        }
        case 2: { //����״̬
            switch(add)
            {
            case	Addr_Error_sta                	: {
                return Re_MODBUS_Rag->	MOD_Error_sta                	;   //	����״̬
            }
            case	Addr_Error_sta_end            	: {
                return Re_MODBUS_Rag->	MOD_Error_sta_end            	;   //	����״̬��β
            }

            default : {
                return 0x000010000;   //��ַ����////�쳣��Addr_ERROR   �����������²������
            }
            }
        }
        case 3: { //ϵͳ״̬
            switch(add)
            {
            case	Addr_Sys_Sta:	{
                return Re_MODBUS_Rag->	MOD_FLASH_OK;   //	FLASH�������ݿɶ�
                case	Addr_L_Speed                   	: {
                    return Re_MODBUS_Rag->	MOD_L_Speed                   	;   //	����ת��
                }
                case	Addr_R_Speed                   	: {
                    return Re_MODBUS_Rag->	MOD_R_Speed                   	;   //	����ת��
                }
                case	Addr_Retain_1                  	: {
                    return Re_MODBUS_Rag->	MOD_Retain_1                  	;   //	����
                }
                case	Addr_L_distance_H              	: {
                    // Send_ActiveSem(Ac_Read_L_Distance,0,0); //ΪActive�������ź���
                    return Re_MODBUS_Rag->	MOD_L_distance_H              	;   //	������̸�λ
                }
                case	Addr_L_distance_L              	: {
                    // Send_ActiveSem(Ac_Read_L_Distance,0,0); //ΪActive�������ź���
                    return Re_MODBUS_Rag->	MOD_L_distance_L              	;   //	������̵�λ

                }
                case	Addr_R_distance_H              	: {
                    // Send_ActiveSem(Ac_Read_R_Distance,0,0); //ΪActive�������ź���
                    return Re_MODBUS_Rag->	MOD_R_distance_H              	;   //	������̸�λ
                }
                case	Addr_R_distance_L             	: {
                    //Send_ActiveSem(Ac_Read_R_Distance,0,0); //ΪActive�������ź���
                    return Re_MODBUS_Rag->	MOD_R_distance_L             	;   //	������̵�λ
                }
                case	Addr_Angle                     	: {
                    return Re_MODBUS_Rag->	MOD_Angle                     	;   //	��ǰ�Ƕ�
                }
                case	Addr_Battery_V                 	: {
                    return Re_MODBUS_Rag->	MOD_Battery_V                 	;   //	��ص���
                }
                case	Addr_Battery_I                	: {
                    return Re_MODBUS_Rag->	MOD_Battery_I                	;   //	����ܵ���
                }
                case	Addr_FLASH_OK                 	: {
                    return Re_MODBUS_Rag->	MOD_FLASH_OK                 	;   //	FLASH�������ݿɶ�
                }
                case	Addr_Encoder1                 	: {
									printf("1:%.2f ",(Re_MODBUS_Rag->MOD_Encoder1-Re_MODBUS_Rag->MOD_Turn1_0Encoder)/45.5);
                    return (Re_MODBUS_Rag->MOD_Encoder1-Re_MODBUS_Rag->MOD_Turn1_0Encoder)/4.55;   //	������1��ǰ��ֵ
                }
                case	Addr_Encoder2                 	: {
									printf("2:%.2f ",(Re_MODBUS_Rag->MOD_Encoder2-Re_MODBUS_Rag->MOD_Turn2_0Encoder)/45.5);
                    return (Re_MODBUS_Rag->MOD_Encoder2-Re_MODBUS_Rag->MOD_Turn2_0Encoder)/4.55;   //	������2��ǰ��ֵ
                }
                case	Addr_Encoder3                 	: {
									printf("3:%.2f ",(Re_MODBUS_Rag->MOD_Encoder3-Re_MODBUS_Rag->MOD_Turn3_0Encoder)/45.5);
                    return (Re_MODBUS_Rag->MOD_Encoder3-Re_MODBUS_Rag->MOD_Turn3_0Encoder)/4.55;   //	������3��ǰ��ֵ
                }
                case	Addr_Encoder4                 	: {
									printf("4:%.2f ",(Re_MODBUS_Rag->MOD_Encoder4-Re_MODBUS_Rag->MOD_Turn4_0Encoder)/45.5);
                    return (Re_MODBUS_Rag->MOD_Encoder4-Re_MODBUS_Rag->MOD_Turn4_0Encoder)/4.55;   //	������4��ǰ��ֵ
                }
                case	Addr_Sys_Sta_end              	: {
                    return Re_MODBUS_Rag->	MOD_Sys_Sta_end              	;   //	ϵͳ״̬��β
                }
            }
            default : {
                return 0x000010000;   //��ַ����////�쳣��Addr_ERROR   �����������²������
            }
            }
        }
        case 4: {//�˶����ò���
            switch(add)
            {
            case	Addr_Start                     	: {
                return Re_MODBUS_Rag->	MOD_Start                     	;   //	����ֹͣ
            }
            case	Addr_Angle_Set                	: {
                return Re_MODBUS_Rag->	MOD_Angle_Set                	;   //	ת��Ƕ�ֵ(*100)
            }
            case	Addr_L_Speed_Set               	: {
                return Re_MODBUS_Rag->	MOD_L_Speed_Set               	;   //	�趨����ת��
            }
            case	Addr_R_Speed_Set               	: {
                return Re_MODBUS_Rag->	MOD_R_Speed_Set               	;   //	�趨����ת��
            }
            case	Addr_Angle_Right_front_Set        	: {
                //	(MODBUS_Rag.MOD_Encoder1-MODBUS_Rag.MOD_Turn1_0Encoder)/4.55)
                return Re_MODBUS_Rag->	MOD_Angle_Right_front_Set ;   //	�趨��ǰ�ֽǶ�    EN1

            }
            case	Addr_Angle_Right_rear_Set           	: {
                return Re_MODBUS_Rag->MOD_Angle_Right_front_Set	;   //	�趨�Һ��ֽǶ�   EN2
            }
            case	Addr_Angle_Left_front_Set	: {
                return Re_MODBUS_Rag->MOD_Angle_Left_front_Set	;   //	�趨��ǰ�ֽǶ�   EN3
            }
            case	Addr_Angle_Left_rear_Set	: {
                return Re_MODBUS_Rag->MOD_Angle_Left_rear_Set	;   //	�趨����ֽǶ�     EN4
            }
            case	Addr_Turn_Active	: {

                return Re_MODBUS_Rag->	MOD_Turn_Active            	;
            }//	ת�ֶ���ִ��
            case	Addr_Reset_Distance            	: {
                return Re_MODBUS_Rag->	MOD_Reset_Distance            	;
            }//	�������

            case	Addr_Motion_sys_end            	: {
                return Re_MODBUS_Rag->	MOD_Motion_sys_end            	;   //	�˶�ϵͳ������β
            }

            default : {
                return 0x000010000;   //��ַ����////�쳣��Addr_ERROR   �����������²������
            }
            }
        }
        case 5: {  //�˶����ò���
            switch(add)
            {
            case	Addr_Limit_L_1                	: {
                return Re_MODBUS_Rag->	MOD_Limit_L_1                	;   //	ת��1����λ
            }
            case	Addr_Limit_R_1                 	: {
                return Re_MODBUS_Rag->	MOD_Limit_R_1                 	;   //	ת��1����λ
            }
            case	Addr_Turn1_0Encoder           	: {
                return Re_MODBUS_Rag->	MOD_Turn1_0Encoder           	;   //	ת��1��0�ȱ���ֵ
            }
            case	Addr_Limit_L_2                	: {
                return Re_MODBUS_Rag->	MOD_Limit_L_2                	;   //	ת��2����λ
            }
            case	Addr_Limit_R_2                	: {
                return Re_MODBUS_Rag->	MOD_Limit_R_2                	;   //	ת��2����λ
            }
            case	Addr_Turn2_0Encoder           	: {
                return Re_MODBUS_Rag->	MOD_Turn2_0Encoder           	;   //	ת��2��0�ȱ���ֵ
            }
            case	Addr_Limit_L_3                	: {
                return Re_MODBUS_Rag->	MOD_Limit_L_3                	;   //	ת��3����λ
            }
            case	Addr_Limit_R_3                	: {
                return Re_MODBUS_Rag->	MOD_Limit_R_3                	;   //	ת��3����λ
            }
            case	Addr_Turn3_0Encoder           	: {
                return Re_MODBUS_Rag->	MOD_Turn3_0Encoder           	;   //	ת��3��0�ȱ���ֵ
            }
            case	Addr_Limit_L_4                	: {
                return Re_MODBUS_Rag->	MOD_Limit_L_4                	;   //	ת��4����λ
            }
            case	Addr_Limit_R_4                	: {
                return Re_MODBUS_Rag->	MOD_Limit_R_4                	;   //	ת��4����λ
            }
            case	Addr_Turn4_0Encoder           	: {
                return Re_MODBUS_Rag->	MOD_Turn4_0Encoder           	;   //	ת��4��0�ȱ���ֵ
            }
            case	Addr_Sports_confi_end         	: {
                return Re_MODBUS_Rag->	MOD_Sports_confi_end         	;   //	�˶����ò�����β
            }
            default : {
                return 0x000010000;   //��ַ����////�쳣��Addr_ERROR   �����������²������
            }
            }
        }
        case 6: { //ϵͳ���ܲ���
            switch(add)
            {
            case	Addr_USART_com_OUT_TIME        	: {
                return Re_MODBUS_Rag->	MOD_USART_com_OUT_TIME        	;   //	ͨѶ���ڳ�ʱʱ��
            }
            case	Addr_Printf_En                 	: {
                return Re_MODBUS_Rag->	MOD_Printf_En                  	;
            }//	����

            case	Addr_Retain_3                  	: {
                return Re_MODBUS_Rag->	MOD_Retain_3                  	;   //	����
            }
            case	Addr_Retain_4                  	: {
                return Re_MODBUS_Rag->	MOD_Retain_4                  	;   //	����
            }
            case	Addr_Systemrestart             	: {
                return Re_MODBUS_Rag->	MOD_Systemrestart             	;   //	ϵͳ����
            }
            case	Addr_system_function_end       	: {
                return Re_MODBUS_Rag->	MOD_system_function_end       	;   //	ϵͳ���ܲ�����β
            }

            default : {
                return 0x000010000;   //��ַ����////�쳣��Addr_ERROR   �����������²������
            }
            }
        }
        default: {
            return 0x000010000;
        }
        }
    }
    else
    {
        return 0x000010000;
    }
}

u8 control_change=0;//ָʾ�в������ı���
//λ0��Start
//λ2��L_PWM
//λ3��R_PWM
/*-------------------------------------------
//��������MODBUS_write_One_Rag   д��һ���Ĵ������Դ���ַУ��
//������   д��ָ���ṹ���ָ��Ԫ��
//������MODBUS_RagTypeDef *Re_MODBUS_Rag,����ṹ��
u16 add,�Ĵ�����ַ
int data д������
//����ֵ��
0:�ɹ�
1:��ַ����////�쳣��Addr_ERROR
2:ִ���쳣////ACT_ERROR
3:���ݳ���////DATA_ERROR 0x03
//���������б�����
---------------------------------------------*/
u8 CAN_Data_Clear_Distance[8]= {0x23,0x64,0x60,0x00,0x00,0x00,0x00,0x00}; //������
//0xE1
//uint8_t Angle_Move_Stop[4]= {0x00,0x7C,0x00,0xE1}; //ת������Ϊ���λ��ģʽ��Ҳ�Ƕ�������
uint8_t Angle_Move_Lenth0[9]= {0x00,0x1E,0x00,0x02,0x04,0x00,0x00,0x00,0x00}; //�˶�������Ϊ0

u8 MODBUS_write_One_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,short data)
{
    u8 add_place;//��ʶ��ַ��ʲô������
    add_place=MODBUS_WriteAddr_check(add);
    if(add_place)
    {
        switch(add_place)
        {
        case 1: { ///*	�˶�ϵͳ����	*/
            switch(add)
            {
            case	Addr_Start:	{     //�˶�����
                if(Re_MODBUS_Rag->MOD_Start!=data)    //�иı��������
                {
                    Re_MODBUS_Rag->MOD_Start=data;    //	����ֹͣ
                    
                }
                break;
            }
            case	Addr_Angle_Set	: { //�Ƕ��趨
                break;
            }//	ת��Ƕ�ֵ(*100)
            case	Addr_L_Speed_Set: { //	�趨����ת��
                if(Re_MODBUS_Rag->MOD_Start)
                {
                    Re_MODBUS_Rag->	MOD_L_Speed_Set=data;
                  
                }
                break;
            }//	�趨����ת��
            case	Addr_R_Speed_Set: { //	�趨����ת��
                if(Re_MODBUS_Rag->MOD_Start)
                {
                  Re_MODBUS_Rag->	MOD_R_Speed_Set=data;
									delay_ms(10);
                 
                }
                break;
            }//	�趨����ת��
            /******************************ת��*****************************************/
            case	Addr_Angle_Right_front_Set: {
                if(data<0||data>180) {
                    return 3;   //���ݴ���
                }
                {
									Re_MODBUS_Rag->	MOD_Angle_Right_front_Set=data;
               }
                break;
            }//	�趨��ǰ�ֽǶ�

            case	Addr_Angle_Right_rear_Set	: {
                if(data<0||data>180) {
                    return 3;   //���ݴ���
                }
                {
									Re_MODBUS_Rag->	MOD_Angle_Right_rear_Set=data;//	�����ֽǶ�
                }
                break;
            }//	�趨�Һ��ֽǶ�
            case	Addr_Angle_Left_front_Set 	: {
                if(data<0||data>180) {
                    return 3;   //���ݴ���
                }
                {
                    Re_MODBUS_Rag->	MOD_Angle_Left_front_Set=data;//	�Ӷ��ֽǶ�
                }
                break;
            }//	�趨��ǰ�ֽǶ�
            case	Addr_Angle_Left_rear_Set 	: {
                if(data<0||data>180) {
                    return 3;   //���ݴ���
                }
                //if(Re_MODBUS_Rag->MOD_Angle_Left_rear_Set!=data)
                {
									Re_MODBUS_Rag->	MOD_Angle_Left_rear_Set=data;//	�Ӷ��ֽǶ�
                }
                break;
            }//	�趨����ֽǶ�
            case	Addr_Turn_Active	: {
//							if(Re_MODBUS_Rag->MOD_Turn_Active==0)
//							{
//								Re_MODBUS_Rag->	MOD_Turn_Active=data;
//								if(Re_MODBUS_Rag->MOD_Turn_Active)
//								{
                //	Send_ActiveSem(Addr_Turn_Active &0xFF,(data>>8)&0xFF,data&0xFF); //ΪActive�������ź���
//								}
//							}
                break;
            }//	ת�ֶ���ִ��

            //////////////////////////////////////////////////////
            case	Addr_Reset_Distance: {
                Re_MODBUS_Rag->	MOD_Reset_Distance=data;
                if(Re_MODBUS_Rag->MOD_Reset_Distance)
                {
                    Re_MODBUS_Rag->	MOD_Reset_Distance=0;
                    CAN_Send(0x601,CAN_Data_Clear_Distance,8);
                    CAN_Send(0x602,CAN_Data_Clear_Distance,8);
                }
                break;
            }//	�������
            case	Addr_Motion_sys_end: {
                Re_MODBUS_Rag->	MOD_Motion_sys_end=data;
                break;
            }//	�˶�ϵͳ������β

            default : {
                return 1;   //��ַ����////�쳣��Addr_ERROR   �����������²������
            }
            }
            break;
        }
        case 2: { ///*	�˶����ò���	*/
            switch(add)
            {
            case	Addr_Limit_L_1                	: {
                Re_MODBUS_Rag->	MOD_Limit_L_1                	=data;    //	ת��1����λ
                break;
            }
            case	Addr_Limit_R_1                 	: {
                Re_MODBUS_Rag->	MOD_Limit_R_1                 	=data;    //	ת��1����λ
                break;
            }
            case	Addr_Turn1_0Encoder           	: {
                Re_MODBUS_Rag->	MOD_Turn1_0Encoder           	=data;    //	ת��1��0�ȱ���ֵ
                break;
            }
            case	Addr_Limit_L_2                	: {
                Re_MODBUS_Rag->	MOD_Limit_L_2                	=data;    //	ת��2����λ
                break;
            }
            case	Addr_Limit_R_2                	: {
                Re_MODBUS_Rag->	MOD_Limit_R_2                	=data;    //	ת��2����λ
                break;
            }
            case	Addr_Turn2_0Encoder           	: {
                Re_MODBUS_Rag->	MOD_Turn2_0Encoder           	=data;    //	ת��2��0�ȱ���ֵ
                break;
            }
            case	Addr_Limit_L_3                	: {
                Re_MODBUS_Rag->	MOD_Limit_L_3                	=data;    //	ת��3����λ
                break;
            }
            case	Addr_Limit_R_3                	: {
                Re_MODBUS_Rag->	MOD_Limit_R_3                	=data;    //	ת��3����λ
                break;
            }
            case	Addr_Turn3_0Encoder           	: {
                Re_MODBUS_Rag->	MOD_Turn3_0Encoder           	=data;    //	ת��3��0�ȱ���ֵ
                break;
            }
            case	Addr_Limit_L_4                	: {
                Re_MODBUS_Rag->	MOD_Limit_L_4                	=data;    //	ת��4����λ
                break;
            }
            case	Addr_Limit_R_4                	: {
                Re_MODBUS_Rag->	MOD_Limit_R_4                	=data;    //	ת��4����λ
                break;
            }
            case	Addr_Turn4_0Encoder           	: {
                Re_MODBUS_Rag->	MOD_Turn4_0Encoder           	=data;    //	ת��4��0�ȱ���ֵ
                break;
            }
            case	Addr_Sports_confi_end         	: {
                Re_MODBUS_Rag->	MOD_Sports_confi_end         	=data;    //	�˶����ò�����β
                break;
            }

            default : {
                return 1;   //��ַ����////�쳣��Addr_ERROR   �����������²������
            }
            }
            break;
        }
        case 3: { ///*	ϵͳ���ܲ���	*/
            switch(add)
            {
            case	Addr_USART_com_OUT_TIME        	: {
                Re_MODBUS_Rag->	MOD_USART_com_OUT_TIME        	=data;    //	ͨѶ���ڳ�ʱʱ��
                break;
            }
            case	Addr_Printf_En                 	: {

                Re_MODBUS_Rag->	MOD_Printf_En                  	=data;
                break;
            }//	����
            case	Addr_Retain_3                  	: {
                Re_MODBUS_Rag->	MOD_Retain_3                  	=data;    //	����
                break;
            }
            case	Addr_Retain_4                  	: {
                Re_MODBUS_Rag->	MOD_Retain_4                  	=data;    //	����
                break;
            }
            case	Addr_Systemrestart             	: {
                Re_MODBUS_Rag->	MOD_Systemrestart             	=data;    //	ϵͳ����
                break;
            }
            case	Addr_system_function_end       	: {
                Re_MODBUS_Rag->	MOD_system_function_end       	=data;    //	ϵͳ���ܲ�����β
                break;
            }

            default : {
                return 1;   //��ַ����////�쳣��Addr_ERROR   �����������²������
            }
            }
            break;
        }
        case 4: { ///*	���ò����洢�Ĵ���	*/
            switch(add)
            {
            case	Addr_FLASH_Motor_function_Update:	{
                Re_MODBUS_Rag->MOD_FLASH_Motor_function_Update=data;
                if(Re_MODBUS_Rag->MOD_FLASH_Motor_function_Update) {
                    MODBUS_FLASH_Write(Re_MODBUS_Rag,1);
                }
                break;
            }//	�����˶����ò���
            case	Addr_FLASH_Sys_function_Update:	{
                Re_MODBUS_Rag->	MOD_FLASH_Sys_function_Update=data;
                if(Re_MODBUS_Rag->MOD_FLASH_Sys_function_Update) {
                    MODBUS_FLASH_Write(Re_MODBUS_Rag,2);
                }
                break;
            }//	����ϵͳ���ܲ���
            case	Addr_FLASH_Motor_function_default:	{
                Re_MODBUS_Rag->	MOD_FLASH_Motor_function_default=data;
                break;
            }//	�˶�ϵͳ�����ָ�Ĭ��ֵ
            case	Addr_FLASH_Sys_function_default:	{
                Re_MODBUS_Rag->	MOD_FLASH_Sys_function_default=data;
                break;
            }//	ϵͳ���ܲ����ָ�Ĭ��ֵ
            case	Addr_FLASH_Read_Motor_function:	{
                Re_MODBUS_Rag->	MOD_FLASH_Read_Motor_function=data;
                if(Re_MODBUS_Rag->MOD_FLASH_Read_Motor_function) {
                    MODBUS_FLASH_Read(Re_MODBUS_Rag,1);
                }
                break;
            }//	��ȡ�˶�ϵͳ����
            case	Addr_FLASH_Read_Sys_function:	{
                Re_MODBUS_Rag->	MOD_FLASH_Read_Sys_function=data;
                if(Re_MODBUS_Rag->MOD_FLASH_Read_Sys_function) {
                    MODBUS_FLASH_Read(Re_MODBUS_Rag,2);
                }
                break;
            }//	��ȡϵͳ���ܲ���
            case	Addr_Conf_storage_end               	: {
                Re_MODBUS_Rag->	MOD_Conf_storage_end               	=data;    //	���ò����洢�Ĵ�����β
                break;
            }
            default : {
                return 1;   //��ַ����////�쳣��Addr_ERROR   �����������²������
            }
            }
            break;
        }
        default : {
            return 1;   //��ַ����////�쳣��Addr_ERROR   �����������²������
        }
        }
    }

    return 0;
}
////////////////////R/////////////////////////
/*
�ֽ�	 0   1      2          3         4        5         6      7
����	ADR	0x03	��ʼ�Ĵ�	��ʼ�Ĵ�	�Ĵ�����	�Ĵ�����	CRC ��	CRC ��
			          �����ֽ�	�����ֽ�	���ֽ�	  ���ֽ�	  �ֽ�	  �ֽ�
�ֽ� 	 0	    1	        2	         3	       4	      5        	6	       7
���� 	ADR 	0x06	"�Ĵ�����    "�Ĵ�����	"���ݸ�	 "���ݵ�	"CRC ��    "CRC ��
                   �ֽڵ�ַ"	 �ֽڵ�ַ"    �ֽ�"   �ֽ�"    ���ֽ�"	 ���ֽ�"

�ֽ�	  0	  1	       2	         3	         4	     5	       6	      7,8	   9,10  	N,N+1	   N+2  	N+3
����	ADR	0x10	��ʼ�Ĵ���	��ʼ�Ĵ���	�Ĵ�����	�Ĵ�����	�����ֽ�	�Ĵ���	�Ĵ���	�Ĵ���	CRC ��	CRC ��
			          ���ֽڵ�ַ	���ֽڵ�ַ	�����ֽ�	�����ֽ�	����	    ���� 1	���� 2	���� M	���ֽ�	���ֽ�*/
/*-------------------------------------------
//��������RBUFF_analysis     MODBUS-RTU���ս���
//������
1����������Ϣǰ�����޹��ַ����ǵ�ַ�ַ���ֱ�Ӻ��Ժ������Ϣ
2��֧�ֵĹ�������0X03,0X06,0X10
3����Ӧ�����������������ĺ����������ݼĴ�����ַ������
4����MODBUSԭ����ɸѡ��ַǰ��Ч�ַ�����ȥ������Ϊ��һ���ַ������ǵ�ַ����������Ӧ�������λ������CRCУ����
//������u8 *RBUFF,   ������Ϣ������
u16 len   ������Ϣ�ܳ���
//����ֵ��ϵͳ״̬
//���������б���
void MODBUS_Read_Rag(u16 add,u16 num)
void MODBUS_write_One_Rag(u16 add,int data)
---------------------------------------------
*/
unsigned char RBUFF_analysis(MODBUS_RagTypeDef *Re_MODBUS_Rag,u8 *RBUFF,u16 len)
{
    u8 add_site=0;//������λ��Ч��������RBUFF��λ��
    //u8 rag_num=0;//0x10�Ĵ�����λ
    u8 rag_num_MAX=0;//0x10��һ���������ټĴ���
    uint16_t CRC_data;
    if(RBUFF[0]!=Local_address)  //����жϻ������Ż�  �����豸��ַ�ֱ�Ϊ0x01��0x02   ����0x03���豸ͬʱ����
    {
        return 1;
    }
    if(!((RBUFF[add_site+1]==0x03)||(RBUFF[add_site+1]==0x06)||(RBUFF[add_site+1]==0x10)))//����0x03��0x06��0x10
    {
        MODBUS_BackError(0x03,FunctionCode_ERROR); //"�޿�ͷ"    ���豸ʱ�ᵼ�¶�����Ϣ
        return 1;
        //���ݳ�����  �쳣��0x01
    }
    //////��ʼУ��///////////
    if(RBUFF[add_site+1]==0x10)   //
    {
        CRC_data=MODBUS_Crc16(RBUFF+add_site,len-add_site-2);//����У����յ�����
        if((RBUFF[len-2]==(CRC_data>>8))&&(RBUFF[len-1]==(CRC_data&0xFF)))  //�����λ��У�� CRCУ��ɹ�
        {
            rag_num_MAX=((RBUFF[add_site+4]<<8)+RBUFF[add_site+5]);
            if(rag_num_MAX==RBUFF[add_site+6]/2)
            {
                MODBUS_0x10write_Rag(Re_MODBUS_Rag,((RBUFF[add_site+2]<<8)+RBUFF[add_site+3]),rag_num_MAX,RBUFF+7);
            }
            else
            {
                MODBUS_BackError(0x10,DATA_ERROR); //	�Ĵ��������������ֽ���������  //�쳣��0x03
                return 0x03;
            }
        }
        else //CRCУ��ʧ��
        {
            MODBUS_BackError(0x03,CRC_ERROR);  //У�����
            printf("У����� CRC_data=%X",CRC_data);//�쳣��0x04
            return 0x04;
        }
    }
    else//0x03  0x06
    {
        CRC_data=MODBUS_Crc16(RBUFF+add_site,len-add_site-2);
        if((RBUFF[len-2]==(CRC_data>>8))&&(RBUFF[len-1]==(CRC_data&0xFF)))  //�����λ��У�� CRCУ��ɹ�
        {
            if(RBUFF[add_site+1]==0x03)//0x03���Ĵ���
            {
                MODBUS_Read_Rag(Re_MODBUS_Rag,(RBUFF[add_site+2]<<8)+RBUFF[add_site+3],(RBUFF[add_site+4]<<8)+RBUFF[add_site+5]);
            }
            else//0x06д�����Ĵ���
            {
                MODBUS_0x06write_Rag(Re_MODBUS_Rag,(RBUFF[add_site+2]<<8)+RBUFF[add_site+3],(RBUFF[add_site+4]<<8)+RBUFF[add_site+5]);
            }
        }
        else //CRCУ��ʧ��
        {
            printf("У����� CRC_data=%X",CRC_data);//�쳣��0x04
            return 0x04;//�쳣��0x04
        }
    }
    //USART1_Send_Len(USART1_DMA_RBUFF+add_site,len-add_site);
    return 0;//��������
}

u8 BACK_BUFF[BUFF_LEN_MAX]= {0};
/*-------------------------------------------
//��������
//������
//������
MODBUS_RagTypeDef *Re_MODBUS_Rag,
u16 add
,u16 num
//����ֵ����
//���������б�����
---------------------------------------------*/
//num �Ĵ�����
u8 MODBUS_Read_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 num)
{
    u16 Addr_mov=0;
    uint16_t CRC_0x06;
    u32 re_data;
    for(Addr_mov=0; Addr_mov<num; Addr_mov++) //�ȼ���ַ�Ƿ���Ч����������Ч��ֱַ�ӱ���������������
    {
        if(!(MODBUS_ReadAddr_check(add+Addr_mov)))//0Ϊ�Ƿ���ַ
        {
            MODBUS_BackError(0x03,Addr_ERROR);//�Ƿ����ݵ�ַ
            return 1;
        }
    }

    BACK_BUFF[0]=Local_address;  //������ַ
    BACK_BUFF[1]=0x03;    //������
    BACK_BUFF[2]=num*2;   //�����ֽ�����

    for(Addr_mov=0; Addr_mov<num; Addr_mov++) //���е�ַ�Ϸ��������ȡ�����ݣ����ӵ�3(0��ʼ)�ֽڿ�ʼ����
    {
        re_data=MODBUS_Read_one_Rag(Re_MODBUS_Rag,add+Addr_mov);
        if(re_data>0xFFFF)     ///�����ֵ��������
        {
            if(re_data==0x00010000) {
                MODBUS_BackError(0x03,Addr_ERROR);    //��ַ����
                return 1;
            }
            else
            {
                if(re_data==0x00020000) {
                    MODBUS_BackError(0x03,ACT_ERROR);    //ִ�д���
                    return 2;
                }
                else   //�п����Ǹ���  ����Ȳ�����
                {
                    //printf("%X",re_data);
                    //MODBUS_BackError(0x03,re_data);return 3;//δ֪����
                }
            }
        }
        BACK_BUFF[Addr_mov*2+3]=re_data>>8;
        BACK_BUFF[Addr_mov*2+4]=re_data&0xFF;
    }
    CRC_0x06=MODBUS_Crc16(BACK_BUFF,num*2+3);//��Чλ��3+�Ĵ�����*2
    BACK_BUFF[Addr_mov*2+3]=CRC_0x06>>8;     //����λ�����У��  ��λ
    BACK_BUFF[Addr_mov*2+4]=CRC_0x06&0xFF;   //У��
    USART1_Send_Len(BACK_BUFF,num*2+5);//��3+�Ĵ�����*2+У��2λ
    return 0;
}
/*-------------------------------------------
//��������MODBUS_0x06write_Rag    ������0x06д����
//����������֤��ַ������ȷ���󷵻�
//������
MODBUS_RagTypeDef *Re_MODBUS_Rag,   //�˶������ṹ��
u16 add,д���ַ
u16 data д������
//����ֵ��
0���ɹ�
1����ַ����
//���������б�����
---------------------------------------------*/
u8 MODBUS_0x06write_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 data)
{
    u8 re_STA;
    if(!(MODBUS_WriteAddr_check(add)))
    {

        MODBUS_BackError(0x06,Addr_ERROR);//�Ƿ����ݵ�ַ
        return 1;
    }
    re_STA=MODBUS_write_One_Rag(Re_MODBUS_Rag,add,data);

    if(re_STA==0) {
        MODBUS_0x06_Back(add,data);   //����
    }
    else
    {
        if(re_STA==1) {
            MODBUS_BackError(0x06,Addr_ERROR);   //��ַ�쳣
        }
        else
        {
            if(re_STA==2) {
                MODBUS_BackError(0x06,ACT_ERROR);   //ִ���쳣
            }
            else {
                if(re_STA==3)
                {
                    MODBUS_BackError(0x06,DATA_ERROR);
                } else
                {   MODBUS_BackError(0x06,UNkwon_ERROR);
                }


            }
        }
    }
    return 0;
}
/*-------------------------------------------
//��������MODBUS_0x10write_Rag    ������0x10д����
//����������֤��ַ������ȷ���󷵻�
//������
MODBUS_RagTypeDef *Re_MODBUS_Rag,   //�˶������ṹ��
u16 add,д���ַ
u16 num,   д������
u16 *data д������
//����ֵ��
0���ɹ�
1����ַ����
//���������б�����
---------------------------------------------*/
u8 MODBUS_0x10write_Rag(MODBUS_RagTypeDef *Re_MODBUS_Rag,u16 add,u16 num,u8 *data)
{
    u16 Addr_mov=0;
    u8 re_STA;
    for(Addr_mov=0; Addr_mov<num; Addr_mov++) //�ȼ���ַ�Ƿ���Ч����������Ч��ֱַ�ӱ���������������
    {
        if(!(MODBUS_WriteAddr_check(add+Addr_mov)))//0Ϊ�Ƿ���ַ
        {
            MODBUS_BackError(0x10,Addr_ERROR);//�Ƿ����ݵ�ַ
            return 1;
        }
    }
    for(Addr_mov=0; Addr_mov<num; Addr_mov++)
    {
        re_STA=MODBUS_write_One_Rag(Re_MODBUS_Rag,add+Addr_mov,(data[Addr_mov*2]<<8)+data[Addr_mov*2+1]);
    }
    if(re_STA==0) {
        MODBUS_0x10_Back(add,num);   //����
    }
    else
    {
        if(re_STA==1) {
            MODBUS_BackError(0x10,Addr_ERROR);   //��ַ�쳣
        }
        else
        {
            if(re_STA==2) {
                MODBUS_BackError(0x10,ACT_ERROR);   //ִ���쳣
            }
            else {
                MODBUS_BackError(0x10,UNkwon_ERROR);
            }
        }
    }
    return 0;
}
/*-------------------------------------------
//��������MODBUS_ReadAdd_check  ������ַ�Ƿ���Ч ��Ч����0
//�������쳵��ַ�Ƿ��Ѷ��� �޶����򷵻�0
//������16 add  ��ַ
//����ֵ��
0����ַ��Ч
1��״̬��ַ
2���˶�״̬�Ĵ���
3���˶����ƼĴ���
//���������б�����
---------------------------------------------*/
int MODBUS_ReadAddr_check(u16 add)
{
    return MODBUS_Addr_check(ReadAddr,ReadAddrnum,add);
}
/*-------------------------------------------
//��������MODBUS_WriteAdd_check    ��д���ַ���
//���������д���ַ�Ƿ���ȷ
//������add   ��ַ
//����ֵ��
1:����
0:��ַ����д
//���������б�����
---------------------------------------------*/
int MODBUS_WriteAddr_check(u16 add)
{
    return MODBUS_Addr_check(WriteAddr,WriteAddrnum,add);
}
/*-------------------------------------------
//��������MODBUS_Addr_check    ��Ч��ַ���
//������  �Ӷ����ַ�����ڼ�������ַ�Ƿ���������
//������
short *Addr,   ��ַ����  ��������Ϊż��  ��ͬһ����������������
unsigned char num,   �������
short addr     ������ַ
//����ֵ��
0:����������
����:��������
//���������б�����
---------------------------------------------*/
int MODBUS_Addr_check(u16 *Addr,u8 num,u16 addr)
{
    for(; num>0; num--)
    {
        if((addr<=Addr[num*2-1])&&(addr>=Addr[num*2-2]))
        {
            return num;
        }
    }
    return 0;
}

////////////////////T//////////////////////////
/*-------------------------------------------
//��������MODBUSָ����󷵻�
//������   ֻ�ܷ���0x03��0x06���쳣   ���빦������쳣�룬����ֱ�ӷ���
//������
u8 Function,  //0x03��0x06
u8 error      //�쳣��
//����ֵ����
//���������б�����
---------------------------------------------*/
void MODBUS_BackError(u8 Function,u8 error)
{
    uint16_t CRC_0x03;
    BACK_BUFF[0]=Local_address;
    BACK_BUFF[1]=Function+0x80;
    BACK_BUFF[2]=error;
    CRC_0x03=MODBUS_Crc16(BACK_BUFF,3);
    BACK_BUFF[3]=CRC_0x03>>8;
    BACK_BUFF[4]=CRC_0x03&0xFF;
    USART1_Send_Len(BACK_BUFF,5);

}
/*-------------------------------------------
//��������MODBUS_0x06_Back    ������0x06��������
//������
//������
u16 add,��ַ
int data ����
//����ֵ����
//���������б���MODBUS_Crc16   CRCУ��
---------------------------------------------*/
void MODBUS_0x06_Back(u16 add,int data)
{
    uint16_t CRC_0x06;
    BACK_BUFF[0]=Local_address;
    BACK_BUFF[1]=0x06;
    BACK_BUFF[2]=add>>8;
    BACK_BUFF[3]=add&0xFF;
    BACK_BUFF[4]=data>>8;
    BACK_BUFF[5]=data&0xFF;
    CRC_0x06=MODBUS_Crc16(BACK_BUFF,6);
    BACK_BUFF[6]=CRC_0x06>>8;
    BACK_BUFF[7]=CRC_0x06&0xFF;
    USART1_Send_Len(BACK_BUFF,8);
}
/*-------------------------------------------
//�������� MODBUS_0x10_Back    ������0x10��������
//������
//������
u16 add,��ַ
u16 num �Ĵ�������
//����ֵ����
//���������б���MODBUS_Crc16   CRCУ��
---------------------------------------------*/
void MODBUS_0x10_Back(u16 add,u16 num)
{
    uint16_t CRC_0x06;
    BACK_BUFF[0]=Local_address;
    BACK_BUFF[1]=0x10;
    BACK_BUFF[2]=add>>8;
    BACK_BUFF[3]=add&0xFF;
    BACK_BUFF[4]=num>>8;
    BACK_BUFF[5]=num&0xFF;
    CRC_0x06=MODBUS_Crc16(BACK_BUFF,6);
    BACK_BUFF[6]=CRC_0x06>>8;
    BACK_BUFF[7]=CRC_0x06&0xFF;
    USART1_Send_Len(BACK_BUFF,8);
}

/*-------------------------------------------
//��������MODBUS_FLASH_Write
//������   д��FALSH
//������MODBUS_RagTypeDef *MODBUS_Raginit   ϵͳ���нṹ��
u8 com   ������
//����ֵ����
//���������б�����
---------------------------------------------*/
void MODBUS_FLASH_Write(MODBUS_RagTypeDef *MODBUS_Raginit,u8 com)
{
//    u16 Flash_Write[13]= {0};
//    switch(com)
//    {
//    case 0: {
//        break;   //����
//    }
//    case 1: { //�˶����ò���
//        Flash_Write[0]=0xAAAA;   //����У���Ƿ�ɹ�����FLASH����
//        Flash_Write[1]=MODBUS_Raginit->	MOD_Limit_L;	//	ת������λ	0x03,0x06,0x10	0-16383	����λ��������ֵ��Ĭ��12200
//        Flash_Write[2]=MODBUS_Raginit->	MOD_Limit_R;	//	ת������λ	0x03,0x06,0x10	0-16383	����λ��������ֵ��Ĭ��4200
//        Flash_Write[3]=MODBUS_Raginit->	MOD_SETEncoder_Middle;	//	0�ȱ���ֵ	0x03,0x06,0x10	0-16383	0�ȱ�������ֵ��Ĭ��8200
//        Flash_Write[4]=MODBUS_Raginit->	MOD_Retain_5;	//	����
//        Flash_Write[5	]=MODBUS_Raginit->	MOD_Retain_6;	//	����
//        Flash_Write[6	]=MODBUS_Raginit->	MOD_Retain_7;	//	����
//        Flash_Write[7	]=MODBUS_Raginit->	MOD_Retain_8;	//	����
//        Flash_Write[8	]=MODBUS_Raginit->	MOD_Retain_9;	//	����
//        Flash_Write[9	]=MODBUS_Raginit->	MOD_Retain_10;	//	����
//        Flash_Write[10]=MODBUS_Raginit->	MOD_PID_P;	//	����PID P����/1000	0x03,0x06,0x10	--	Ĭ��200=0.2
//        Flash_Write[11]=MODBUS_Raginit->	MOD_PID_I;	//	����PID I����/1000	0x03,0x06,0x10	--	Ĭ��500=0.5
//        Flash_Write[12]=MODBUS_Raginit->	MOD_PID_D;	//	����PID D����/1000	0x03,0x06,0x10	--	Ĭ��30=0.03
//        STMFLASH_Write(Addr_Flash_Move,Flash_Write,13);		//��ָ����ַ��ʼд��ָ�����ȵ�����
//        break;
//    }
//    case 2: { //ϵͳ���ܲ���
//        Flash_Write[0]=0xAAAA;   //����У���Ƿ�ɹ�����FLASH����
//        Flash_Write[1]=MODBUS_Raginit->	MOD_USART_com_OUT_TIME;	//	ͨѶ���ڳ�ʱʱ��	0x03,0x06,0x10	0~65535	*10ms
//        Flash_Write[2]=MODBUS_Raginit->	MOD_distance_clear_after_read;	//	�����������������	0x03,0x06,0x10	0/����	Ĭ����������  0:����  ����:������
//        Flash_Write[3]=MODBUS_Raginit->	MOD_Turn_er_Clear;	//	ת���ֱ����������	0x03,0x06,0x10	0/����	Ĭ����������  0:����  ����:������
//        Flash_Write[4]=MODBUS_Raginit->	MOD_USART_Move_OUT_TIME;	//	��ͣ��Чʱ��	0x03,0x06,0x10	0~65535	*10ms
//        STMFLASH_Write(Addr_Flash_Sys_com,Flash_Write,5);		//��ָ����ַ��ʼд��ָ�����ȵ�����
//        break;
//    }
//    default : {
//        break;
//    }
//    }

}
/*-------------------------------------------
//��������MODBUS_FLASH_Read
//������  ��ȡfalsh���ṹ��
//������  MODBUS_RagTypeDef *MODBUS_Raginit   ϵͳ���нṹ��
//����ֵ����
u8 com   ������
//���������б�����
---------------------------------------------*/
void MODBUS_FLASH_Read(MODBUS_RagTypeDef *MODBUS_Raginit,u8 com)
{
//    u16 Flash_Read[13]= {0};
////	STMFLASH_Read(0x08070000,Flash_Read,4);
//    switch(com)
//    {
//    case 0: { //ϵͳ����
//        break;
//    }
//    case 1: { //�˶����ò���
//        STMFLASH_Read(Addr_Flash_Move,Flash_Read,13);
//        if(Flash_Read[0]==0xAAAA)
//        {   MODBUS_Raginit->  MOD_FLASH_EN|=1<<1;//����˶����ò�������
//            MODBUS_Raginit->	MOD_Limit_L=Flash_Read[1];//	ת������λ	0x03,0x06,0x10	0-16383	����λ��������ֵ��Ĭ��12200
//            MODBUS_Raginit->	MOD_Limit_R=Flash_Read[2];//	ת������λ	0x03,0x06,0x10	0-16383	����λ��������ֵ��Ĭ��4200
//            MODBUS_Raginit->	MOD_SETEncoder_Middle=Flash_Read[3];//	0�ȱ���ֵ	0x03,0x06,0x10	0-16383	0�ȱ�������ֵ��Ĭ��8200
//            MODBUS_Raginit->	MOD_Retain_5=Flash_Read[4];//	����
//            MODBUS_Raginit->	MOD_Retain_6=Flash_Read[5];//	����
//            MODBUS_Raginit->	MOD_Retain_7=Flash_Read[6];//	����
//            MODBUS_Raginit->	MOD_Retain_8=Flash_Read[7];//	����
//            MODBUS_Raginit->	MOD_Retain_9=Flash_Read[8];//	����
//            MODBUS_Raginit->	MOD_Retain_10=Flash_Read[9];//	����
//            MODBUS_Raginit->	MOD_PID_P=Flash_Read[10];//	����PID P����/1000	0x03,0x06,0x10	--	Ĭ��200=0.2
//            MODBUS_Raginit->	MOD_PID_I=Flash_Read[11];//	����PID I����/1000	0x03,0x06,0x10	--	Ĭ��500=0.5
//            MODBUS_Raginit->	MOD_PID_D=Flash_Read[12];//	����PID D����/1000	0x03,0x06,0x10	--	Ĭ��30=0.03
//        }
//        else
//        {
//            MODBUS_Raginit->MOD_FLASH_EN&=~(1<<1);//����˶����ò�������ʧ��
//        }
//        break;
//    }
//    case 2: { //ϵͳ���ܲ���
//        STMFLASH_Read(Addr_Flash_Sys_com,Flash_Read,5);
//        if(Flash_Read[0]==0xAAAA)
//        {
//            MODBUS_Raginit->MOD_FLASH_EN|=1<<0;//���ϵͳ���ܲ�������
//            MODBUS_Raginit->MOD_USART_com_OUT_TIME=Flash_Read[1];//	ͨѶ���ڳ�ʱʱ��	0x03,0x06,0x10	0~65535	*10ms
//            MODBUS_Raginit->MOD_distance_clear_after_read=Flash_Read[2];//	�����������������	0x03,0x06,0x10	0/����	Ĭ����������  0:����  ����:������
//            MODBUS_Raginit->MOD_Turn_er_Clear=Flash_Read[3];//	ת���ֱ����������	0x03,0x06,0x10	0/����	Ĭ����������  0:����  ����:������
//            MODBUS_Raginit->MOD_USART_Move_OUT_TIME=Flash_Read[4];//	��ͣ��Чʱ��	0x03,0x06,0x10	0~65535	*10ms
//        }
//        else
//        {
//            MODBUS_Raginit->MOD_FLASH_EN&=~(1<<0);//���ϵͳ���ܲ�������ʧ��
//        }
//        break;
//    }
//    case 4: { //����
//        break;
//    }
//    default : {
//        break;
//    }
//    }
}
/*-------------------------------------------
//�������� MODBUS_FLASH_default
//������  �ָ�Ĭ��ֵ��д��FLASH
//������  MODBUS_RagTypeDef *MODBUS_Raginit   ϵͳ���нṹ��
//����ֵ����
u8 com   ������
//���������б�����
---------------------------------------------*/
void MODBUS_FLASH_default(MODBUS_RagTypeDef *MODBUS_Raginit,u8 com)
{
//    u16 Flash_Write[13]= {0};
////	STMFLASH_Read(0x08070000,Flash_Read,4);
//    switch(com)
//    {
//    case 1: { //�˶����ò���
//        Flash_Write[0]=0xAAAA;   //����У���Ƿ�ɹ�����FLASH����
//        Flash_Write[1]=Limit_L_default;	//	ת������λ	0x03,0x06,0x10	0-16383	����λ��������ֵ��Ĭ��12200
//        Flash_Write[2]=Limit_R_default;	//	ת������λ	0x03,0x06,0x10	0-16383	����λ��������ֵ��Ĭ��4200
//        Flash_Write[3]=Encoder_Middle_default;	//	0�ȱ���ֵ	0x03,0x06,0x10	0-16383	0�ȱ�������ֵ��Ĭ��8200
//        Flash_Write[4]=0x0000;	//	����
//        Flash_Write[5	]=0x0000;	//	����
//        Flash_Write[6	]=0x0000;	//	����
//        Flash_Write[7	]=0x0000;	//	����
//        Flash_Write[8	]=0x0000;	//	����
//        Flash_Write[9	]=0x0000;	//	����
//        Flash_Write[10]=PID_P_default;	//	����PID P����/1000	0x03,0x06,0x10	--	Ĭ��200=0.2
//        Flash_Write[11]=PID_I_default;	//	����PID I����/1000	0x03,0x06,0x10	--	Ĭ��500=0.5
//        Flash_Write[12]=PID_D_default;	//	����PID D����/1000	0x03,0x06,0x10	--	Ĭ��30=0.03
//        STMFLASH_Write(Addr_Flash_Move,Flash_Write,13);		//��ָ����ַ��ʼд��ָ�����ȵ�����
//        MODBUS_Raginit->MOD_FLASH_EN&=~(1<<1);//����˶����ò�������ʧ��
//        MODBUS_FLASH_Read(MODBUS_Raginit,1);
//        break;
//    }
//    case 2: { //ϵͳ���ܲ���
//        Flash_Write[0]=0xAAAA;   //����У���Ƿ�ɹ�����FLASH����
//        Flash_Write[1]=USART_com_OUT_TIME_default;	//	ͨѶ���ڳ�ʱʱ��	0x03,0x06,0x10	0~65535	*10ms
//        Flash_Write[2]=distance_clear_after_read_default;	//	�����������������	0x03,0x06,0x10	0/����	Ĭ����������  0:����  ����:������
//        Flash_Write[3]=MOD_Turn_er_Clear_default;	//	ת���ֱ����������	0x03,0x06,0x10	0/����	Ĭ����������  0:����  ����:������
//        Flash_Write[4]=Stop_Entime_default;	//	��ͣ��Чʱ��	0x03,0x06,0x10	0~65535	*10ms
//        STMFLASH_Write(Addr_Flash_Sys_com,Flash_Write,5);		//��ָ����ַ��ʼд��ָ�����ȵ�����
//        MODBUS_Raginit->MOD_FLASH_EN&=~(1<<0);//���ϵͳ���ܲ�������ʧ��
//        MODBUS_FLASH_Read(MODBUS_Raginit,2);
//        break;
//    }
//    default : {
//        break;
//    }
//    }
}

////////////////////CRC////////////////////////////
//////////////////////CRC_16У��/////////////////////////////////////////
const uint8_t MODBUS_auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
const uint8_t MODBUS_auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;
uint16_t MODBUS_Crc16(uint8_t *ptr, uint16_t len)   //��λ��ǰ��
{
    uint8_t uchCRCHi = 0xFF ; /* ��CRC��ʼ�� */
    uint8_t uchCRCLo = 0xFF ; /* ��CRC��ʼ�� */
    unsigned long uIndex ; /* CRCѭ���е����� */
    while (len--) /* ������Ϣ������ */
    {
        uIndex = uchCRCHi ^ *ptr++ ; /* ����CRC */
        uchCRCHi = uchCRCLo ^ MODBUS_auchCRCHi[uIndex] ;
        uchCRCLo = MODBUS_auchCRCLo[uIndex] ;
    }
    return (uchCRCHi << 8 | uchCRCLo) ;
    //	BACK_BUFF[3]=CRC_0x03>>8;
    //  BACK_BUFF[4]=CRC_0x03&0xFF;
}


