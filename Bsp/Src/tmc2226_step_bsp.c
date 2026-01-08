/**
 * @file tmc2226_step_bsp.c
 *
 * @brief 
 * @warning This software tmc2226 Drivers
 * 
 *
 */
#include "tmc2226_step_bsp.h"
#include "main.h"
#include "usart.h"
#include "tim.h"


#include "string.h"
#include "stdio.h"
#ifndef TMC_USART_USED
/******************LSB**************/
#define TMC_USART_USED 
#define TMC_USART_FRAME_HEAD  0x05  
#define TMC_SLAVE_ADDR        0x00   // MS1(bit0) MS2(bit1)(0x00,0x01,0x02 0x03)(0~3)
#define TMC_USART_WRITE_MASK  0x80
#define TMC_USART_READ_MASK   0x00

//GENERAL CONFIGURATION REGISTERS (0X00��0X0F)
#define TMC_USART_GCONF_REG         0x00
#define TMC_USART_GSTAT_REG         0x01
#define TMC_USART_IFCNT_REG         0x02
#define TMC_USART_SLAVECONF_REG     0x03
#define TMC_USART_OTP_PROG_REG      0x04
#define TMC_USART_OTP_READ_REG      0x05
#define TMC_USART_IOIN_REG          0x06
#define TMC_USART_FACTORY_CONF_REG  0x07
//VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0X10��0X1F)
#define TMC_USART_FIHOLD_IRUN_REG   0x10
#define TMC_USART_TPOWER DOWN_REG   0x11
#define TMC_USART_TSTEP_REG         0x12
#define TMC_USART_TPWMTHRS_REG      0x13
#define TMC_USART_VACTUAL_REG       0x14
//COOLSTEP AND STALLGUARDCONTROL REGISTER SET (0X14,0X40��0X42)
#define TMC_USART_TCOOLTHRS_REG     0x14
#define TMC_USART_SGTHRS_REG        0x40
#define TMC_USART_SG_RESULT_REG     0x41
#define TMC_USART_COOLCONF_REG      0x42
//MICROSTEPPING CONTROL REGISTER SET (0X60��0X6B)
#define TMC_USART_MSCNT_REG         0x6A
#define TMC_USART_MSCURACT_REG      0x6B
//DRIVER REGISTER SET (0X6C��0X7F)
#define TMC_USART_HOPCONF_REG       0x6C
#define TMC_USART_DRV_STATUS_REG    0x6F
#define TMC_USART_PWMCONF_REG       0x70
#define TMC_USART_PWM_SCALE_REG     0x71
#define TMC_USART_PWM_AUTO_REG      0x72

typedef struct {
  unsigned char frameHead;
  unsigned char SlaveAddr;
  unsigned char reg;  
  unsigned char data[4];
  unsigned char crc;
}TMC_USART_FRAME;

#endif

#define TMC_ONE_CIRCLE_STEPS  200

typedef struct {
  unsigned char run;//0，stop;1 low;2,mid,3high;
  unsigned char dir;  
  unsigned char errStatus;
  unsigned char rdb_speed;
  unsigned int  pulse_count;//单步计数
}TMC_INFO;

static TMC_INFO tmc2226_rdb_info;
static unsigned char tmc_usart_config[8]={0};
static unsigned short int tmc_speed_list6[6]={100,150,200,250,300,350};//rpm  ,50ml/Min ~200ml/min


static void app_tmc2226_speed_set(unsigned char spdLevel);

//extern UART_HandleTypeDef huart10;;
//usart change speed
//??? = ?8 +?2 + ?1 + ?0
/**
  * @brief swuart_calcCRC
  * 
  * @param  unsigned char* datagram, unsigned char datagramLength
  * @note   crc 
  * @retval None
  */
 void swuart_calcCRC(unsigned char* datagram, unsigned char datagramLength)
 {
  int i,j;
  unsigned char* crc = datagram + (datagramLength-1); // CRC located in last byte of message
  unsigned char currentByte;
  *crc = 0;
  for (i=0; i<(datagramLength-1); i++) 
  { // Execute for all bytes of a message
    currentByte = datagram[i]; // Retrieve a byte to be sent from Array
    for (j=0; j<8; j++) 
    {
      if ((*crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
      {
        *crc = (*crc << 1) ^ 0x07;
      }
      else
      {
        *crc = (*crc << 1);
      }
      currentByte = currentByte >> 1;
    } // for CRC bit
  }   // for message byte  
 }
/**
  * @brief  tmc2226_en
  * @param  void
  * @note   蠕动泵使能
  * @retval None
  */
  void tmc2226_en ( unsigned  char en )
  {
    tmc2226_rdb_info.run=en;
    if(en==0)  
    {     
      HAL_GPIO_WritePin ( TMC2226_CTR_EN_OUT_GPIO_Port , TMC2226_CTR_EN_OUT_Pin , GPIO_PIN_SET );    
    } 
    else 
    {
      HAL_GPIO_WritePin ( TMC2226_CTR_EN_OUT_GPIO_Port , TMC2226_CTR_EN_OUT_Pin , GPIO_PIN_RESET );   
    }  
  }
 /**
  * @brief  void tmc2226_param_default(void)
  * @param  void
  * @note   步进默认参数
  * @retval None
  */
  void tmc2226_param_default ( void )
  {
    tmc2226_rdb_info.run = 0;
    tmc2226_rdb_info.dir = 0;
    tmc2226_rdb_info.errStatus = 0;
    tmc2226_rdb_info.rdb_speed = 0;
    if(tmc2226_rdb_info.rdb_speed < 5) tmc2226_rdb_info.rdb_speed = 5;
    if(tmc2226_rdb_info.rdb_speed >35) tmc2226_rdb_info.rdb_speed = 35;
  }
/**
  * @brief tmc2226_dir
  * @param  void
  * @note  电机方向
  * @retval None
  */
 void tmc2226_dir(unsigned  char dir)
 {
    if(dir==0)  HAL_GPIO_WritePin(TMC2226_DIR_OUT_GPIO_Port, TMC2226_DIR_OUT_Pin, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(TMC2226_DIR_OUT_GPIO_Port, TMC2226_DIR_OUT_Pin, GPIO_PIN_SET);
 } 
/**
  * @brief temc2226_init
  * @param  void
  * @note   步进电机驱动初始化
  * @retval None
  */
 void tmc2226_init(void)
 {   
    tmc2226_param_default();     
    tmc2226_stop();       
 }
 /**
  * @brief tmc2226_step_pwm
  * @param  void
  * @note   PWM控制step   1~20K
  * @retval None
  *        //queit speed   10K~5.5k
  */
 void tmc2226_step_pwm_set(unsigned  int speed)
 { 
  if(speed!=0)
  {    
		unsigned int timeUs;
		unsigned short int period;
		//check freq,timer8 1M clock freq		
    //period=1000000/50000;
    if(speed==3)//high speed  //10k
    {
      period = 100;
    }
    else if(speed==2)//mid speed 8k
    {
      period = 125;
    }
    else //if(speed==1)//5.5k
    {
      period=180;
    } 
		__HAL_TIM_SetAutoreload(&htim8,period-1);//freq =10k
		//duty 1%  100%; 0% close	
		timeUs=period /2;	//duty 50%
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,timeUs-1);	
  }
  else
  {
    HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
    HAL_TIM_Base_Stop_IT(&htim4);
  }	
 }
/**
  * @brief app_steps_pulse
  * @param  void
  * @note   1 circle: 1 index  pulse -> 200 steps
  * @retval None
  */
void app_steps_pulse(unsigned int steps)
 { 
  static unsigned int timeout;    
  if(steps==0)
  {
    if(tmc2226_rdb_info.run!=0) tmc2226_stop(); 
  }
  else 
  {      
    if(tmc2226_rdb_info.run!=0) 
    {
      tmc2226_rdb_info.pulse_count++;      
      if(steps<CONTINUOUS_STEPS_COUNT)
      {
        if(tmc2226_rdb_info.pulse_count>steps)
        {          
          tmc2226_stop(); 
        }
      }        
    }  
  }     
 } 
 /**
  * @brief tmc2226_start
  * @param  unsigned char dir,unsigned short int speed,unsigned  int steps
  * @note   启动步进电机 
  *  10mm ->10 circle-> 1000 ctr pulse=4000 micro steps -> 40000 encoder (phseA and paseB)-> only phase A pulse 10000
  *  1mm ->1 circle-> 100 ctr pulse = 400 micro steps -> 4000 encoder (phseA and paseB)-> only phase A pulse 1000
  *  0.01mm ->3.6° -> 1 ctr pulse = 4 micro steps -> 40 encoder (phseA and paseB) -> only phase A pulse  10
  *  0.001mm ->0.36°-> 0.1 ctr pulse = 0.4 micro steps -> 4 encoder (phseA and paseB) -> only phase A pulse  1
  * 
  *   MOTOR_DIR_FORWARD:encoder count up;
  *   MOTOR_DIR_REVERSE:encoder count down;
  * @retval None
  */
void tmc2226_start(unsigned char dir,unsigned short int spdLevel,unsigned  int steps)
{
	//check status ,error status	
  unsigned   int targetUm=0;
	if(spdLevel==0||steps<2)
	{
		tmc2226_stop();
	}
	else 
	{      
		tmc2226_dir(dir);    
    app_tmc2226_speed_set(spdLevel);
    tmc2226_step_pwm_set(tmc2226_rdb_info.rdb_speed);
    if(steps < MAX_TRIP_STEPS_COUNT+1) // until run
    {      
      __HAL_TIM_SET_COUNTER(&htim4,0); 
      if(dir==MOTOR_DIR_ZERO)  __HAL_TIM_SetAutoreload(&htim4,MAX_TRIP_STEPS_COUNT);         
      else __HAL_TIM_SetAutoreload(&htim4,steps+2);   //0.005mm   
      HAL_TIM_Base_Start_IT(&htim4);             
    }
    else HAL_TIM_Base_Stop_IT(&htim4); //until
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);         
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
    tmc2226_en(1); 
    
	}
}
  /**
* @brief 停止步进电机
  * @param  void
  * @note   停止步进电机
  * @retval 本次运行步数
  */
void  tmc2226_stop(void)
{  	
  HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);   
  tmc2226_en(0);
  __HAL_TIM_SET_COUNTER(&htim4,0);  
  HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_ALL); 
} 
 /************************************************************************//**
  * @brief 设置步进速度等级
  * @param spdLevel: 0,1,2,3
  * @note   0关闭  3最快   
  * @retval 无
  *****************************************************************************/
static void app_tmc2226_speed_set(unsigned char spdLevel)
{
  if(spdLevel==0)
  {
    tmc2226_stop();
  }
  else 
  {      
    if(spdLevel==1)
    {	        
      tmc2226_rdb_info.rdb_speed = 1;
    }
    else if(spdLevel==2)
    { 
      tmc2226_rdb_info.rdb_speed = 2;
    }
    else //if(spdLevel==3)
    {	
      tmc2226_rdb_info.rdb_speed = 3;      
    }
  }
} 
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM3)
  {
    tmc2226_stop(); 
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))//DOWN
    {    
     DEBUG_PRINTF("reverse OC position=%d \r\n",htim3.Instance->CNT);
    }
    else
    {
      DEBUG_PRINTF("arrive OC=%d \r\n",htim3.Instance->CNT);
    }
  }
}
 /************************************************************************//**
  * @brief  encoder_dir_count_config(unsigned char dir)
  * @param   
  * @note    
  * @retval None
  ****************************************************************************/
static void encoder_dir_count_config(unsigned char dir,unsigned  int encoderCount)
{
  unsigned  int target_code=__HAL_TIM_GET_COUNTER(&htim3); 
  if(dir==MOTOR_DIR_FORWARD)//encoder up
  {   
    if(encoderCount>ENCODER_MAX_COUNT)//32mm
    {
      target_code=ENCODER_MAX_COUNT;
    }
    else target_code=encoderCount;
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,target_code-1);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC3); 
  }
  else if(dir==MOTOR_DIR_REVERSE)// encoder DOWN
  {  
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,encoderCount-1);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC3); 
  }
  else //if(dir==MOTOR_DIR_ZERO)// cali zero
  {      
    __HAL_TIM_SET_COUNTER(&htim3,ENCODER_MAX_COUNT); 
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC3); 
  }  
}
/************************************************************************//**
  * @brief app_tmc_um_start 
  * @param   
  * @note   distancUm  1μm  -> 1circle 1mm
  * @retval None
  ****************************************************************************/
void app_tmc_um_start(unsigned char dir, unsigned  int distanceUm,unsigned char speed)
{  
  unsigned  int steps=0; 
  encoder_dir_count_config( dir,distanceUm<<1); //distanceUm*2
  if(dir==MOTOR_DIR_ZERO)
  {
    tmc2226_start(dir,speed,MAX_TRIP_STEPS_COUNT);  
  }
  else
  {
    unsigned  int history_positionUm =(__HAL_TIM_GET_COUNTER(&htim3)+1)>>1;//μm
    if(history_positionUm!=distanceUm)
    {
      if(history_positionUm>distanceUm) steps=history_positionUm-distanceUm;
      else steps=distanceUm-history_positionUm;
      if(steps<10) steps=2;
      else  steps = (steps/10);//1 step-> 10μm 
      tmc2226_start(dir,speed,steps);  
    }  
  }
 
}