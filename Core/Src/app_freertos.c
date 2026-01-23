/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "adc.h"
#include "tim.h"
#include "dac.h"
#include "gpio.h"
#include "fdcan.h"
#include "tmc2226_step_bsp.h"
#include "eeprom_bsp.h"
#include "tec_control_bsp.h"
#include "CAN_modbusRTU_bsp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//motorEvent01Handle  
#define EVENTS_MOTOR_ZERO_BIT0  			    0x01//0校准
#define EVENTS_MOTOR_IDLE_BIT1  					0x01<<1 //电机空闲 
#define EVENTS_MOTOR_OK_ALL_BITS_MASK        	(EVENTS_MOTOR_ZERO_BIT0|EVENTS_MOTOR_IDLE_BIT1)
//laserEvent02Handle  
#define EVENTS_LASER_POSITON_BIT0 			 0x01
#define EVENTS_LASER_TEMPRATUR_BIT1  		 0x01<<1 
#define EVENTS_LASER_VOLTAGE_BIT2  			 0x01<<2
#define EVENTS_LASER_OK_ALL_BITS_MASK      (EVENTS_LASER_POSITON_BIT0|EVENTS_LASER_TEMPRATUR_BIT1|EVENTS_LASER_VOLTAGE_BIT2)

//deviceErrorEvent03  
#define EVENTS_DEV_ERR_CAN_HEART_BIT0 			       0x01
#define EVENTS_DEV_ERR_TEC_CTR_BIT1  		           0x01<<1 
#define EVENTS_DEV_ERR_MOTOR_TMC_BIT2  		         0x01<<2
#define EVENTS_DEV_ERR_LASER_TEMPRATURE_BIT3  		 0x01<<3
#define EVENTS_DEV_ERR_MOTOR_TEMPRATURE_BIT4 		   0x01<<4
#define EVENTS_DEV_ERR_POSITON_BIT5 			         0x01<<5
#define EVENTS_DEV_ERR_VBUS_BIT6 			             0x01<<6
#define EVENTS_DEV_ERR_IBUS_BIT7			             0x01<<7
#define EVENTS_DEV_ERR_ALL_BITS_MASK  (EVENTS_DEV_ERR_CAN_HEART_BIT0|EVENTS_DEV_ERR_TEC_CTR_BIT1|EVENTS_DEV_ERR_MOTOR_TMC_BIT2\
                                      |EVENTS_DEV_ERR_LASER_TEMPRATURE_BIT3|EVENTS_DEV_ERR_MOTOR_TEMPRATURE_BIT4 \
                                      |EVENTS_DEV_ERR_POSITON_BIT5|EVENTS_DEV_ERR_VBUS_BIT6|EVENTS_DEV_ERR_IBUS_BIT7)
       
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
LASER_980_STATUS laser_980_sta;
U_SYS_CONFIG_PARAM u_sys_param;
U_SYS_CONFIG_PARAM u_sys_default_param;
LASER_CTR_PARAM  laser_ctr_param;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal2,
  .stack_size = 256 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .priority = (osPriority_t) osPriorityNormal2,
  .stack_size = 128 * 4
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for motorPositonQueue01 */
osMessageQueueId_t motorPositonQueue01Handle;
uint8_t positonQueue01Buffer[ 2 * sizeof( uint16_t ) ];
osStaticMessageQDef_t positonQueue01ControlBlock;
const osMessageQueueAttr_t motorPositonQueue01_attributes = {
  .name = "motorPositonQueue01",
  .cb_mem = &positonQueue01ControlBlock,
  .cb_size = sizeof(positonQueue01ControlBlock),
  .mq_mem = &positonQueue01Buffer,
  .mq_size = sizeof(positonQueue01Buffer)
};
/* Definitions for tecBinarySem01 */
osSemaphoreId_t tecBinarySem01Handle;
const osSemaphoreAttr_t tecBinarySem01_attributes = {
  .name = "tecBinarySem01"
};
/* Definitions for motorEvent01 */
osEventFlagsId_t motorEvent01Handle;
const osEventFlagsAttr_t motorEvent01_attributes = {
  .name = "motorEvent01"
};
/* Definitions for laserEvent02 */
osEventFlagsId_t laserEvent02Handle;
const osEventFlagsAttr_t laserEvent02_attributes = {
  .name = "laserEvent02"
};
/* Definitions for deviceErrorEvent03 */
osEventFlagsId_t deviceErrorEvent03Handle;
const osEventFlagsAttr_t deviceErrorEvent03_attributes = {
  .name = "deviceErrorEvent03"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void app_tec_ctr_semo(void);
void app_tec_auto_manage(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void motorTask02(void *argument);
void CANopenTask03(void *argument);
void laserWorkTask04(void *argument);
void laserProhotTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of tecBinarySem01 */
  tecBinarySem01Handle = osSemaphoreNew(1, 1, &tecBinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of motorPositonQueue01 */
  motorPositonQueue01Handle = osMessageQueueNew (2, sizeof(uint16_t), &motorPositonQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(motorTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(CANopenTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(laserWorkTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(laserProhotTask05, NULL, &myTask05_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of motorEvent01 */
  motorEvent01Handle = osEventFlagsNew(&motorEvent01_attributes);

  /* creation of laserEvent02 */
  laserEvent02Handle = osEventFlagsNew(&laserEvent02_attributes);

  /* creation of deviceErrorEvent03 */
  deviceErrorEvent03Handle = osEventFlagsNew(&deviceErrorEvent03_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  app_load_power_switch( ENABLE);
  app_start_multi_channel_adc();
  float local_disp; 
  for(;;)
  { 
    app_get_adc_value(AD1_CH1_IBUS,&local_disp);
    DEBUG_PRINTF("AD:iBus=%.1fmA",local_disp);    
    app_get_adc_value(AD1_CH2_VBUS,&local_disp);
    DEBUG_PRINTF(" vBus=%.2fV",local_disp*0.001);    
    app_get_adc_value(AD1_CH3_I_TEC,&local_disp);
    DEBUG_PRINTF(" i_TEC=%.1f",local_disp);   
    app_get_adc_value(AD2_CH3_NTC_MOTOR_TEMPRATURE,&local_disp);
    DEBUG_PRINTF(" motor_T=%.2f℃",local_disp);  
    app_get_adc_value(AD2_CH4_TEC_TEMPRATURE,&local_disp);
    DEBUG_PRINTF(" tec_T=%.1f℃",local_disp);     
    app_get_adc_value(AD3_CH1_ENERGE_FEEDBACK,&local_disp);
    DEBUG_PRINTF(" energe=%.1f\r\n",local_disp);
    DEBUG_PRINTF("POSITON:pos=%dμm\r\n",laser_980_sta.real_motor_positon_um);
    DEBUG_PRINTF("canpac=%d\r\n",can_count);    
    osDelay(1000); 
    HAL_GPIO_TogglePin(MCU_LED_CTR_OUT_GPIO_Port,MCU_LED_CTR_OUT_Pin);    
    app_tec_auto_manage();
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_motorTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motorTask02 */
void motorTask02(void *argument)
{
  /* USER CODE BEGIN motorTask02 */
  /* Infinite loop */
  tmc2226_init(); 
  unsigned int local_system=0;
  unsigned short int targetPosition=0;
  osDelay(1000);
  for(;;)
  {
    uint32_t motor_events =osEventFlagsGet(motorEvent01Handle); 
    if(motor_events==EVENTS_MOTOR_OK_ALL_BITS_MASK)  
    {
      laser_980_sta.real_motor_positon_um=app_get_motor_real_position();
      if(osMessageQueueGet(motorPositonQueue01Handle, &targetPosition, NULL, 50)==osOK)
      {   
        if(laser_980_sta.real_motor_positon_um!=targetPosition)
        {
          if(laser_980_sta.real_motor_positon_um>targetPosition)
          {
            app_motor_slide_position(MOTOR_DIR_REVERSE,targetPosition,3);
          }
          else app_motor_slide_position(MOTOR_DIR_FORWARD,targetPosition,3);
        }
      }      
    }
    else 
    {
      if((motor_events&EVENTS_MOTOR_IDLE_BIT1)==EVENTS_MOTOR_IDLE_BIT1) 
      {
        if(HAL_GPIO_ReadPin(MOTOR_ZERO_CHECK_EXTI9_5_IN_GPIO_Port,MOTOR_ZERO_CHECK_EXTI9_5_IN_Pin)==GPIO_PIN_RESET)
        {    
          DEBUG_PRINTF(" reverse zero\r\n");  
          app_motor_slide_position(MOTOR_DIR_ZERO,MOTOR_MAX_UM,3);
        }
        else   
        {  
          DEBUG_PRINTF("prepare find zero \r\n"); 
          __HAL_TIM_SET_COUNTER(&htim3,1);
          app_motor_slide_position(MOTOR_DIR_FORWARD,5000,3);
        }
      }  
      else 
      {
        if(osMessageQueueGet(motorPositonQueue01Handle, &targetPosition, NULL, 0)==osOK)
        {          
          DEBUG_PRINTF("motor is moving\r\n"); 
        }
      }    
    }
    osDelay(5);
  }
  /* USER CODE END motorTask02 */
}

/* USER CODE BEGIN Header_CANopenTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CANopenTask03 */
void CANopenTask03(void *argument)
{
  /* USER CODE BEGIN CANopenTask03 */
  /* Infinite loop */
  uint8_t buff[8];
   uint32_t Identifier;
    uint16_t len;
  for(;;)
  {   
    if(FDCAN1_Receive_Msg(buff, &Identifier,&len))
    {  
      uint8_t packageType = Identifier-CAN_SLAVE_ID;  
      CAN_receivePackageHandle(buff,packageType);      
    }
    osDelay(1);
  }
  /* USER CODE END CANopenTask03 */
}

/* USER CODE BEGIN Header_laserWorkTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_laserWorkTask04 */
void laserWorkTask04(void *argument)
{
  /* USER CODE BEGIN laserWorkTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END laserWorkTask04 */
}

/* USER CODE BEGIN Header_laserProhotTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_laserProhotTask05 */
void laserProhotTask05(void *argument)
{
  /* USER CODE BEGIN laserProhotTask05 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END laserProhotTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
  if(GPIO_Pin==MOTOR_ZERO_CHECK_EXTI9_5_IN_Pin)
  {
    DEBUG_PRINTF("HR zero \r\n");      
  }   
  if(GPIO_Pin==ECODER_Z_EXT2I_IN_Pin)
  {
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))//down
    {
      if(HAL_GPIO_ReadPin(MOTOR_ZERO_CHECK_EXTI9_5_IN_GPIO_Port,MOTOR_ZERO_CHECK_EXTI9_5_IN_Pin)==GPIO_PIN_SET)
      {
        tmc2226_stop(); 
        osEventFlagsSet(motorEvent01Handle,EVENTS_MOTOR_ZERO_BIT0);  
        DEBUG_PRINTF("index zero encode=%d \r\n",htim3.Instance->CNT);  
        __HAL_TIM_SET_COUNTER(&htim3,0);
      }      
    }
  }
}
/************************************************************************//**
  * @brief  relese
  * @param   
  * @note    
  * @retval None
  ****************************************************************************/
void app_tec_ctr_semo(void)
{
  osSemaphoreRelease(tecBinarySem01Handle);
}
 /************************************************************************//**
  * @brief  app_motor_run_sta
  * @param  runflag:0stop;1move
  * @note    
  * @retval None
  ****************************************************************************/
 void app_motor_run_sta(unsigned char runFlag)
 {
  if(runFlag==0)  osEventFlagsSet(motorEvent01Handle,EVENTS_MOTOR_IDLE_BIT1);
  else  osEventFlagsClear(motorEvent01Handle,EVENTS_MOTOR_IDLE_BIT1);
 }
/************************************************************************//**
* @brief  app_tec_auto_manage
* @param  
* @note    
* @retval None
****************************************************************************/
void app_tec_auto_manage(void)
{
  int volta;    
  if(laser_980_sta.real_laser_temprature+0.2<u_sys_param.sys_config_param.cool_temprature_target*0.1)
  {
    volta = 100;//volta=-100;
    if(osSemaphoreAcquire(tecBinarySem01Handle,0)==osOK)
    {
      tec_start(volta|0x80,500);
    } 
    else DEBUG_PRINTF("the tec is running\r\n");        
  }
  else if(laser_980_sta.real_laser_temprature>0.2+u_sys_param.sys_config_param.cool_temprature_target*0.1)
  {  
    volta = 100;
    if(osSemaphoreAcquire(tecBinarySem01Handle,0)==osOK)
    {
      tec_start(volta,500);
    } 
    else DEBUG_PRINTF("the tec is running\r\n");
  }  
}
/* USER CODE END Application */

