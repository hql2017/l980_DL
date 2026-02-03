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
#include "common_function.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//motorEvent01Handle  
#define EVENTS_MOTOR_ZERO_BIT0  			    0x01//0校准
#define EVENTS_MOTOR_IDLE_BIT1  					0x01<<1 //电机空闲 
#define EVENTS_MOTOR_TEMPRATURE_BIT2 					0x01<<2 //电机温度正常
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

//tecEvent04Handle  
#define EVENTS_TEC_STA_IDLE_BIT0 			 0x01 //1空闲，0运行
#define EVENTS_TEC_ERR_BIT1 			 0x01<<1//1无故障
#define EVENTS_TEC_OK_ALL_BITS_MASK  (EVENTS_TEC_STA_IDLE_BIT0|EVENTS_TEC_ERR_BIT1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
HSM_DL_STATUS hsm_dl_sta;
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
/* Definitions for laserWorkTimer01 */
osTimerId_t laserWorkTimer01Handle;
osStaticTimerDef_t laserWorkTimer01ControlBlock;
const osTimerAttr_t laserWorkTimer01_attributes = {
  .name = "laserWorkTimer01",
  .cb_mem = &laserWorkTimer01ControlBlock,
  .cb_size = sizeof(laserWorkTimer01ControlBlock),
};
/* Definitions for tecRunTimer02 */
osTimerId_t tecRunTimer02Handle;
const osTimerAttr_t tecRunTimer02_attributes = {
  .name = "tecRunTimer02"
};
/* Definitions for motorMutex01 */
osMutexId_t motorMutex01Handle;
const osMutexAttr_t motorMutex01_attributes = {
  .name = "motorMutex01"
};
/* Definitions for prohotBinarySem02 */
osSemaphoreId_t prohotBinarySem02Handle;
const osSemaphoreAttr_t prohotBinarySem02_attributes = {
  .name = "prohotBinarySem02"
};
/* Definitions for laserCloseBinarySem03 */
osSemaphoreId_t laserCloseBinarySem03Handle;
const osSemaphoreAttr_t laserCloseBinarySem03_attributes = {
  .name = "laserCloseBinarySem03"
};
/* Definitions for poweroffBinarySem04 */
osSemaphoreId_t poweroffBinarySem04Handle;
const osSemaphoreAttr_t poweroffBinarySem04_attributes = {
  .name = "poweroffBinarySem04"
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
/* Definitions for tecEvent04 */
osEventFlagsId_t tecEvent04Handle;
const osEventFlagsAttr_t tecEvent04_attributes = {
  .name = "tecEvent04"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void app_tec_ctr_semo(void);
void app_tec_auto_manage(void);
unsigned short int  app_laser_980_energe_to_voltage(unsigned short int energe);
void app_motor_err_handle(osStatus_t errSta);
void app_laser_prohot_semo(void);
void app_sys_param_load(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void motorTask02(void *argument);
void CANopenTask03(void *argument);
void laserWorkTask04(void *argument);
void laserProhotTask05(void *argument);
void laserWorkTimerCallback01(void *argument);
void tecRunCallback02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */

   #ifdef IWDG_USED
   HAL_IWDG_Refresh(&hiwdg);
   #endif 
}
/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of motorMutex01 */
  motorMutex01Handle = osMutexNew(&motorMutex01_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of prohotBinarySem02 */
  prohotBinarySem02Handle = osSemaphoreNew(1, 0, &prohotBinarySem02_attributes);

  /* creation of laserCloseBinarySem03 */
  laserCloseBinarySem03Handle = osSemaphoreNew(1, 0, &laserCloseBinarySem03_attributes);

  /* creation of poweroffBinarySem04 */
  poweroffBinarySem04Handle = osSemaphoreNew(1, 0, &poweroffBinarySem04_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of laserWorkTimer01 */
  laserWorkTimer01Handle = osTimerNew(laserWorkTimerCallback01, osTimerOnce, NULL, &laserWorkTimer01_attributes);

  /* creation of tecRunTimer02 */
  tecRunTimer02Handle = osTimerNew(tecRunCallback02, osTimerOnce, NULL, &tecRunTimer02_attributes);

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

  /* creation of motorEvent01 */
  motorEvent01Handle = osEventFlagsNew(&motorEvent01_attributes);

  /* creation of laserEvent02 */
  laserEvent02Handle = osEventFlagsNew(&laserEvent02_attributes);

  /* creation of deviceErrorEvent03 */
  deviceErrorEvent03Handle = osEventFlagsNew(&deviceErrorEvent03_attributes);

  /* creation of tecEvent04 */
  tecEvent04Handle = osEventFlagsNew(&tecEvent04_attributes);

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
  tec_init( );
  tec_stop();
  EEPROM_M24C32_init();
  app_sys_param_load();
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
    hsm_dl_sta.real_tmc_temprature=local_disp;
    if(hsm_dl_sta.real_tmc_temprature>MOTOR_TMC_PROTECT_TEMPRATUR)
    {//high
      unsigned int motor_events=osEventFlagsGet(motorEvent01Handle);
      if((motor_events&EVENTS_MOTOR_IDLE_BIT1)!=EVENTS_MOTOR_IDLE_BIT1 ) app_motor_err_handle(osError); 
      osEventFlagsClear(motorEvent01Handle, EVENTS_MOTOR_TEMPRATURE_BIT2);
    }
    else osEventFlagsSet(motorEvent01Handle, EVENTS_MOTOR_TEMPRATURE_BIT2);
    DEBUG_PRINTF(" motor_T=%.2f℃",hsm_dl_sta.real_tmc_temprature);  
    app_get_adc_value(AD2_CH4_TEC_TEMPRATURE,&local_disp);
    //u_s_l980.sta.realtemprature=( short int )(local_disp*10);
    u_s_l980.sta.realtemprature=( short int )(local_disp*10);
    u_sys_param.sys_config_param.targetTempratureSet=240;
    DEBUG_PRINTF(" laser_tec_T=%.1f℃ target=%d",u_s_l980.sta.realtemprature*0.1,u_sys_param.sys_config_param.targetTempratureSet);     
    app_get_adc_value(AD3_CH1_ENERGE_FEEDBACK,&local_disp);
    DEBUG_PRINTF(" energe=%.1f\r\n",local_disp);
    DEBUG_PRINTF("POSITON:pos=%dμm\r\n",u_s_l980.sta.realPosition);       
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
  unsigned int timeOut=0;
  unsigned short int targetPosition=0;
  osDelay(1000);
  osStatus_t m_sta;
  for(;;)
  {
    uint32_t motor_events =osEventFlagsGet(motorEvent01Handle); 
    if((motor_events&EVENTS_MOTOR_ZERO_BIT0)==EVENTS_MOTOR_ZERO_BIT0 )  
    {
      u_s_l980.sta.realPosition=app_get_motor_real_position();
      if(osMessageQueueGet(motorPositonQueue01Handle, &targetPosition, NULL, 20)==osOK)
      {   
        if(u_s_l980.sta.realPosition!=targetPosition)
        {
          if(targetPosition>=L980_MAX_MOTOR_DISTANCE_UM)
          {
            //find zero
            targetPosition=0;
          }
          m_sta= osMutexAcquire(motorMutex01Handle,MOTOR_MOVE_ERROR_TIMEOUT_S); 
          if(m_sta==osOK)
          {
            if(u_s_l980.sta.realPosition>targetPosition)
            {
              app_motor_slide_position(MOTOR_DIR_REVERSE,targetPosition,3);
            }
            else app_motor_slide_position(MOTOR_DIR_FORWARD,targetPosition,3);
            timeOut=0;
            do
            {
              osDelay(L980_CAN_MINI_TIME_MS);
              timeOut+=L980_CAN_MINI_TIME_MS;
              if(timeOut>MOTOR_MOVE_ERROR_TIMEOUT_S)
              {
                app_motor_err_handle(osErrorTimeout); 
                DEBUG_PRINTF("MOTOR error timeout !stop =%dμm\r\n",app_get_motor_real_position());
                break;
              }
              motor_events=osEventFlagsGet(motorEvent01Handle);
            }while((motor_events&EVENTS_MOTOR_IDLE_BIT1)==EVENTS_MOTOR_IDLE_BIT1);
            DEBUG_PRINTF("motor move to=%dμm \r\n",app_get_motor_real_position());
            osMutexRelease(motorMutex01Handle);
          }
        }
      }      
    }
    else 
    {
      if((motor_events&EVENTS_MOTOR_IDLE_BIT1)==EVENTS_MOTOR_IDLE_BIT1) 
      {
        GPIO_PinState zero_pin=HAL_GPIO_ReadPin(MOTOR_ZERO_CHECK_EXTI9_5_IN_GPIO_Port,MOTOR_ZERO_CHECK_EXTI9_5_IN_Pin);
        if(zero_pin==GPIO_PIN_RESET)
        {   
          m_sta= osMutexAcquire(motorMutex01Handle,MOTOR_MOVE_ERROR_TIMEOUT_S); 
          if(m_sta==osOK)
          {
            DEBUG_PRINTF(" reverse zero\r\n");
            app_motor_slide_position(MOTOR_DIR_ZERO,MOTOR_MAX_UM,3);
            timeOut=0;
            do
            {
              osDelay(L980_CAN_MINI_TIME_MS);
              timeOut+=L980_CAN_MINI_TIME_MS;
              if(timeOut>MOTOR_MOVE_ERROR_TIMEOUT_S)
              {
                app_motor_err_handle(osErrorTimeout); 
                DEBUG_PRINTF("MOTOR error timeout !stop =%dμm\r\n",app_get_motor_real_position());
                break;
              }
              motor_events=osEventFlagsGet(motorEvent01Handle);
            }while((motor_events&EVENTS_MOTOR_IDLE_BIT1)!=EVENTS_MOTOR_IDLE_BIT1);
            osMutexRelease(motorMutex01Handle);
          } 
          else  DEBUG_PRINTF(" motor is moving\r\n");        
        }
        else   
        {  
          m_sta= osMutexAcquire(motorMutex01Handle,MOTOR_MOVE_ERROR_TIMEOUT_S); 
          if(m_sta==osOK)
          {
            DEBUG_PRINTF("prepare find zero \r\n"); 
            __HAL_TIM_SET_COUNTER(&htim3,1);
            app_motor_slide_position(MOTOR_DIR_FORWARD,5000,3);
            timeOut=0;
            do
            {
              osDelay(L980_CAN_MINI_TIME_MS);
              timeOut+=L980_CAN_MINI_TIME_MS;
              motor_events=osEventFlagsGet(motorEvent01Handle);
              if(timeOut>MOTOR_MOVE_ERROR_TIMEOUT_S)
              {               
                app_motor_err_handle(osErrorTimeout); 
                DEBUG_PRINTF("MOTOR error timeout !stop =%dμm\r\n",app_get_motor_real_position());
                break;
              }
            }while((motor_events&EVENTS_MOTOR_IDLE_BIT1)==EVENTS_MOTOR_IDLE_BIT1);           
            app_motor_slide_position(MOTOR_DIR_ZERO,MOTOR_MAX_UM,3);
            timeOut=0;
            do
            {
              osDelay(L980_CAN_MINI_TIME_MS);
              timeOut+=L980_CAN_MINI_TIME_MS;
              motor_events=osEventFlagsGet(motorEvent01Handle);
              if(timeOut>MOTOR_MOVE_ERROR_TIMEOUT_S)
              {
                app_motor_err_handle(osErrorTimeout); 
                DEBUG_PRINTF("MOTOR error timeout !stop =%dμm\r\n",app_get_motor_real_position());
                break;
              }              
            }while((motor_events&EVENTS_MOTOR_IDLE_BIT1)==EVENTS_MOTOR_IDLE_BIT1);            
            osMutexRelease(motorMutex01Handle);
          } 
          else  DEBUG_PRINTF(" motor is moving\r\n");    
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
    uint32_t l980_heart_timeout=0;
  for(;;)
  {   
    osDelay(5);
    if(FDCAN1_Receive_Msg(buff, &Identifier,&len))
    {  
      uint8_t packageType = Identifier-CAN_MASTER_ID;  
      CAN_receivePackageHandle(buff,packageType);  
      u_s_l980.sta.staByte|=L980_STA_HEART_BIT0;  //heart
      l980_heart_timeout=0;  
      if(u_l980.set_param.auxLedBulbDutySet!=u_sys_param.sys_config_param.auxLedBulbDutySet)  
      {
        u_sys_param.sys_config_param.auxLedBulbDutySet=u_l980.set_param.auxLedBulbDutySet;
        app_auxiliary_bulb_pwm(u_sys_param.sys_config_param.auxLedBulbDutySet,ENABLE);
      }
    }
    else
    {
      l980_heart_timeout+=5;
      if(l980_heart_timeout>0)
      {//l980 heart timeout 
        l980_heart_timeout=0;  
      // memset(&u_s_l980.sta,0,sizeof(L980_STATUS));//clear 
        u_s_l980.sta.staByte&=(~L980_STA_HEART_BIT0);
        if((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)==L980_STA_PROHOT_BIT1) 
        {
          if (laser_ctr_param.pro_hot!=0)
          {
            laser_ctr_param.pro_hot=0;         
            osSemaphoreRelease(laserCloseBinarySem03Handle);         
          }
        }    
      }     
    }
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
  unsigned int timeout ;
  unsigned int laserWorkTime=0 ;
  for(;;)
  {
    uint32_t laser_event=osEventFlagsWait(laserEvent02Handle,osFlagsWaitAll|osFlagsNoClear,EVENTS_LASER_OK_ALL_BITS_MASK,portMAX_DELAY);
    osStatus_t c_sta= osSemaphoreAcquire(laserCloseBinarySem03Handle,5);
    if(c_sta==osOK)
    {
      app_980_pwr_en(DISABLE);
      u_s_l980.sta.staByte&=(~L980_STA_PULSEOUT_BIT2);
      app_dac_out(0);
      osEventFlagsClear(laserEvent02Handle, EVENTS_LASER_VOLTAGE_BIT2);  
      //position
      osStatus_t m_sta= osMutexAcquire(motorMutex01Handle,MOTOR_MOVE_ERROR_TIMEOUT_S); 
      unsigned int motor_events=osEventFlagsGet(motorEvent01Handle); 
      if(m_sta==osOK)
      {
        DEBUG_PRINTF("motor return zero...\r\n");            
        app_motor_slide_position(MOTOR_DIR_ZERO,0,3);
        motor_events=osEventFlagsGet(motorEvent01Handle); 
        timeout=0;
        do
        {
          osDelay(L980_CAN_MINI_TIME_MS);
          timeout+=L980_CAN_MINI_TIME_MS;
          motor_events=osEventFlagsGet(motorEvent01Handle);
          if(timeout>MOTOR_MOVE_ERROR_TIMEOUT_S)
          {
            app_motor_err_handle(osErrorTimeout);                    
            u_s_l980.sta.staByte&=(~L980_STA_PROHOT_BIT1);
            DEBUG_PRINTF("MOTOR error timeout !stop =%dμm\r\n",app_get_motor_real_position());
            break;
          }         
        }while((motor_events&EVENTS_MOTOR_OK_ALL_BITS_MASK)!=EVENTS_MOTOR_OK_ALL_BITS_MASK); 
        if((motor_events&EVENTS_MOTOR_OK_ALL_BITS_MASK)==EVENTS_MOTOR_OK_ALL_BITS_MASK)  
        {
          osEventFlagsClear(laserEvent02Handle, EVENTS_LASER_POSITON_BIT0);
        }
        osMutexRelease(motorMutex01Handle);
      } 
      else
      {
        osEventFlagsClear(laserEvent02Handle, EVENTS_LASER_POSITON_BIT0);
        osEventFlagsSet(motorEvent01Handle,EVENTS_MOTOR_IDLE_BIT1);
        u_s_l980.sta.staByte|=L980_ERR_MOTOR_BIT7;
        u_s_l980.sta.staByte&=(~L980_STA_PROHOT_BIT1);
        DEBUG_PRINTF("motor error! \r\n"); 
      } 
      if((motor_events&EVENTS_MOTOR_OK_ALL_BITS_MASK)==EVENTS_MOTOR_OK_ALL_BITS_MASK)
      {
        osEventFlagsClear(laserEvent02Handle, EVENTS_LASER_POSITON_BIT0);
      }  
      osTimerStop(laserWorkTimer01Handle);
      u_s_l980.sta.staByte&=(~L980_STATUS_BYTE_MASK);   
      DEBUG_PRINTF("l980 exit prohot \r\n");
    }    
    if(laser_ctr_param.JT_laser_out!=0&&((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)==L980_STA_PROHOT_BIT1))
    {
      if((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)!=L980_STA_PULSEOUT_BIT2)
      { 
        laserWorkTime=osKernelGetTickCount();
        if(laser_ctr_param.laserCountTimerCtr!=0&&u_l980.set_param.timerSet!=0)
        {
          if((u_s_l980.sta.staByte&L980_STA_TIMERS_BIT3)!=L980_STA_TIMERS_BIT3) 
          {
            u_s_l980.sta.staByte|=L980_STA_TIMERS_BIT3;
            osTimerStart(laserWorkTimer01Handle,u_l980.set_param.timerSet); 
          }
        }        
        app_980_pwr_en(ENABLE);   
        u_s_l980.sta.staByte|=L980_STA_PULSEOUT_BIT2;
        DEBUG_PRINTF("l980 JT pulse out \r\n");
      }  
      else 
      {
        if(osKernelGetTickCount()>laserWorkTime+1000)
        {
          laserWorkTime=osKernelGetTickCount();
          u_sys_param.sys_config_param.laser_use_timeS++;
          u_s_l980.sta.laserUseTimeS=u_sys_param.sys_config_param.laser_use_timeS;
        }        
      }    
    }
    else 
    {
      if(laser_ctr_param.JT_laser_out==0) 
      {
        osTimerStop(laserWorkTimer01Handle);
        u_s_l980.sta.staByte&=(~L980_STA_TIMERS_BIT3);
        if((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)==L980_STA_PULSEOUT_BIT2) 
        {
          DEBUG_PRINTF("l980 JT stop pulse out \r\n");
          app_980_pwr_en(DISABLE);  
          u_s_l980.sta.staByte&=(~L980_STA_PULSEOUT_BIT2);
        }        
      }
    }
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
  unsigned int timeout=0;
  unsigned char local_prohot;  
  for(;;)
  {
    osSemaphoreAcquire(prohotBinarySem02Handle,portMAX_DELAY);
    local_prohot=laser_ctr_param.pro_hot;
    if(local_prohot!=0)
    {  
      //temprature 
      
      //laser_ctr_param.energe=u_l980.set_param.energeSet+u_l980.set_param.energeCaliSet;laser_ctr_param.energe=u_l980.set_param.energeSet;
      unsigned short int dac_voltage= app_laser_980_energe_to_voltage(laser_ctr_param.energe);
      app_dac_out(dac_voltage); 
      osEventFlagsSet(laserEvent02Handle, EVENTS_LASER_VOLTAGE_BIT2);
      //position       
      osStatus_t m_sta= osMutexAcquire(motorMutex01Handle,MOTOR_MOVE_ERROR_TIMEOUT_S); 
      if(m_sta==osOK)
      {
        DEBUG_PRINTF("motor move to=%dμm \r\n",u_l980.set_param.positionSet);            
        app_motor_slide_position(MOTOR_DIR_FORWARD,u_l980.set_param.positionSet,3);
        unsigned int motor_events=osEventFlagsGet(motorEvent01Handle); 
        timeout=0;
        do
        {
          osDelay(L980_CAN_MINI_TIME_MS);
          timeout+=L980_CAN_MINI_TIME_MS;
          if(timeout>MOTOR_MOVE_ERROR_TIMEOUT_S)
          {
            app_motor_err_handle(osErrorTimeout);             
            osEventFlagsClear(laserEvent02Handle, EVENTS_LASER_POSITON_BIT0);
            app_dac_out(0);
            osEventFlagsClear(laserEvent02Handle, EVENTS_LASER_VOLTAGE_BIT2);                    
            u_s_l980.sta.staByte&=(~L980_STATUS_BYTE_MASK);            
            DEBUG_PRINTF("MOTOR error timeout! prohot fail\r\n"); 
            break;
          }
          motor_events=osEventFlagsGet(motorEvent01Handle);          
        }while((motor_events&EVENTS_MOTOR_OK_ALL_BITS_MASK)!=EVENTS_MOTOR_OK_ALL_BITS_MASK); 
        if((motor_events&EVENTS_MOTOR_OK_ALL_BITS_MASK)==EVENTS_MOTOR_OK_ALL_BITS_MASK)  
        {
          if(u_l980.set_param.positionSet==u_s_l980.sta.realPosition)  
          {
            osEventFlagsSet(laserEvent02Handle, EVENTS_LASER_POSITON_BIT0);
          }
        }
        osMutexRelease(motorMutex01Handle);
      } 
      else
      {
        osEventFlagsSet(motorEvent01Handle,EVENTS_MOTOR_IDLE_BIT1);
        u_s_l980.sta.staByte|=L980_ERR_MOTOR_BIT7;
        u_s_l980.sta.staByte&=(~L980_STA_PROHOT_BIT1);
        DEBUG_PRINTF("motor error! prohot fail\r\n"); 
      } 
      uint32_t laser_event=osEventFlagsGet(laserEvent02Handle);  
      if((laser_event&EVENTS_LASER_OK_ALL_BITS_MASK)==EVENTS_LASER_OK_ALL_BITS_MASK)
      {
        u_s_l980.sta.staByte|=L980_STA_PROHOT_BIT1;
        DEBUG_PRINTF("l9890 prohot success\r\n");
      }      
    }
    else 
    {
      if((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)==L980_STA_PROHOT_BIT1)  osSemaphoreRelease(laserCloseBinarySem03Handle);
    }
    osDelay(1);
  }
  /* USER CODE END laserProhotTask05 */
}

/* laserWorkTimerCallback01 function */
void laserWorkTimerCallback01(void *argument)
{
  /* USER CODE BEGIN laserWorkTimerCallback01 */
  if(laser_ctr_param.laserCountTimerCtr!=0)
  {
    app_980_pwr_en(DISABLE);
    u_s_l980.sta.staByte&=(~(L980_STA_TIMERS_BIT3|L980_STA_PULSEOUT_BIT2)); 
  } 
  /* USER CODE END laserWorkTimerCallback01 */
}

/* tecRunCallback02 function */
void tecRunCallback02(void *argument)
{
  /* USER CODE BEGIN tecRunCallback02 */
  tec_stop();
  /* USER CODE END tecRunCallback02 */
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
        u_s_l980.sta.staByte&=(~L980_ERR_MOTOR_BIT7);
        osEventFlagsSet(motorEvent01Handle,EVENTS_MOTOR_OK_ALL_BITS_MASK);  
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
  osEventFlagsSet(tecEvent04Handle,EVENTS_TEC_STA_IDLE_BIT0);
}
/************************************************************************//**
  * @brief  relese
  * @param   
  * @note    
  * @retval None
  ****************************************************************************/
 void app_laser_prohot_semo(void)
 {
   osSemaphoreRelease(prohotBinarySem02Handle);
 }
   /************************************************************************//**
  * @brief laser
  * @param energe ,能量
  * @note   能量单位mJ;
  * @retval  换算后电压100mV
  *****************************************************************************/
 unsigned short int  app_laser_980_energe_to_voltage(unsigned short int energe)
 {
   unsigned short int ret_vol; 
   ret_vol=energe*4.5+300;   
   if(ret_vol<LASER_980_MIN_ENERGE_MV) ret_vol=LASER_980_MIN_ENERGE_MV;
   if(ret_vol>LASER_980_MAX_ENERGE_MV) ret_vol=LASER_980_MAX_ENERGE_MV;
   return ret_vol;
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
  * @brief  app_motor_err_handle
  * @param  errSta
  * @note    只响应超时
  * @retval None
  ****************************************************************************/
 void app_motor_err_handle(osStatus_t errSta)
 {
    if(errSta!=osOK) 
    {
      tmc2226_stop();
      u_s_l980.sta.staByte|=L980_ERR_MOTOR_BIT7;
      osEventFlagsSet(motorEvent01Handle,EVENTS_MOTOR_IDLE_BIT1);
      DEBUG_PRINTF("MOTOR error timeout !stop =%dμm\r\n",app_get_motor_real_position());
    }   
 }
   /************************************************************************//**
  * @brief  电机正常停止并更新状态
  * @param  errSta
  * @note    只响应超时
  * @retval None
  ****************************************************************************/
 void app_motor_stop_fresh_status(void)
 {   
    tmc2226_stop();
    u_s_l980.sta.staByte&=(~L980_ERR_MOTOR_BIT7);
    osEventFlagsSet(motorEvent01Handle,EVENTS_MOTOR_IDLE_BIT1);
    DEBUG_PRINTF("MOTOR stop =%dμm\r\n",app_get_motor_real_position());
 }
    /************************************************************************//**
  * @brief  app_motor_move_to_sem
  * @param  errSta
  * @note    电机移动
  * @retval None
  ****************************************************************************/
 void app_motor_move_to_sem(unsigned short int targetPosition)
 {   
  if(targetPosition==0)
  {
    app_motor_stop_fresh_status();//立即停止                    
  }        
  else if(targetPosition>L980_MAX_MOTOR_DISTANCE_UM)
  {
    targetPosition=L980_MAX_MOTOR_DISTANCE_UM;
    osMessageQueuePut(motorPositonQueue01Handle, &targetPosition, NULL, 1);    
    DEBUG_PRINTF("980 motor find zero\r\n");        
  }
  else
  {
    osMessageQueuePut(motorPositonQueue01Handle, &targetPosition, NULL, 1);
    DEBUG_PRINTF("980 motor move to =%dμm\r\n",targetPosition); 
  } 
    
 }
/************************************************************************//**
* @brief  app_tec_auto_manage
* @param  
* @note    
         volta,tec Voltage(max 24V)
* @retval None
****************************************************************************/
void app_tec_auto_manage(void)
{
  int volta;    
  u_s_l980.sta.realtemprature=(unsigned short int )hsm_dl_sta.real_tmc_temprature*10;
  DEBUG_PRINTF("T=%d",u_s_l980.sta.realtemprature); 
  if(HAL_GPIO_ReadPin(TEC_DRV8701_ERROR_IN_GPIO_Port,TEC_DRV8701_ERROR_IN_Pin)==GPIO_PIN_SET)
  {
    osEventFlagsSet(tecEvent04Handle,EVENTS_TEC_ERR_BIT1);
  }
  else osEventFlagsClear(tecEvent04Handle,EVENTS_TEC_ERR_BIT1);
  uint32_t tec_evnet = osEventFlagsGet(tecEvent04Handle);  
  if(u_s_l980.sta.realtemprature+2>240)//u_sys_param.sys_config_param.targetTempratureSet*0.1)
  {
    volta = -20;
    if(tec_evnet==EVENTS_TEC_OK_ALL_BITS_MASK)
    {
      osEventFlagsClear(tecEvent04Handle,EVENTS_TEC_STA_IDLE_BIT0);
      osTimerStart(tecRunTimer02Handle,500);      
     // tec_start(volta,500);
    } 
    else DEBUG_PRINTF("the tec is running\r\n");        
  }
  else if(u_s_l980.sta.realtemprature+2<240)//u_sys_param.sys_config_param.targetTempratureSet*0.1)
  {  
    volta = 20;    
    if(tec_evnet==EVENTS_TEC_OK_ALL_BITS_MASK)
    {
      osTimerStart(tecRunTimer02Handle,500);
      osEventFlagsClear(tecEvent04Handle,EVENTS_TEC_STA_IDLE_BIT0);
      //tec_start(volta,500);
    } 
    else DEBUG_PRINTF("the tec is running\r\n");   
  }  
  if(u_s_l980.sta.realtemprature>300||u_s_l980.sta.realtemprature<160)
  {//温度超限
    u_s_l980.sta.staByte|=L980_ERR_TEMPRATURE_BIT4;
    osEventFlagsClear(laserEvent02Handle, EVENTS_LASER_TEMPRATUR_BIT1);
  }
  else 
  {
    u_s_l980.sta.staByte&=(~L980_ERR_TEMPRATURE_BIT4);
    osEventFlagsSet(laserEvent02Handle, EVENTS_LASER_TEMPRATUR_BIT1);
  } 
}
/************************************************************************//**
  * @brief 计算设备ID
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 unsigned  int app_get_cali_devid(void)
 {
   unsigned  int id;
   unsigned char buf[12];//96bit	
   buf[0]=HAL_GetUIDw0()&0xFF;
   buf[1]=(HAL_GetUIDw0()>>8)&0xFF;
   buf[2]=(HAL_GetUIDw0()>>16)&0xFF;
   buf[3]=(HAL_GetUIDw0()>>24)&0xFF;
 
   buf[4]=HAL_GetUIDw1()&0xFF;
   buf[5]=(HAL_GetUIDw1()>>8)&0xFF;
   buf[6]=(HAL_GetUIDw1()>>16)&0xFF;
   buf[7]=(HAL_GetUIDw1()>>24)&0xFF;	
 
   buf[8]=HAL_GetUIDw2()&0xFF;
   buf[9]=(HAL_GetUIDw2()>>8)&0xFF;
   buf[10]=(HAL_GetUIDw2()>>16)&0xFF;
   buf[11]=(HAL_GetUIDw2()>>24)&0xFF;	
   id =  crc32_MPEG(buf,12);
   return id;
 }	
 /************************************************************************//**
   * @brief 使用默认系统参数
   * @param 无
   * @note   
   * @retval 
   *****************************************************************************/
 void app_set_default_sys_config_param(void)
 {	
    DEBUG_PRINTF("sys param load failed! load defalut param\r\n");
    u_sys_param.sys_config_param.deviceID= app_get_cali_devid();
    u_sys_param.sys_config_param.laser_use_timeS=0;
    u_sys_param.sys_config_param.targetTempratureSet=240;//23.0
    u_sys_param.sys_config_param.auxLedBulbDutySet=20;
    u_sys_param.sys_config_param.auxLedBulbFreqSet=10;//10k
    u_sys_param.sys_config_param.positionSet=15000;
    u_sys_param.sys_config_param.timerSet=180;
    for(unsigned short int i=0;i<41;i++)
    {
    u_sys_param.sys_config_param.e_cali[i]=25000;
    }
    u_sys_param.sys_config_param.checkSum=sumCheck(u_sys_param.data,sizeof(SYS_CONFIG_PARAM)-4);
    memcpy(u_sys_default_param.data,u_sys_param.data,sizeof(SYS_CONFIG_PARAM));
    memcpy(u_l980.data,&u_sys_param.data[8],sizeof(L980_SET_PARAM));
    memset(u_s_l980.data,0,sizeof(L980_STATUS));
    u_s_l980.sta.laserUseTimeS=u_sys_param.sys_config_param.laser_use_timeS; 
 }
 /************************************************************************//**
   * @brief 加载本地系统参数
   * @param 无
   * @note   
   * @retval 
   *****************************************************************************/
   void app_sys_param_load(void)
   {
     unsigned char flag;
     flag = EEPROM_M24C32_Read(EEROM_SYS_PARAM_SAVE_ADDR, u_sys_param.data, sizeof(SYS_CONFIG_PARAM));
     unsigned int sum = sumCheck(u_sys_param.data,sizeof(SYS_CONFIG_PARAM)-4);   
     if(u_sys_param.sys_config_param.checkSum!=sum)//
     {
       app_set_default_sys_config_param();
       DEBUG_PRINTF("load default sys param\r\n");		 
     }
     else 
     {
       memcpy(u_sys_default_param.data,u_sys_param.data,sizeof(SYS_CONFIG_PARAM));
       memcpy(u_l980.data,&u_sys_param.data[8],sizeof(L980_SET_PARAM));
       memset(u_s_l980.data,0,sizeof(L980_STATUS));
       u_s_l980.sta.laserUseTimeS=u_sys_param.sys_config_param.laser_use_timeS; 
   
       #if 1
       DEBUG_PRINTF("***************sys param read ok*************************\r\n");
      
       DEBUG_PRINTF("*******************sys param end*************************\r\n");
       #endif
     }	  
   }
 /************************************************************************//**
   * @brief 系统参数保存到本地
   * @param 无
   * @note   
   * @retval 
   *****************************************************************************/
  unsigned char app_sys_param_save_data(void)
  {
     unsigned char flag=0;	 
     if(compare_buff_no_change(u_sys_param.data,u_sys_default_param.data,sizeof(SYS_CONFIG_PARAM))!=HAL_OK)
     {
       u_sys_param.sys_config_param.checkSum=sumCheck(u_sys_param.data,sizeof(SYS_CONFIG_PARAM)-4);
       flag = EEPROM_M24C32_Write(EEROM_SYS_PARAM_SAVE_ADDR, u_sys_param.data, sizeof(SYS_CONFIG_PARAM));	
     }		  
     return flag;
  }
/* USER CODE END Application */

