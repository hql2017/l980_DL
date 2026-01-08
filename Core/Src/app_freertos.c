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

#include "tim.h"
#include "dac.h"
#include "gpio.h"
#include "tmc2226_step_bsp.h"
#include "eeprom_bsp.h"
#include "tec_control_bsp.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void testTask02(void *argument);

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(testTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  tmc2226_init(); 
   unsigned int local_system=0;
    unsigned char testflag=0;
  for(;;)
  {    
    local_system++;    
    local_system%=20;

    if(testflag==0) //zero
    {
      if(local_system==1)
      {
        if(HAL_GPIO_ReadPin(MOTOR_ZERO_CHECK_EXTI9_5_IN_GPIO_Port,MOTOR_ZERO_CHECK_EXTI9_5_IN_Pin)==GPIO_PIN_RESET)
        {
          DEBUG_PRINTF("find zero\r\n ");                     
          app_tmc_um_start(MOTOR_DIR_ZERO,MAX_TRIP_STEPS_COUNT,3);
        }  
        else   
        {
          testflag=1;
          app_tmc_um_start(MOTOR_DIR_FORWARD,12000,3);
        }
      }
    }
    else 
    {
      if(local_system==10)
      {
        if((htim3.Instance->CNT)<12000)
        {
          DEBUG_PRINTF(" forward 12mm\r\n ");
          app_tmc_um_start(MOTOR_DIR_FORWARD,15000,3);
        }
        else
        {
        
          DEBUG_PRINTF(" rever  8mm\r\n ");
          app_tmc_um_start(MOTOR_DIR_REVERSE,5000,3);
        }
      }
    }    
    DEBUG_PRINTF("MOTOR:DIR=%d STEPS=%d distanceUm=%d\r\n",__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3),htim4.Instance->CNT,(htim3.Instance->CNT+1)>>1);
    osDelay(1000);   
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_testTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_testTask02 */
void testTask02(void *argument)
{
  /* USER CODE BEGIN testTask02 */
  /* Infinite loop */
  HAL_GPIO_WritePin(MCU_LED_CTR_OUT_GPIO_Port,MCU_LED_CTR_OUT_Pin,GPIO_PIN_SET);  
  for(;;)
  {
		HAL_GPIO_TogglePin(MCU_LED_CTR_OUT_GPIO_Port,MCU_LED_CTR_OUT_Pin);
    osDelay(500);
  }
  /* USER CODE END testTask02 */
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
        DEBUG_PRINTF("index zero encode=%d \r\n",htim3.Instance->CNT);  
        __HAL_TIM_SET_COUNTER(&htim3,0);
      }      
    }
  }
}
/* USER CODE END Application */

