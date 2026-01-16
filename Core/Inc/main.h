/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

typedef struct
{  
  unsigned char  heartFlag;                   //  
	unsigned char  jt_key_status;               //  脚踏状态 
  unsigned short int cool_temprature_target;  //）10倍冷却系统目标温度  
  unsigned  int laser_use_timeS;              //）连续激光使用时间S（980脉冲激光运行时间。(MAX,连续49.7天)）
  unsigned char tec_switch;                   //）制冷片开关
  unsigned char laser_led_light;              //）激光指示灯亮度
  unsigned short int  e_cali[40];             //）功率校准值
}__attribute__ ((packed)) SYS_CONFIG_PARAM ;//系统配置参数
typedef union 
{
	SYS_CONFIG_PARAM sys_config_param;
	unsigned char data[sizeof(SYS_CONFIG_PARAM)];
}U_SYS_CONFIG_PARAM;

extern U_SYS_CONFIG_PARAM u_sys_param;
extern U_SYS_CONFIG_PARAM u_sys_default_param;

typedef struct
{    
  unsigned char       caliFlag;//是否校准模式
  unsigned short int  energe;                     //
  unsigned short int  laserContinuousTimes;        // 定时  
}__attribute__ ((packed)) LASER_CTR_PARAM ;//激光控制参数
typedef struct
{  
  unsigned char laser_status;
  unsigned char laser_prohot_status;
  unsigned char laser_980_out_status;
  unsigned int zeroFlag;        //0:not set zero   1; set  zero
  unsigned int positionUm;     //position μm
}__attribute__ ((packed)) LASER_980_STATUS ;//运行状态参数

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_LPUART1_RX_Pin GPIO_PIN_0
#define DEBUG_LPUART1_RX_GPIO_Port GPIOC
#define DEBUG_LPUART1_TX_Pin GPIO_PIN_1
#define DEBUG_LPUART1_TX_GPIO_Port GPIOC
#define LOAD_PWR_ON_OUT_Pin GPIO_PIN_3
#define LOAD_PWR_ON_OUT_GPIO_Port GPIOC
#define ADC1_IN1_iBUS_Pin GPIO_PIN_0
#define ADC1_IN1_iBUS_GPIO_Port GPIOA
#define ADC1_IN2_VBUS_Pin GPIO_PIN_1
#define ADC1_IN2_VBUS_GPIO_Port GPIOA
#define ADC1_IN3_iTEC_DRV8701_Pin GPIO_PIN_2
#define ADC1_IN3_iTEC_DRV8701_GPIO_Port GPIOA
#define ADC1_IN4_i980_Pin GPIO_PIN_3
#define ADC1_IN4_i980_GPIO_Port GPIOA
#define DAC1_OUT1_LASER980_PWR4V2_Pin GPIO_PIN_4
#define DAC1_OUT1_LASER980_PWR4V2_GPIO_Port GPIOA
#define MOTOR_ZERO_CHECK_EXTI9_5_IN_Pin GPIO_PIN_5
#define MOTOR_ZERO_CHECK_EXTI9_5_IN_GPIO_Port GPIOA
#define ADC2_IN3_MOTOR_TEMPRATURE_Pin GPIO_PIN_6
#define ADC2_IN3_MOTOR_TEMPRATURE_GPIO_Port GPIOA
#define ADC2_IN4_TEC_TEMPRATURE_Pin GPIO_PIN_7
#define ADC2_IN4_TEC_TEMPRATURE_GPIO_Port GPIOA
#define LASER980_PWR_EN_OUT_Pin GPIO_PIN_4
#define LASER980_PWR_EN_OUT_GPIO_Port GPIOC
#define LASER980_ERROR_OVERLOAD_IN_Pin GPIO_PIN_5
#define LASER980_ERROR_OVERLOAD_IN_GPIO_Port GPIOC
#define ENERGE_FEEDBACK_OVERLOAD_IN_Pin GPIO_PIN_0
#define ENERGE_FEEDBACK_OVERLOAD_IN_GPIO_Port GPIOB
#define ADC3_IN1_ENERGE_FEEDBACK_Pin GPIO_PIN_1
#define ADC3_IN1_ENERGE_FEEDBACK_GPIO_Port GPIOB
#define TMC2226_USART3_TX_Pin GPIO_PIN_10
#define TMC2226_USART3_TX_GPIO_Port GPIOB
#define TMC2226_USART3_RX_Pin GPIO_PIN_11
#define TMC2226_USART3_RX_GPIO_Port GPIOB
#define TMC2226_ERROR_IN_Pin GPIO_PIN_13
#define TMC2226_ERROR_IN_GPIO_Port GPIOB
#define TMC2226_CTR_EN_OUT_Pin GPIO_PIN_14
#define TMC2226_CTR_EN_OUT_GPIO_Port GPIOB
#define TMC2226_DIR_OUT_Pin GPIO_PIN_15
#define TMC2226_DIR_OUT_GPIO_Port GPIOB
#define TMC2226_STEP_TIM8CH1_PWM_Pin GPIO_PIN_6
#define TMC2226_STEP_TIM8CH1_PWM_GPIO_Port GPIOC
#define AUXILIARY_BULB_TIM8CH2_PWM_Pin GPIO_PIN_7
#define AUXILIARY_BULB_TIM8CH2_PWM_GPIO_Port GPIOC
#define TEC_DRV8701_TIM20CH2_PWM_Pin GPIO_PIN_8
#define TEC_DRV8701_TIM20CH2_PWM_GPIO_Port GPIOC
#define TEC_DRV8701_PHASE_CTR_OUT_Pin GPIO_PIN_9
#define TEC_DRV8701_PHASE_CTR_OUT_GPIO_Port GPIOC
#define TEC_DRV8701_ERROR_IN_Pin GPIO_PIN_8
#define TEC_DRV8701_ERROR_IN_GPIO_Port GPIOA
#define TEC_DRV8701_SLEEP_OUT_Pin GPIO_PIN_9
#define TEC_DRV8701_SLEEP_OUT_GPIO_Port GPIOA
#define FDCAN1_RX_Pin GPIO_PIN_11
#define FDCAN1_RX_GPIO_Port GPIOA
#define FDCAN1_TX_Pin GPIO_PIN_12
#define FDCAN1_TX_GPIO_Port GPIOA
#define EEROM_I2C1_SCL_Pin GPIO_PIN_15
#define EEROM_I2C1_SCL_GPIO_Port GPIOA
#define ECODER_Z_EXT2I_IN_Pin GPIO_PIN_2
#define ECODER_Z_EXT2I_IN_GPIO_Port GPIOD
#define ECODER_Z_EXT2I_IN_EXTI_IRQn EXTI2_IRQn
#define ECODER_A_TIM3CH1_INPUT_Pin GPIO_PIN_4
#define ECODER_A_TIM3CH1_INPUT_GPIO_Port GPIOB
#define ECODER_B_TIM3CH2_INPUT_Pin GPIO_PIN_5
#define ECODER_B_TIM3CH2_INPUT_GPIO_Port GPIOB
#define EEROM_WC_OUT_Pin GPIO_PIN_6
#define EEROM_WC_OUT_GPIO_Port GPIOB
#define EEROM_I2C1_SDA_Pin GPIO_PIN_7
#define EEROM_I2C1_SDA_GPIO_Port GPIOB
#define MCU_LED_CTR_OUT_Pin GPIO_PIN_9
#define MCU_LED_CTR_OUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#ifndef DEBUG_MSG_UART 
#define DEBUG_MSG_UART  /*use printf*/
#define DEBUG_PRINTF(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(fmt, ...) do { } while(0)
#endif

#ifdef IWDG_USED
#define IWDG_USED  
#endif

#ifndef MODBUS_RTU_CAN_ENABLE
#define MODBUS_RTU_CAN_ENABLE
#define  CAN_MASTER_ID   0x055
#define  CAN_SLAVE_ID   0x088
#else
#define CANOPEN_USED
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
