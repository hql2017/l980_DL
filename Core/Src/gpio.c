/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L980_LASER_PWR_ON_OUT_Pin|LASER980_DAC_EN_OUT_Pin|TEC_DRV8701_PHASE_CTR_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TMC2226_CTR_EN_OUT_Pin|TMC2226_DIR_OUT_Pin|EEROM_WC_OUT_Pin|MCU_LED_CTR_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEC_DRV8701_SLEEP_OUT_GPIO_Port, TEC_DRV8701_SLEEP_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L980_LASER_PWR_ON_OUT_Pin LASER980_DAC_EN_OUT_Pin TEC_DRV8701_PHASE_CTR_OUT_Pin */
  GPIO_InitStruct.Pin = L980_LASER_PWR_ON_OUT_Pin|LASER980_DAC_EN_OUT_Pin|TEC_DRV8701_PHASE_CTR_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_ZERO_CHECK_EXTI9_5_IN_Pin */
  GPIO_InitStruct.Pin = MOTOR_ZERO_CHECK_EXTI9_5_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MOTOR_ZERO_CHECK_EXTI9_5_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LASER980_ERROR_OVERLOAD_IN_Pin */
  GPIO_InitStruct.Pin = LASER980_ERROR_OVERLOAD_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LASER980_ERROR_OVERLOAD_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENERGE_FEEDBACK_OVERLOAD_IN_Pin TMC2226_ERROR_IN_Pin */
  GPIO_InitStruct.Pin = ENERGE_FEEDBACK_OVERLOAD_IN_Pin|TMC2226_ERROR_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TMC2226_CTR_EN_OUT_Pin TMC2226_DIR_OUT_Pin EEROM_WC_OUT_Pin MCU_LED_CTR_OUT_Pin */
  GPIO_InitStruct.Pin = TMC2226_CTR_EN_OUT_Pin|TMC2226_DIR_OUT_Pin|EEROM_WC_OUT_Pin|MCU_LED_CTR_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TEC_DRV8701_ERROR_IN_Pin */
  GPIO_InitStruct.Pin = TEC_DRV8701_ERROR_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TEC_DRV8701_ERROR_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEC_DRV8701_SLEEP_OUT_Pin */
  GPIO_InitStruct.Pin = TEC_DRV8701_SLEEP_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEC_DRV8701_SLEEP_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECODER_Z_EXT2I_IN_Pin */
  GPIO_InitStruct.Pin = ECODER_Z_EXT2I_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECODER_Z_EXT2I_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 2 */
 /************************************************************************//**
  * @brief 激光电源开关
  * @param  flag-使能信号
  * @note   高电平有效
  * @retval None
  ****************************************************************************/
 void app_load_power_switch( FunctionalState flag)
 {   
   if(flag==DISABLE)  HAL_GPIO_WritePin(L980_LASER_PWR_ON_OUT_GPIO_Port,L980_LASER_PWR_ON_OUT_Pin,GPIO_PIN_RESET);
   else HAL_GPIO_WritePin(L980_LASER_PWR_ON_OUT_GPIO_Port,L980_LASER_PWR_ON_OUT_Pin,GPIO_PIN_SET);  
 }

/************************************************************************//**
  * @brief mcu_sysLED运行模式
  * @param  flag-使能信号
  * @note   0，cool ；1 hot
  * @retval None
  ****************************************************************************/
 void app_mcu_sys_led(  FunctionalState flag)
 {   
   if(flag==0)  HAL_GPIO_WritePin(MCU_LED_CTR_OUT_GPIO_Port,MCU_LED_CTR_OUT_Pin,GPIO_PIN_RESET);
   else HAL_GPIO_WritePin(MCU_LED_CTR_OUT_GPIO_Port,MCU_LED_CTR_OUT_Pin,GPIO_PIN_SET);  
 }
    /************************************************************************//**
  * @brief app_980_dac_en
  * @param  flag-使能信号
  * @note   激光DAC使能 
  * @retval None
  ****************************************************************************/
 void app_980_dac_en(  FunctionalState flag)
 {   
   if(flag==0)  HAL_GPIO_WritePin(LASER980_DAC_EN_OUT_GPIO_Port,LASER980_DAC_EN_OUT_Pin,GPIO_PIN_RESET);
   else HAL_GPIO_WritePin(LASER980_DAC_EN_OUT_GPIO_Port,LASER980_DAC_EN_OUT_Pin,GPIO_PIN_SET); 
    
 }
/* USER CODE END 2 */
