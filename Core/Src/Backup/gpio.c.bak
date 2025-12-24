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
  HAL_GPIO_WritePin(GPIOC, LOAD_PWR_ON_OUT_Pin|LASER980_PWR_EN_OUT_Pin|TEC_DRV8701_PHASE_CTR_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TMC2226_CTR_EN_OUT_Pin|TMC2226_DIR_OUT_Pin|EEROM_WC_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEC_DRV8701_SLEEP_OUT_GPIO_Port, TEC_DRV8701_SLEEP_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LOAD_PWR_ON_OUT_Pin LASER980_PWR_EN_OUT_Pin TEC_DRV8701_PHASE_CTR_OUT_Pin */
  GPIO_InitStruct.Pin = LOAD_PWR_ON_OUT_Pin|LASER980_PWR_EN_OUT_Pin|TEC_DRV8701_PHASE_CTR_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pins : TMC2226_CTR_EN_OUT_Pin TMC2226_DIR_OUT_Pin EEROM_WC_OUT_Pin */
  GPIO_InitStruct.Pin = TMC2226_CTR_EN_OUT_Pin|TMC2226_DIR_OUT_Pin|EEROM_WC_OUT_Pin;
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

  /*Configure GPIO pin : ECODE_Z_IN_Pin */
  GPIO_InitStruct.Pin = ECODE_Z_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECODE_Z_IN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
