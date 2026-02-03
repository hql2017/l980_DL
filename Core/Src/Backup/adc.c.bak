/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

#include "stdio.h"
#include "math.h"
#include "tim.h"
#define AD_VREF_VOLTAGE  3000
#define MAX_AD1_BUFF_LENGTH  64
#define MAX_AD1_BUFF_BYTES_LENGTH  128//MAX_AD1_BUFF_LENGTH*2 

#define MAX_AD2_BUFF_LENGTH  32
#define MAX_AD2_BUFF_BYTES_LENGTH  64//MAX_AD2_BUFF_LENGTH*2 

#define MAX_AD3_BUFF_LENGTH  16
#define MAX_AD3_BUFF_BYTES_LENGTH  32//MAX_AD3_BUFF_LENGTH*2

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
  /* USER CODE END ADC2_Init 2 */

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc3,ADC_SINGLE_ENDED);
  /* USER CODE END ADC3_Init 2 */

}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC1 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4
    */
    GPIO_InitStruct.Pin = ADC1_IN1_iBUS_Pin|ADC1_IN2_VBUS_Pin|ADC1_IN3_iTEC_DRV8701_Pin|ADC1_IN4_i980_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC2 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA6     ------> ADC2_IN3
    PA7     ------> ADC2_IN4
    */
    GPIO_InitStruct.Pin = ADC2_IN3_MOTOR_TEMPRATURE_Pin|ADC2_IN4_TEC_TEMPRATURE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA1_Channel2;
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC3 clock enable */
    __HAL_RCC_ADC345_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PB1     ------> ADC3_IN1
    */
    GPIO_InitStruct.Pin = ADC3_IN1_ENERGE_FEEDBACK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC3_IN1_ENERGE_FEEDBACK_GPIO_Port, &GPIO_InitStruct);

    /* ADC3 DMA Init */
    /* ADC3 Init */
    hdma_adc3.Instance = DMA2_Channel1;
    hdma_adc3.Init.Request = DMA_REQUEST_ADC3;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc3);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4
    */
    HAL_GPIO_DeInit(GPIOA, ADC1_IN1_iBUS_Pin|ADC1_IN2_VBUS_Pin|ADC1_IN3_iTEC_DRV8701_Pin|ADC1_IN4_i980_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PA6     ------> ADC2_IN3
    PA7     ------> ADC2_IN4
    */
    HAL_GPIO_DeInit(GPIOA, ADC2_IN3_MOTOR_TEMPRATURE_Pin|ADC2_IN4_TEC_TEMPRATURE_Pin);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC345_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PB1     ------> ADC3_IN1
    */
    HAL_GPIO_DeInit(ADC3_IN1_ENERGE_FEEDBACK_GPIO_Port, ADC3_IN1_ENERGE_FEEDBACK_Pin);

    /* ADC3 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

static  unsigned  short int ad1Buff[MAX_AD1_BUFF_LENGTH];
static  unsigned  short int ad2Buff[MAX_AD2_BUFF_LENGTH];
static  unsigned  short int ad3Buff[MAX_AD3_BUFF_LENGTH];
static  unsigned  short int advalue[AD_CHANNEL_NUM_MAX];

void app_start_multi_channel_adc(void)
{  
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,4000);
  HAL_TIM_OC_Start(&htim1,TIM_CHANNEL_2);//trigger 
  HAL_TIM_Base_Start(&htim1);//trigger
  HAL_ADC_Start_DMA(&hadc1,(unsigned int*)ad1Buff,MAX_AD1_BUFF_LENGTH);  
  HAL_ADC_Start_DMA(&hadc2,(unsigned int*)ad2Buff,MAX_AD2_BUFF_LENGTH);  
}
/**
  * @brief filter
  * @param void
  * @note   均值滤波
  * @retval None
  */
void filter_ad(ADC_HandleTypeDef  *hadc)
{
  unsigned int sum[4]={0},i;  	
  if(hadc->Instance==ADC1)
  {
    for(i=0;i<(MAX_AD1_BUFF_LENGTH>>2);i++)
    {   
      sum[0]+=ad1Buff[i*4];
      sum[1]+=ad1Buff[i*4+1];
      sum[2]+=ad1Buff[i*4+2];
      sum[3]+=ad1Buff[i*4+3];     
    } 
    advalue[0]=(unsigned short int)(sum[0]>>4);        
    advalue[1]=(unsigned short int)(sum[1]>>4);  
    advalue[2]=(unsigned short int)(sum[2]>>4); 
    advalue[3]=(unsigned short int)(sum[3]>>4);       
  }
  if(hadc->Instance==ADC2)
  {
    for( i=0;i<(MAX_AD2_BUFF_LENGTH>>1);i++)
    {   
      sum[0]+=ad2Buff[i*2];
      sum[1]+=ad2Buff[i*2+1];         
    } 
    advalue[4]=(unsigned short int)(sum[0]>>4);        
    advalue[5]=(unsigned short int)(sum[1]>>4);           
  }
  if(hadc->Instance==ADC3)
  {
    for( i=0;i<MAX_AD3_BUFF_LENGTH;i++)
    {   
      sum[0]+=ad2Buff[i];
    } 
    advalue[6]=(unsigned short int)(sum[0]>>4);  
  }   
} 
/**
  * @brief NTC_T cal
  * @param void         
  * @note   B = (T1*T2)/(T2-T1) * ln(RT1/RT2)  ,T1=25+273.15;
  * 25/50:B=3380 ;                 25/80:B=3428 ; 25/85:B=3434 ; 25/100:B=3455 ;
  * -10:R=42.506k ;/50:R=4.917K ;  /80:R=1.669K ; /85:R=1.452k ;/100:R=0.974k ;
  * @retval T                
  */
 static float  NTC_T_cal( unsigned short int voltage)
 { 
    float ret,refR=2.0; //2.0k 
    unsigned short int refVoltage=3300;  
    double  T0,Tt,R0,Rt,B; 
    T0=25+273.15;  
    R0=10;//10KΩ  
    //B=TtT0*log(Rt/R0)/(T0-Tt);
    //Tt = (1.0/(log(Rt/R0)/B + 1/T0) )-273.15;
    Rt=(refVoltage*refR/voltage)-refR;//r=2k 
    if(Rt>4.917)      B=3380;//<50
    else if(Rt>1.669) B=3428;//<80
    else if(Rt>1.452) B=3434;//<85
    else if(Rt>0.974) B=3455;//<100
    else              B=3455;
    Tt = (1.0/(log(((refVoltage*refR/voltage)-refR)/R0)/B + 1/T0) )-273.15;
    ret=Tt;
    return ret;    
 }
/**
  * @brief NTC_T cal
  * @param void         
  * @note   B = (T1*T2)/(T2-T1) * ln(RT1/RT2)  ,T1=25+273.15;
  * 25/50:B=3380 ;                 25/80:B=3428 ; 25/85:B=3434 ; 25/100:B=3455 ;
  * -10:R=42.506k ;/50:R=4.917K ;  /80:R=1.669K ; /85:R=1.452k ;/100:R=0.974k ;
  * @retval T                
  */
 static float  NTC_TEC_T_cal( unsigned short int voltage)
 { 
    float ret,refR=28.0; //28.0k   
    unsigned short int refVoltage=3300; 
    double  T0,Tt,R0,Rt,B; 
    T0=25+273.15;  
    R0=10;//10KΩ  
    //B=TtT0*log(Rt/R0)/(T0-Tt);
    //Tt = (1.0/(log(Rt/R0)/B + 1/T0) )-273.15;
    //voltage=refVoltage*(refR/(refR+Rt))*((510+100))/510)-(0.1/1.02)*refVoltage;
    voltage=((voltage+(refVoltage/10.2))*51/61);
    Rt=refR*refVoltage/voltage-refR;//r=28k
    if(Rt>4.917)      B=3380;//<50
    else if(Rt>1.669) B=3428;//<80
    else if(Rt>1.452) B=3434;//<85
    else if(Rt>0.974) B=3455;//<100
    else              B=3455;
    Tt = (1.0/(log(((refVoltage*refR/voltage)-refR)/R0)/B + 1/T0) )-273.15;
    ret = Tt;
    return ret;    
 }
/**
  * @brief HAL_ADC_ConvCpltCallback
  * @param void
  * @note   
  * @retval None 
  */
 void  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
 { 
  filter_ad(hadc);   
 }
 
 /**
  * @brief app_get_adc_value
  * @param void         
  * @note   12 bit
  * @retval None
  */
void app_get_adc_value(unsigned char adChannel,float *vBuff)
{  
  unsigned int temp;	 
  if(adChannel==AD1_CH1_IBUS)
  {
    temp=(((advalue[AD1_CH1_IBUS]*AD_VREF_VOLTAGE)>>12));    
    *vBuff=temp*4.676;
  // DEBUG_PRINTF("iBus=%dmA %d\r\n", temp,advalue[AD1_CH1_IBUS]);
  }
  else if(adChannel==AD1_CH2_VBUS)
  {
    temp=((advalue[AD1_CH2_VBUS]*AD_VREF_VOLTAGE)>>12)*11;
    *vBuff=temp*1.00840;
   //DEBUG_PRINTF("vBus=%dmV ad=%d\r\n", temp,advalue[AD1_CH2_VBUS]);
  }
  else if(adChannel==AD1_CH3_I_TEC)
  {
    temp=((advalue[AD1_CH3_I_TEC]*AD_VREF_VOLTAGE)>>12);
    //I=((Vso-Voff)/Adv*Rsence); Adv=20;Rsence=0.01;
    if(temp<901) temp=901;//VOFF =900mV  
    *vBuff=temp*5.0-4505;//i_tec   
   //DEBUG_PRINTF("iTec=%dmA ad=%d\r\n", temp,advalue[AD1_CH3_I_TEC]);
  } 
  else if(adChannel==AD1_CH4_I980)
  {
    temp=((advalue[AD1_CH4_I980]*AD_VREF_VOLTAGE)>>12)*11;
    *vBuff=temp*1.00359621978757;//i_980
   //DEBUG_PRINTF("iLaser980=%dmV ad=%04x\r\n", temp,advalue[AD1_CH4_I980]);
  } 
  else if(adChannel==AD2_CH3_NTC_MOTOR_TEMPRATURE)
  {
    temp=((advalue[AD2_CH3_NTC_MOTOR_TEMPRATURE]*AD_VREF_VOLTAGE)>>12);
    *vBuff=NTC_T_cal(temp);//NTC voltage VALUE
   //DEBUG_PRINTF("NTC=%dmv ad=%d\r\n",temp,advalue[AD2_CH3_NTC_MOTOR_TEMPRATURE]);
  } 
  else if(adChannel==AD2_CH4_TEC_TEMPRATURE)
  {
    temp=((advalue[AD2_CH4_TEC_TEMPRATURE]*AD_VREF_VOLTAGE)>>12);
    *vBuff=NTC_TEC_T_cal(temp);//tec_temprature  NTC   
   //DEBUG_PRINTF("tec_ntc=%dmV ad=%d\r\n", temp,advalue[AD2_CH4_TEC_TEMPRATURE]);
  }   
  else if(adChannel==AD3_CH1_ENERGE_FEEDBACK)
  {
    temp=((advalue[AD3_CH1_ENERGE_FEEDBACK]*AD_VREF_VOLTAGE)>>12);
    *vBuff=temp*1.00359621978757;//energe
   //DEBUG_PRINTF("energe=%dmV ad=%04x\r\n", temp,advalue[AD3_CH1_ENERGE_FEEDBACK]);
  }   
}
/* USER CODE END 1 */
