/**
 * @file tec_control_bsp.c
 *
 * @brief 
 * @warning This software tec control Drivers
 * 
 *
 */
#include "tec_control_bsp.h"
#include "main.h"
#include "usart.h"
#include "tim.h"

/**
  * @brief tec_pwm_set
  * @param  void
  * @note   outVoltage
  * @retval None
  */
 void tec_en(unsigned char flag)
 {
    if(flag==0) HAL_GPIO_WritePin(TEC_DRV8701_SLEEP_OUT_GPIO_Port, TEC_DRV8701_SLEEP_OUT_Pin, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(TEC_DRV8701_SLEEP_OUT_GPIO_Port, TEC_DRV8701_SLEEP_OUT_Pin, GPIO_PIN_SET);
 }
 /**
  * @brief tec_pwm_set
  * @param  void
  * @note   outVoltage
  * @retval None
  */
 void tec_dir(unsigned char flag)
 {
    if(flag==0) HAL_GPIO_WritePin(TEC_DRV8701_PHASE_CTR_OUT_GPIO_Port, TEC_DRV8701_PHASE_CTR_OUT_Pin, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(TEC_DRV8701_PHASE_CTR_OUT_GPIO_Port, TEC_DRV8701_PHASE_CTR_OUT_Pin, GPIO_PIN_SET);
 }
/**
 * 
  * @brief tec_init
  * @param  void
  * @note   
  * @retval None
  */
 void tec_init(void)
 {
    tec_en(0);
 } 
  /**
  * @brief tec_pwm_set
  * @param  void
  * @note   outVoltage
  *         freq:=timer20 1M clock freq
  *         duty:1~100
  * @retval None
  */
 void tec_pwm_set(unsigned short int outVoltage)
 {
	if(outVoltage!=0)
  {    
		unsigned int timeUs;
		unsigned short int period;
				
    //period=10000000/100000;
		period=100;
		__HAL_TIM_SetAutoreload(&htim20,period-1);//freq =100k
		//duty 1%  100%; 0% close	
		timeUs=period /2;	//duty 50%
		__HAL_TIM_SetCompare(&htim20,TIM_CHANNEL_3,timeUs-1);
		//HAL_TIM_PWM_Start(&htim20,TIM_CHANNEL_3);
  }
  else  
  {
    HAL_TIM_PWM_Stop(&htim20,TIM_CHANNEL_3);
  }		 
 }

   /**
  * @brief tec_pwm_set
  * @param  void
  * @note   outVoltage
  * @retval None
  */
 void tec_start(unsigned short int outVoltage,unsigned  int runtimeMs)
 {
    if(outVoltage==0) tec_stop();
    else 
    {
      tec_pwm_set(outVoltage);       
      HAL_TIM_PWM_Start(&htim20,TIM_CHANNEL_3);

      __HAL_TIM_SET_AUTORELOAD(&htim2,runtimeMs*10);//htim2 10K
      HAL_TIM_Base_Start_IT(&htim2);
    }    
 }
   /**
  * @brief tec_pwm_set
  * @param  void
  * @note   outVoltage
  * @retval None
  */
 void tec_stop(void)
 {
  HAL_TIM_PWM_Stop(&htim20,TIM_CHANNEL_3);
  HAL_TIM_Base_Stop_IT(&htim2);
 } 