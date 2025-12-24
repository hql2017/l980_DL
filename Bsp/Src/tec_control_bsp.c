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
  * @brief tmc_init
  * @param  void
  * @note   
  * @retval None
  */
 void tmc_init(void)
 {
       	 
 } 

  /**
  * @brief tec_pwm_set
  * @param  void
  * @note   outVoltage
  * @retval None
  */
 void tec_pwm_set(unsigned short int outVoltage)
 {
	 if(outVoltage!=0)
  {    
		unsigned int timeUs;
		unsigned short int period;
		//check freq,timer3 1M clock freq		
    //period=10000000/100000;
		period=100;
		__HAL_TIM_SetAutoreload(&htim16,period-1);//freq =100k
		//duty 1%  100%; 0% close	
		timeUs=period /2;	//duty 50%
		__HAL_TIM_SetCompare(&htim20,TIM_CHANNEL_3,timeUs-1);
		HAL_TIM_PWM_Start(&htim20,TIM_CHANNEL_3);
  }
  else
  {
    HAL_TIM_PWM_Stop(&htim20,TIM_CHANNEL_3);
  }	
	 
 }