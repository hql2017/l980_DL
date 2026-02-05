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
#include "stdlib.h"

#define  TEC_COOL_DIR  1
#define  TEC_HOT_DIR  0
  /************************************************************************//**
  * @brief tec开关
  * @param  flag-使能信号
  * @note   0，standby ；1 work
  * @retval None
  ****************************************************************************/
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
  *         freq:=timer20 10M clock freq
  *         pWM freq=100k
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
    //duty=(outVoltage*100/24)=outVoltage*25/6;
		//timeUs=period /2;	//duty 50% ,tset
    //timeUs=outVoltage*25/6;//100/24;
    timeUs=outVoltage*25/6;//100/24; 
    if(timeUs<1) timeUs=1;
    if(timeUs>100) timeUs=100;
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
  * @note   outVoltage:<0 降温； >0 升温 ； =0 停止
  * @retval None
  */
 void tec_start(  int outVoltage,unsigned  int runtimeMs)
 {
    if(outVoltage==0) tec_stop();
    else 
    {
      if(outVoltage<0)
      {
        tec_dir(TEC_COOL_DIR);
      }
      else tec_dir(TEC_HOT_DIR);             
      tec_pwm_set((unsigned short int )(abs(outVoltage)));       
      HAL_TIM_PWM_Start(&htim20,TIM_CHANNEL_3);  
      tec_en(1);
    }    
 }
   /**
  * @brief tec_pwm_set
  * @param  void
  * @note   outVoltage
  * @retval None
  */
 extern void app_tec_ctr_semo(void);
 void tec_stop(void)
 {
  tec_en(0);
  HAL_TIM_PWM_Stop(&htim20,TIM_CHANNEL_3);
  app_tec_ctr_semo();
 } 