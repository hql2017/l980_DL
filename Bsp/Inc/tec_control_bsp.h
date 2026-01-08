/*
 *tec_control_bsp.h
 *
 *  Created on: Dec 24, 2025
 *      Author: Hql2017
 */
#ifndef TEC_CONTROL_BSP_H_
#define TEC_CONTROL_BSP_H_
 void tec_init(void);
  void tec_pwm_set(unsigned short int outVoltage);
  void tec_stop(void);
#endif /* TEC_CONTROL_BSP_H_ */

