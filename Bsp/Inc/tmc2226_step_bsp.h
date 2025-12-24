/*
 *tmc2226_step_bsp.h
 *
 *  Created on: Dec 24, 2025
 *      Author: Hql2017
 */
#ifndef TMC2226_STEP_BSP_H_
#define TMC2226_STEP_BSP_H_

#define CONTINUOUS_STEPS_COUNT   0xFFFFFFFFUL //
#define TMC_WATER_OUT_DIR_VALUE   0//

void tmc2226_init(void);
void app_steps_pulse(unsigned int steps);
void tmc2226_start(unsigned char dir,unsigned short int spdLevel);
unsigned int tmc2226_stop(void);

void tmc2226_step_pwm_set(unsigned  int speed);

#endif /* TMC2226_STEP_BSP_H_ */

