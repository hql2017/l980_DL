/*
 *tmc2226_step_bsp.h
 *
 *  Created on: Dec 24, 2025
 *      Author: Hql2017
 */
#ifndef TMC2226_STEP_BSP_H_
#define TMC2226_STEP_BSP_H_

#ifndef  MOTOR_ENCODER_USED
#define  MOTOR_ENCODER_USED 
#endif   

#define MAX_TRIP_STEPS_COUNT    3500//35mm
#define CONTINUOUS_STEPS_COUNT  80000// 0xFFFFFFFF //until
#define ONE_CIRCLE_CTR_STEPS_COUNT    100//
#define ENCODER_ONE_CIRCLE_COUNT      4000//4 multiple:
#define ENCODER_MAX_COUNT      ENCODER_ONE_CIRCLE_COUNT*35//       4000*35//35mm
#define  MOTOR_DIR_FORWARD      0
#define  MOTOR_DIR_REVERSE      1
#define  MOTOR_DIR_ZERO         2

void tmc2226_init(void);
void app_steps_pulse(unsigned int steps);
void tmc2226_start(unsigned char dir,unsigned short int spdLevel,unsigned  int steps);
extern void tmc2226_stop(void);
void tmc2226_en ( unsigned  char en );
void tmc2226_step_pwm_set(unsigned  int speed);

#endif /* TMC2226_STEP_BSP_H_ */

