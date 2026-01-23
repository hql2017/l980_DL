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

#define MOTOR_MAX_TRIP_STEPS_COUNT    3200////max35mm,use 32mm
#define MOTOR_MAX_UM           MOTOR_MAX_TRIP_STEPS_COUNT*10
#define CONTINUOUS_STEPS_COUNT  80000// 0xFFFFFFFF //until
#define ONE_CIRCLE_CTR_STEPS_COUNT    100//
#define ENCODER_ONE_CIRCLE_COUNT      2000//2 multiple:
#define ENCODER_MAX_COUNT      ENCODER_ONE_CIRCLE_COUNT*32//       2000*35//35mm
#define  MOTOR_DIR_FORWARD      0
#define  MOTOR_DIR_REVERSE      1
#define  MOTOR_DIR_ZERO         2

void tmc2226_init(void);
void app_steps_pulse(unsigned int steps);
void app_motor_slide_position(unsigned char dir, unsigned  int distanceUm,unsigned char speed);
extern void tmc2226_stop(void);
void tmc2226_en ( unsigned  char en );
void tmc2226_step_pwm_set(unsigned  int speed);

unsigned int app_get_motor_real_position(void);

#endif /* TMC2226_STEP_BSP_H_ */

