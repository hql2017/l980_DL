
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

extern  void StartDefaultTask(void *argument);
extern  void motorTask02(void *argument);
extern  void CANopenTask03(void *argument);
extern  void laserWorkTask04(void *argument);
extern  void laserProhotTask05(void *argument);

extern void laserWorkTimerCallback01(void *argument);
extern void tecRunCallback02(void *argument);

extern  void MX_FREERTOS_Init(void);