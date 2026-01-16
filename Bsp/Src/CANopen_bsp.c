
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 

#include "CANopen_bsp.h"
#include "fdcan.h"
#include "tim.h"
#include "main.h"

#include "CO_app_STM32.h"
#include "OD.h"

void CANopen_init(void)
{
  CANopenNodeSTM32  p_canOpenNodeSTM32;
  p_canOpenNodeSTM32.CANHandle = &hfdcan1;
  p_canOpenNodeSTM32.HWInitFunction = MX_FDCAN1_Init;
  p_canOpenNodeSTM32.timerHandle = &htim17;
  p_canOpenNodeSTM32.desiredNodeID = 24;
  p_canOpenNodeSTM32.baudrate = 125;
  canopen_app_init(&p_canOpenNodeSTM32);     
}




