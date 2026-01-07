
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 

#include "CANopen_bsp.h"
#include "fdcan.h"

void CANopen_init(void)
{
   MX_FDCAN1_Init();
}