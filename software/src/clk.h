#ifndef CLK_H
#define CLK_H

#include <stdint.h>
#include "../system/stm32f4xx.h"

#define TIMEOUT 15000

uint32_t SystemClock_Config();

uint32_t HAL_RCC_GetSysClockFreq();


#endif 