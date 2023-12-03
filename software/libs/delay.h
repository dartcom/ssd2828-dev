#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>
#include "../system/stm32f4xx.h"



void delay_ms(uint32_t delay);

void delay_us(uint32_t delay);

void delay_init();

#endif 