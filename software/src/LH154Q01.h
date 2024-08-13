#ifndef LH154Q01_H
#define LH154Q01_H

#include <stdint.h>
#include <stdio.h>
#include "../system/stm32f4xx.h"

#include "../libs/delay.h"
#include "../libs/spi.h"
#include "ssd2828.h"

#include "DCS.h"

#define LH154Q01_V_PORT     GPIOB
#define LH154Q01_3V0        12U
#define LH154Q01_1V8        1U

#define LH154Q01_C_PORT     GPIOB
#define LH154Q01_BACKLIGHT  13U
#define LH154Q01_RST        14U


void LH154Q01_SSD2828_write_cfg();

void LH154Q01_init();

void LH154Q01_start();

void LH154Q01_init_GPIO();

void LH154Q01_backlight(uint8_t i);

#endif