#ifndef SSD2828_H
#define SSD2828_H

#include <stdint.h>
#include <stdio.h>
#include "../system/stm32f4xx.h"

#include "../libs/delay.h"
#include "../libs/spi.h"

#define RST     GPIO_ODR_OD3
#define DC      GPIO_ODR_OD1
#define SHUT    GPIO_ODR_OD2

void ssd2828_gpio_init();

void ssd2828_init();

void ssd2828_write_reg(uint8_t reg, uint16_t val);
void ssd2828_read_reg(uint8_t reg, uint16_t *val);

uint16_t ssd2828_get_id();


#endif 