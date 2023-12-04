#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stdio.h>
#include "../system/stm32f4xx.h"


void spi_init();

void spi_exchange_8(uint8_t data_in, uint8_t *data_out);

#endif 