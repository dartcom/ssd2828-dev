#ifndef VS035ZSM_H
#define VS035ZSM_H

#include <stdint.h>
#include <stdio.h>
#include "../system/stm32f4xx.h"

#include "../libs/delay.h"
#include "../libs/spi.h"
#include "ssd2828.h"

#define VS035ZSM_V_PORT     GPIOB
#define VS035ZSM_BIAS       15U
#define VS035ZSM_1V8        1U

#define VS035ZSM_C_PORT     GPIOA
#define VS035ZSM_BACKLIGHT  8U
#define VS035ZSM_RST        9U

#define VS035ZSM_H_ACTIVE   1440U
#define VS035ZSM_V_ACTIVE   1600U

#define VS035ZSM_HSYNC      2U
#define VS035ZSM_VSYNC      2U

#define VS035ZSM_HFP        60U
#define VS035ZSM_HBP        60U
#define VS035ZSM_VFP        10U
#define VS035ZSM_VBP        10U

void VS035ZSM_SSD2828_write_cfg();

void VS035ZSM_init();

void VS035ZSM_start();

void VS035ZSM_init_GPIO();

void VS035ZSM_init_BACKLIGHT();

void VS035ZSM_backlight(uint8_t i);
void VS035ZSM_vid();


#endif
