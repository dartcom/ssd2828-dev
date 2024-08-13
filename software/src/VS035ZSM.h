#ifndef VS035ZSM_H
#define VS035ZSM_H

#include <stdint.h>
#include <stdio.h>
#include "../system/stm32f4xx.h"

#include "../libs/delay.h"
#include "../libs/spi.h"
#include "ssd2828.h"

#include "DCS.h"


#define VS035ZSM_V_PORT         GPIOB
#define VS035ZSM_BIAS           15U
#define VS035ZSM_1V8            1U

#define VS035ZSM_C_PORT         GPIOA
#define VS035ZSM_BACKLIGHT      8U
#define VS035ZSM_RST            9U

#define VS035ZSM_H_ACTIVE       1440U
#define VS035ZSM_V_ACTIVE       1600U

#define VS035ZSM_HSYNC          40U
#define VS035ZSM_VSYNC          40U
#define VS035ZSM_HFP            80U
#define VS035ZSM_HBP            80U
#define VS035ZSM_VFP            40U
#define VS035ZSM_VBP            40U


#define VS035ZSM_PARTIAL_HSYNC      10U
#define VS035ZSM_PARTIAL_VSYNC      10U
#define VS035ZSM_PARTIAL_HFP        80U
#define VS035ZSM_PARTIAL_HBP        90U
#define VS035ZSM_PARTIAL_VFP        30U
#define VS035ZSM_PARTIAL_VBP        40U
#define VS035ZSM_PARTIAL_HACT       1440U
#define VS035ZSM_PARTIAL_VACT       864U

#define VS035ZSM_PARTIAL_RES_X      1440U
#define VS035ZSM_PARTIAL_RES_Y      864U

//#define VS035ZSM_PARTIAL_RES_X    (VS035ZSM_PARTIAL_HSYNC + VS035ZSM_PARTIAL_HFP + VS035ZSM_PARTIAL_HBP + VS035ZSM_PARTIAL_HACT)
//#define VS035ZSM_PARTIAL_RES_Y    (VS035ZSM_PARTIAL_VSYNC + VS035ZSM_PARTIAL_VFP + VS035ZSM_PARTIAL_VBP + VS035ZSM_PARTIAL_VACT)

void VS035ZSM_SSD2828_write_cfg();

void VS035ZSM_init();

void VS035ZSM_start();

void VS035ZSM_init_GPIO();

void VS035ZSM_init_BACKLIGHT();

void VS035ZSM_backlight(uint8_t i);

void VS035ZSM_set_partial();

#endif
