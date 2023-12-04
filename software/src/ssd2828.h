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


#define DIR_REG     0xB0

#define VICR1_REG   0xB1
    #define VICR1_VSA_POS   8U
    #define VICR1_VSA_MSK   0xFF00
    #define VICR1_HSA_POS   0U
    #define VICR1_HSA_MSK   0x00FF

#define VICR2_REG   0xB2
    #define VICR2_VBP_POS   8U
    #define VICR2_VBP_MSK   0xFF00
    #define VICR2_HBP_POS   0U
    #define VICR2_HBP_POS   0x00FF

#define VICR3_REG   0xB3
    #define VICR3_VFP_POS   8U
    #define VICR3_VFP_MSK   0xFF00
    #define VICR3_HFP_POS   0U
    #define VICR3_HBFP_POS   0x00FF

#define VICR4_REG   0xB4
    #define VICR4_HACT_POS   0U
    #define VICR4_HACT_MSK   0xFFFF

#define VICR5_REG   0xB5
    #define VICR5_VACT_POS   0U
    #define VICR5_VACT_MSK   0xFFFF

#define VICR6_REG   0xB6
    #define VICR6_VS_P_POS      15U
    #define VICR6_VS_P_MSK      0x8000
    #define VICR6_HS_P_POS      14U
    #define VICR6_HS_P_MSK      0x4000
    #define VICR6_PCLK_P_POS    13U
    #define VICR6_PCLK_P_MSK    0x2000
    #define VICR6_CBM_POS       8U
    #define VICR6_CBM_MSK       0x0100
    #define VICR6_NVB_POS       7U
    #define VICR6_NVB_MSK       0x0080
    #define VICR6_NVD_POS       6U
    #define VICR6_NVD_MSK       0x0040
    #define VICR6_BLLP_POS      5U
    #define VICR6_BLLP_MSK      0x0040
    #define VICR6_VCS_POS       4U
    #define VICR6_VCS_MSK       0x0010
    #define VICR6_VM_POS        2U
    #define VICR6_VM_MSK        0x000C
    #define VICR6_VPF_POS       0U
    #define VICR6_VPF_MSK       0x0003

#define CFGR_REG    0xB7
    #define CFGR_TXD_POS        11U
    #define CFGR_TXD_MSK        0x0800
    #define CFGR_LPE_POS        10U
    #define CFGR_LPE_MSK        0x0800
    #define CFGR_EOT_POS        9U
    #define CFGR_EOT_MSK        0x0800
    #define CFGR_ECD_POS        8U
    #define CFGR_ECD_MSK        0x0800
    #define CFGR_REN_POS        7U
    #define CFGR_REN_MSK        0x0800
    #define CFGR_DCS_POS        6U
    #define CFGR_DCS_MSK        0x0800
    #define CFGR_TXD_POS        5U
    #define CFGR_TXD_MSK        0x0800
    #define CFGR_TXD_POS        4U
    #define CFGR_TXD_MSK        0x0800
    #define CFGR_TXD_POS        3U
    #define CFGR_TXD_MSK        0x0800
    #define CFGR_TXD_POS        2U
    #define CFGR_TXD_MSK        0x0800
    #define CFGR_TXD_POS        1U
    #define CFGR_TXD_MSK        0x0800
    #define CFGR_TXD_POS        0U
    #define CFGR_TXD_MSK        0x0800

#define VCR_REG     0xB8
#define PCR_REG     0xB9
#define PLCR_REG    0xBA
#define CCR_REG     0xBB
#define PSCR1_REG   0xBC
#define PSCR2_REG   0xBD
#define PSCR3_REG   0xBE
#define PDR_REG     0xBF
#define OCR_REG     0xC0
#define MRSR_REG    0xC1
#define RDCR_REG    0xC2
#define ARSR_REG    0xC3
#define LCR_REG     0xC4
#define ICR_REG     0xC5
#define ECR_REG     0xC6
#define DAR1_REG    0xC7
#define DAR3_REG    0xC9
#define DAR4_REG    0xCA
#define DAR5_REG    0xCB
#define DAR6_REG    0xCC
#define ICR_REG     0xCD
#define DAR6_REG    0xCE
#define HTTR1_REG   0xCF
#define HTTR2_REG   0xD0
#define LTTR1_REG   0xD1
#define LTTR2_REG   0xD2
#define TSR_REG     0xD3
#define LRR_REG     0xD4
#define PLLR_REG    0xD5
#define TR_REG      0xD6
#define TECR_REG    0xD7
#define ACR1_REG    0xD8
#define ACR2_REG    0xD9
#define ACR3_REG    0xDA
#define ACR4_REG    0xDB
#define IOCR_REG    0xDC
#define VICR7_REG   0xDD
#define LCFR_REG    0xDE
#define DAR7_REG    0xDF
#define PUCR1_REG   0xE9
#define PUCR2_REG   0xE0
#define PUCR3_REG   0xE1
#define CBCR1_REG   0xE9
#define CBSR_REG    0xEA
#define ECR_REG     0xEB
#define VSDR_REG    0xEC
#define TMR_REG     0xED
#define TMR_REG     0xEE
#define GPIO1_REG   0xEF
#define GPIO2_REG   0xF0
#define DLYA01_REG  0xF1
#define DLYA23_REG  0xF2
#define DLYB01_REG  0xF3
#define DLYB23_REG  0xF4
#define DLYC01_REG  0xF5
#define DLYC23_REG  0xF6
#define ACR5_REG    0xF7
#define RR_REG      0xFF



void ssd2828_gpio_init();

void ssd2828_init();

void ssd2828_write_reg(uint8_t reg, uint16_t val);
void ssd2828_read_reg(uint8_t reg, uint16_t *val);

uint16_t ssd2828_get_id();


#endif 