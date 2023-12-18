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
    #define VICR1_VSA_POS       8U
    #define VICR1_VSA_MSK       0xFF00
    #define VICR1_HSA_POS       0U
    #define VICR1_HSA_MSK       0x00FF

#define VICR2_REG   0xB2
    #define VICR2_VBP_POS       8U
    #define VICR2_VBP_MSK       0xFF00
    #define VICR2_HBP_POS       0U
    #define VICR2_HBP_MSK       0x00FF

#define VICR3_REG   0xB3
    #define VICR3_VFP_POS       8U
    #define VICR3_VFP_MSK       0xFF00
    #define VICR3_HFP_POS       0U
    #define VICR3_HFP_MSK      0x00FF

#define VICR4_REG   0xB4
    #define VICR4_HACT_POS      0U
    #define VICR4_HACT_MSK      0xFFFF

#define VICR5_REG   0xB5
    #define VICR5_VACT_POS      0U
    #define VICR5_VACT_MSK      0xFFFF

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
    #define VICR6_BLLP_MSK      0x0020
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
    #define CFGR_LPE_MSK        0x0400
    #define CFGR_EOT_POS        9U
    #define CFGR_EOT_MSK        0x0200
    #define CFGR_ECD_POS        8U
    #define CFGR_ECD_MSK        0x0100
    #define CFGR_REN_POS        7U
    #define CFGR_REN_MSK        0x0080
    #define CFGR_DCS_POS        6U
    #define CFGR_DCS_MSK        0x0040
    #define CFGR_CSS_POS        5U
    #define CFGR_CSS_MSK        0x0020
    #define CFGR_HCLK_POS       4U
    #define CFGR_HCLK_MSK       0x0010
    #define CFGR_VEN_POS        3U
    #define CFGR_VEN_MSK        0x0008
    #define CFGR_SLP_POS        2U
    #define CFGR_SLP_MSK        0x0004
    #define CFGR_CKE_POS        1U
    #define CFGR_CKE_MSK        0x0002
    #define CFGR_HS_POS         0U
    #define CFGR_HS_MSK         0x0001

#define VCR_REG     0xB8
    #define VCR_VCM_POS         6U
    #define VCR_VCM_MSK         0x00C0
    #define VCR_VCE_POS         4U
    #define VCR_VCE_MSK         0x0030
    #define VCR_VC2_POS         2U
    #define VCR_VC2_MSK         0x000C
    #define VCR_VC1_POS         0U
    #define VCR_VC1_MSK         0x0003

#define PCR_REG     0xB9
    #define PCR_SYSD_POS        14U
    #define PCR_SYSD_MSK        0xC000
    #define PCR_SYS_DIS_POS     13U
    #define PCR_SYS_DIS_MSK     0x2000
    #define PCR_PEN_POS         0U
    #define PCR_PEN_MSK         0x0001

#define PLCR_REG    0xBA
    #define PLCR_FR_POS         14U
    #define PLCR_FR_MSK         0xC000
    #define PLCR_MS_POS         8U
    #define PLCR_MS_MSK         0x1F00
    #define PLCR_NS_POS         0U
    #define PLCR_NS_MSK         0x00FF

#define CCR_REG     0xBB
    #define CCR_LPD_POS         0U
    #define CCR_LPD_MSK         0x003F

#define PSCR1_REG   0xBC
    #define PSCR1_TDC_POS       0U
    #define PSCR1_TDC_MSK       0xFFFF

#define PSCR2_REG   0xBD
    #define PSCR2_TDC_POS       0U
    #define PSCR2_TDC_MSK       0xFFFF

#define PSCR3_REG   0xBE
    #define PSCR3_PST_POS       0U
    #define PSCR3_PST_MSK       0x1FFF

#define PDR_REG     0xBF
    #define PDR_GPD_POS                     0U
    #define PDR_GPD_MSK                     0xFFFF

#define OCR_REG     0xC0
    #define OCR_RST_POS                     8U
    #define OCR_RST_MSK                     0x0100
    #define OCR_COP_POS                     0U
    #define OCR_COP_MSK                     0x0001

#define MRSR_REG    0xC1
    #define MRSR_MRS_POS                    0U
    #define MRSR_MRS_MSK                    0xFFFF

#define RDCR_REG    0xC2
    #define RDCR_RDC_POS                    0U
    #define RDCR_RDC_MSK                    0xFFFF

#define ARSR_REG    0xC3
    #define ARSR_AR_POS                     0U
    #define ARSR_AR_MSK                     0xFFFF

#define LCR_REG     0xC4
    #define LCR_IBC_POS                     5U
    #define LCR_IBC_MSK                     0x0020
    #define LCR_RT_POS                      4U
    #define LCR_RT_MSK                      0x0010
    #define LCR_RTB_POS                     3U
    #define LCR_RTB_MSK                     0x0008
    #define LCR_FBC_POS                     2U
    #define LCR_FBC_MSK                     0x0004
    #define LCR_FBT_POS                     1U
    #define LCR_FBT_MSK                     0x0002
    #define LCR_FBW_POS                     0U
    #define LCR_FBW_MSK                     0x0001

#define ICR_REG     0xC5
    #define ICR_CBEE_POS                    15U
    #define ICR_CBEE_MSK                    0x8000
    #define ICR_CBAE_POS                    14U
    #define ICR_CBAE_MSK                    0x4000
    #define ICR_MLEE_POS                    9U
    #define ICR_MLEE_MSK                    0x0200
    #define ICR_MLAE_POS                    8U
    #define ICR_MLAE_MSK                    0x0100
    #define ICR_PLSE_POS                    7U
    #define ICR_PLSE_MSK                    0x0080
    #define ICR_LPTOE_POS                   6U
    #define ICR_LPTOE_MSK                   0x0040
    #define ICR_HSTOE_POS                   5U
    #define ICR_HSTOE_MSK                   0x0020
    #define ICR_ARRE_POS                    3U
    #define ICR_ARRE_MSK                    0x0008
    #define ICR_BTARE_POS                   2U
    #define ICR_BTARE_MSK                   0x0004
    #define ICR_POE_POS                     1U
    #define ICR_POE_MSK                     0x0002
    #define ICR_RDRE_POS                    0U
    #define ICR_RDRE_MSK                    0x0001
  
#define ISR_REG     0xC6
    #define ISR_CBE_POS                     15U
    #define ISR_CBE_MSK                     0x80000
    #define ISR_CBA_POS                     14U
    #define ISR_CBA_MSK                     0x4000
    #define ISR_CST_POS                     11U
    #define ISR_CST_MSK                     0x0800
    #define ISR_DST_POS                     10U
    #define ISR_DST_MSK                     0x0400
    #define ISR_MLE_POS                     9U
    #define ISR_MLE_MSK                     0x0200
    #define ISR_MLA_POS                     8U
    #define ISR_MLA_MSK                     0x0100
    #define ISR_PLS_POS                     7U
    #define ISR_PLS_MSK                     0x0080
    #define ISR_LPTO_POS                    6U
    #define ISR_LPTO_MSK                    0x0040
    #define ISR_HSTO_POS                    5U
    #define ISR_HSTO_MSK                    0x0020
    #define ISR_ATR_POS                     4U
    #define ISR_ATR_MSK                     0x0010
    #define ISR_ARR_POS                     3U
    #define ISR_ARR_MSK                     0x0008
    #define ISR_BTAR_POS                    2U
    #define ISR_BTAR_MSK                    0x0004
    #define ISR_PO_POS                      1U
    #define ISR_PO_MSK                      0x0002
    #define ISR_RDR_POS                     0U
    #define ISR_RDR_MSK                     0x0001

#define ESR_REG     0xC7
    #define ESR_CRCE_POS                    10U
    #define ESR_CRCE_MSK                    0x0400
    #define ESR_ECCE2_POS                   9U
    #define ESR_ECCE2_MSK                   0x0200
    #define ESR_ECCE1_POS                   8U
    #define ESR_ECCE1_MSK                   0x0100
    #define ESR_SO_POS                      7U
    #define ESR_SO_MSK                      0x0080
    #define ESR_MLO_POS                     4U
    #define ESR_MLO_MSK                     0x0010
    #define ESR_CONT_POS                    2U
    #define ESR_CONT_MSK                    0x0004
    #define ESR_VMM_POS                     0U
    #define ESR_VMM_MSK                     0x0001

#define DAR1_REG    0xC9
    #define DAR1_HZD_POS                    8U
    #define DAR1_HZD_MSK                    0xFF00
    #define DAR1_HPD_POS                    0U
    #define DAR1_HPD_MSK                    0x00FF

#define DAR2_REG    0xCA
    #define DAR2_CXD_POS                    8U
    #define DAR2_CZD_MSK                    0xFF00
    #define DAR2_CPD_POS                    0U
    #define DAR2_CPD_MSK                    0x00FF

#define DAR3_REG    0xCB
    #define DAR3_CPED_POS                   8U
    #define DAR3_CPED_MSK                   0xFF00
    #define DAR3_CPTD_POS                   0U
    #define DAR3_CPTD_MSK                   0x00FF

#define DAR4_REG    0xCC
    #define DAR4_CTD_POS                    8U
    #define DAR4_CTD_MSK                    0xFF00
    #define DAR4_HTD_POS                    0U
    #define DAR4_HTD_MSK                    0x00FF

#define DAR5_REG    0xCD
    #define DAR5_WUD_POS                    0U
    #define DAR5_WUD_MSK                    0xFFFF

#define DAR6_REG    0xCE
    #define DAR6_TGO_POS                    8U
    #define DAR6_TGO_MSK                    0x0F00
    #define DAR6_TGET_POS                   0U
    #define DAR6_TGET_MSK                   0x000F

#define HTTR1_REG   0xCF
    #define HTTR1_HTT_POS                   0U
    #define HTTR1_HTT_MSK                   0xFFFF

#define HTTR2_REG   0xD0
    #define HTTR2_HTT_POS                   0U
    #define HTTR2_HTT_MSK                   0xFFFF

#define LTTR1_REG   0xD1
    #define LTTR1_LRT_POS                   0U
    #define LTTR1_LRT_MSK                   0xFFFF

#define LTTR2_REG   0xD2
    #define LTTR2_LRT_POS                   0U
    #define LTTR2_LRT_MSK                   0xFFFF

#define TSR_REG     0xD3
    #define TSR_TER_POS                     0U
    #define TSR_TER_MSK                     0x0001

#define LRR_REG     0xD4
    #define LRR_RRA_POS                     0U
    #define LRR_RRA_MSK                     0x00FF

#define PLLR_REG    0xD5
    #define PLRR_LOCK_POS                   0U
    #define PLRR_LOCK_MSK                   0xFFFF

#define TR_REG      0xD6
    #define TR_TM_FL0_POS                   14U
    #define TR_TM_FL0_MSK                   0xC000
    #define TR_EIC_POS                      9U
    #define TR_EIC_MSK                      0x3E00
    #define TR_FLM_POS                      8U
    #define TR_FLM_MSK                      0x0100
    #define TR_PNB_POS                      2U
    #define TR_PNB_MSK                      0x00FC
    #define TR_END_POS                      1U
    #define TR_END_MSK                      0x0002
    #define TR_CO_POS                       0U
    #define TR_CO_MSK                       0x0001

#define TECR_REG    0xD7
    #define TECR_TEC_POS                    0U
    #define TECR_TEC_MSK                    0xFFFF

#define ACR1_REG    0xD8
    #define ACR1_D3_DELAY_SEL3_2_POS        14U
    #define ACR1_D3_DELAY_SEL3_2_MSK        0xC000
    #define ACR1_D1_DELAY_SEL_POS           8U
    #define ACR1_D1_DELAY_SEL_MSK           0x3F00
    #define ACR1_D3_DELAY_SEL1_0_POS        6U
    #define ACR1_D3_DELAY_SEL1_0_MSK        0x00CF
    #define ACR1_D0_DELAY_SEL_POS           0U
    #define ACR1_D0_DELAY_SEL_MSK           0x003F

#define ACR2_REG    0xD9
    #define ACR2_HSTX_Z_POS                 14U
    #define ACR2_HSTX_Z_MSK                 0xC000
    #define ACR2_LPTXDS_POS                 11U
    #define ACR2_LPTXDS_MSK                 0x3800
    #define ACR2_HSTX_DS_POS                8U
    #define ACR2_HSTX_DS_MSK                0x0700
    #define ACR2_D3_DELAY_SEL5_4_POS        6U
    #define ACR2_D3_DELAY_SEL5_4_MSK        0x00C0
    #define ACR2_D2_DELAY_SEL_POS           0U
    #define ACR2_D2_DELAY_SEL_MSK           0x003F
    
#define ACR3_REG    0xDA
    #define ACR3_TLFT1_POS                  15U
    #define ACR3_TLFT1_MSK                  0x8000
    #define ACR3_TLFT0_POS                  14U
    #define ACR3_TLFT0_MSK                  0x4000
    #define ACR3_PREEM_SEL_POS              11U
    #define ACR3_PREEM_SEL_MSK              0x3800
    #define ACR3_PREEM_MOD_POS              9U
    #define ACR3_PREEM_MOD_MSK              0x0600
    #define ACR3_PREEM_E_POS                8U
    #define ACR3_PREEM_E_MSK                0x0100
    #define ACR3_THFT1_POS                  7U
    #define ACR3_THFT1_MSK                  0x0080
    #define ACR3_THFT0_POS                  6U
    #define ACR3_THFT0_MSK                  0x0040
    #define ACR3_TC_POS                     3U
    #define ACR3_TC_MSK                     0x0038
    #define ACR3_ISEL_POS                   0U
    #define ACR3_ISEL_MSK                   0x0007

#define ACR4_REG    0xDB
    #define ACR4_CLK_DELAY_SEL_POS          10U
    #define ACR4_CLK_DELAY_SEL_MSK          0xFC00
    #define ACR4_CKF_POS                    9U
    #define ACR4_CKF_MSK                    0x0200
    #define ACR4_TCI_POS                    5U
    #define ACR4_TCI_MSK                    0x00E0
    #define ACR4_ENLV_POS                   4U
    #define ACR4_ENLV_MSK                   0x0010
    #define ACR4_CD_EN_POS                  3U
    #define ACR4_CD_EN_MSK                  0x0008
    #define ACR4_GFFT1_POS                  2U
    #define ACR4_GFFT1_MSK                  0x0004
    #define ACR4_GFFT0_POS                  1U
    #define ACR4_GFFT0_MSK                  0x0002
    #define ACR4_GF_E_POS                   0U
    #define ACR4_GF_E_MSK                   0x0001

#define IOCR_REG    0xDC
    #define IOCR_IOT_POS                    1U
    #define IOCR_IOT_MSK                    0x0006
    #define IOCR_IAS_POS                    0U
    #define IOCR_IAS_MSK                    0x0001

#define VICR7_REG   0xDD
    #define VICR7_VBN_POS                   4U
    #define VICR7_VBN_MSK                   0x000F
    #define VICR7_VFN_POS                   0U
    #define VICR7_VFN_MSK                   0x00F0

#define LCFR_REG    0xDE
    #define LCFR_LS_POS                     0U
    #define LCFR_LS_MSK                     0x0003

#define DAR7_REG    0xDF
    #define DAR7_HED_POS                    0U
    #define DAR7_HED_MSK                    0x001F

#define PUCR1_REG   0xE0
    #define PUCR1_XTAL_PULL_POS             14U
    #define PUCR1_XTAL_PULL_MSK             0xC000
    #define PUCR1_PS4_PULL_POS              12U
    #define PUCR1_PS4_PULL_MSK              0x3000
    #define PUCR1_PS3_PULL_POS              10U
    #define PUCR1_PS3_PULL_MSK              0x0C00
    #define PUCR1_PS2_PULL_POS              8U
    #define PUCR1_PS2_PULL_MSK              0x0300
    #define PUCR1_PS1_PULL_POS              6U
    #define PUCR1_PS1_PULL_MSK              0x00C0
    #define PUCR1_PS0_PULL_POS              4U
    #define PUCR1_PS0_PULL_MSK              0x0030
    #define PUCR1_IS_PULL_POS               2U
    #define PUCR1_IS_PULL_MSK               0x000C
    #define PUCR1_MR_PULL_POS               0U
    #define PUCR1_MR_PULL_MSK               0x0003

#define PUCR2_REG   0xE1
    #define PUCR2_DEN_PULL_POS              14U
    #define PUCR2_DEN_PULL_MSK              0xC000
    #define PUCR2_HS_PULL_POS               12U
    #define PUCR2_HS_PULL_MSK               0x3000
    #define PUCR2_PC_PULL_POS               10U
    #define PUCR2_PC_PULL_MSK               0x0C00
    #define PUCR2_VS_PULL_POS               8U
    #define PUCR2_VS_PULL_MSK               0x0300
    #define PUCR2_DH_PULL_POS               6U
    #define PUCR2_DH_PULL_MSK               0x00C0
    #define PUCR2_DM_PULL_POS               4U
    #define PUCR2_DM_PULL_MSK               0x0030
    #define PUCR2_DL_PULL_POS               2U
    #define PUCR2_DL_PULL_MSK               0x000C
    #define PUCR2_CSX_PULL_POS              0U
    #define PUCR2_CSX_PULL_MSK              0x0003

#define PUCR3_REG   0xE2
    #define PUCR3_SDI_PULL_POS              8U
    #define PUCR3_SDI_PULL_MSK              0x0300
    #define PUCR3_SCK_PULL_POS              6U
    #define PUCR3_SCK_PULL_MSK              0x00C0
    #define PUCR3_SDC_PULL_POS              4U
    #define PUCR3_SDC_PULL_MSK              0x0030
    #define PUCR3_SHUT_PULL_POS             2U
    #define PUCR3_SHUT_PULL_MSK             0x000C
    #define PUCR3_CM_PULL_POS               0U
    #define PUCR3_CM_PULL_MSK               0x0003

#define CBCR1_REG   0xE9
    #define CBCR1_WDBV_POS                  8U
    #define CBCR1_WDBV_MSK                  0xFF00
    #define CBCR1_GAM18_POS                 6U
    #define CBCR1_GAM18_MSK                 0x0040
    #define CBCR1_BL_POS                    5U
    #define CBCR1_BL_MSK                    0x0020
    #define CBCR1_DD_POS                    4U
    #define CBCR1_DD_MSK                    0x0010
    #define CBCR1_BCTR_POS                  3U
    #define CBCR1_BCTR_MSK                  0x00088023
    #define CBCR1_CABC_EN_POS               0U
    #define CBCR1_CABC_EN_MSK               0x0003

#define CBCR2_REG   0xEA
    #define CBCR2_PWM_PS_POS                12U
    #define CBCR2_PWM_PS_MSK                0xF000
    #define CBCR2_BCD_PS_POS                8U
    #define CBCR2_BCD_PS_MSK                0x0F00
    #define CBCR2_CABC_MB_POS               0U
    #define CBCR2_CABC_MB_MSK               0x00FF

#define CBSR_REG    0xEB
    #define CBSR_VGA_SEL_POS                12U
    #define CBSR_VGA_SEL_MSK                0xF000
    #define CBSR_BCL_POS                    1U
    #define CBSR_BCL_MSK                    0x0100
    #define CBSR_RDBV_EN_POS                0U
    #define CBSR_RDBV_EN_MSK                0x00FF

#define ECR_REG     0xEC
    #define ECR_ENC_LW_POS                  4U
    #define ECR_ENC_LW_MSK                  0xFFF0
    #define ECR_ENC_MODE_POS                1U
    #define ECR_ENC_MODE_MSK                0x0002
    #define ECR_ENC_EN_POS                  0U
    #define ECR_ENC_EN_MSK                  0x0001

#define VSDR_REG    0xED
    #define VSDR_VSD_POS                    8U
    #define VSDR_VSD_MSK                    0xFF00
    #define VSDR_HSD_POS                    0U
    #define VSDR_HSD_MSK                    0x00FF

#define TMR_REG     0xEE
    #define TMR_TRIM_DONE_POS               15U
    #define TMR_TRIM_DONE_MSK               0xFF00
    #define TMR_TRIM_PASS_POS               14U
    #define TMR_TRIM_PASS_MSK               0x00FF
    #define TMR_XORC_DONE_POS               13U
    #define TMR_XORC_DONE_MSK               0xFF00
    #define TMR_XORC_SEL_POS                12U
    #define TMR_XORC_SEL_MSK                0x00FF
    #define TMR_XORC_EN_POS                 11U
    #define TMR_XORC_EN_MSK                 0xFF00
    #define TMR_VBIST_SRT_POS               10U
    #define TMR_VBIST_SRT_MSK               0x00FF
    #define TMR_VBIST_EN_POS                9U
    #define TMR_VBIST_EN_MSK                0xFF00
    #define TMR_TRIM_EN_POS                 8U
    #define TMR_TRIM_EN_MSK                 0x00FF
    #define TMR_TRIM_CMD_POS                4U
    #define TMR_TRIM_CMD_MSK                0x00F
    #define TMR_TRIM_CMD_XORC_FILTER_POS    0U
    #define TMR_TRIM_CMD_XORC_FILTER_MSK    0x000F

#define GPIO1_REG   0xEF
    #define GPIO1_GPIO1_STAT_POS            15U
    #define GPIO1_GPIO1_STAT_MSK            0x8000
    #define GPIO1_GPIO1_CTR_POS             8U
    #define GPIO1_GPIO1_CTR_MSK             0x7F00
    #define GPIO1_GPIO0_STAT_POS            7U
    #define GPIO1_GPIO0_STAT_MSK            0x0080
    #define GPIO1_GPIO0_CTR_POS             0U
    #define GPIO1_GPIO0_CTR_MSK             0x007F

#define GPIO2_REG   0xF0
    #define GPIO2_GPIO2_STAT_POS            7U
    #define GPIO2_GPIO2_STAT_MSK            0x0080
    #define GPIO2_GPIO2_CTR_POS             0U
    #define GPIO2_GPIO2_CTR_MSK             0x007F

#define DLYA01_REG  0xF1
    #define DLYA01_DELAY_A_1_POS            8U
    #define DLYA01_DELAY_A_1_MSK            0x3F00
    #define DLYA01_DELAY_A_0_POS            0U
    #define DLYA01_DELAY_A_0_MSK            0x003F

#define DLYA23_REG  0xF2
    #define DLYA23_DELAY_A_3_POS            8U
    #define DLYA23_DELAY_A_3_MSK            0x3F00
    #define DLYA23_DELAY_A_2_POS            0U
    #define DLYA23_DELAY_A_2_MSK            0x003F

#define DLYB01_REG  0xF3
    #define DLYB01_DELAY_B_1_POS            8U
    #define DLYB01_DELAY_B_1_MSK            0x3F00
    #define DLYB01_DELAY_B_0_POS            0U
    #define DLYB01_DELAY_B_0_MSK            0x003F

#define DLYB23_REG  0xF4
    #define DLYB23_DELAY_B_3_POS            8U
    #define DLYB23_DELAY_B_3_MSK            0x3F00
    #define DLYB23_DELAY_B_2_POS            0U
    #define DLYB23_DELAY_B_2_MSK            0x003F

#define DLYC01_REG  0xF5
    #define DLYC01_DELAY_C_1_POS            8U
    #define DLYC01_DELAY_C_1_MSK            0x3F00
    #define DLYC01_DELAY_C_0_POS            0U
    #define DLYC01_DELAY_C_0_MSK            0x003F

#define DLYC23_REG  0xF6
    #define DLYC23_DELAY_C_3_POS            8U
    #define DLYC23_DELAY_C_3_MSK            0x3F00
    #define DLYC23_DELAY_C_2_POS            0U
    #define DLYC23_DELAY_C_2_MSK            0x003F

#define ACR5_REG    0xF7
    #define ACR5_DEC_XOR_E_POS                7U
    #define ACR5_DEC_XOR_E_MSK                0x0080
    #define ACR5_DEC_FB_E_POS                6U
    #define ACR5_DEC_FB_E_MSK                0x0040
    #define ACR5_D3_FB_E_POS                5U
    #define ACR5_D3_FB_E_MSK                0x0020
    #define ACR5_D2_FB_E_POS                4U
    #define ACR5_D2_FB_E_MSK                0x0010
    #define ACR5_D1_FB_E_POS                3U
    #define ACR5_D1_FB_E_MSK                0x0008
    #define ACR5_D0_FB_E_POS                2U
    #define ACR5_D0_FB_E_MSK                0x0004
    #define ACR5_XOR_TUNE_EN_POS            1U
    #define ACR5_XOR_TUNE_EN_MSK            0x0002
    #define ACR5_REG_CTR_POS                0U
    #define ACR5_REG_CTR_MSK                0x0001

#define RR_REG      0xFF
    #define RR_RD_POS                       0U
    #define RR_RD_MSK                       0xFFFF




void ssd2828_gpio_init();

void ssd2828_init();

void ssd2828_SPI_write(uint8_t *buffer, uint32_t len);

void ssd2828_SPI_read(uint8_t *buffer, uint32_t len);

void ssd2828_SPI_write_reg(uint8_t reg, uint16_t val);

void ssd2828_SPI_read_reg(uint8_t reg, uint16_t *val);

uint16_t ssd2828_get_id();

void ssd2828_MIPI_write_long_generic(uint8_t reg, uint8_t* data, uint32_t len);

void ssd2828_MIPI_write_long_DCS(uint8_t reg, uint16_t* data, uint32_t len);

void ssd2828_MIPI_write_short_generic(uint8_t reg,uint16_t data,int len);
void ssd2828_MIPI_write_short_DCS(uint8_t reg,uint16_t data,int len);


#endif 