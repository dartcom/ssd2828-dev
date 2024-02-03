#include "LH154Q01.h"

void LH154Q01_SSD2828_write_cfg(){
    //set lanes number to 1
    ssd2828_SPI_write_reg(LCFR_REG, 0x0000);
    //disable the PLL
    ssd2828_SPI_write_reg(PCR_REG, 0x0000);
    //set PLL freq to 336MHZ 
    ssd2828_SPI_write_reg(PLCR_REG, 0x0000|(1U << PLCR_FR_POS) | (1U << PLCR_MS_POS) | (14U << PLCR_NS_POS));
    //enable the PLL
    ssd2828_SPI_write_reg(PCR_REG, 0x0000|(1U << PCR_PEN_POS));
    //wait 2 ms to stabilize
    delay_ms(2);
    //set divider for low speed to 8MHZ
    ssd2828_SPI_write_reg(CCR_REG, 0x0000|(4U << CCR_LPD_POS));
    //set MIPI packet format
    //enable EOT DCS HS
    //disable Read Video
    ssd2828_SPI_write_reg(CFGR_REG, 0x0000 | (1U << CFGR_EOT_POS) | (1U << CFGR_DCS_POS)  | (1U << CFGR_HS_POS) );
    //set virtual channel 0x0000
    ssd2828_SPI_write_reg(VCR_REG, 0x0000);
}

void LH154Q01_init(){
    LH154Q01_init_GPIO();

    //enable 1V8
    LH154Q01_V_PORT->ODR |= (1U << LH154Q01_1V8);
    //pull rst high
    LH154Q01_C_PORT->ODR |= (1U << LH154Q01_RST);
    //wait 250ms 
    delay_ms(300);
    //pull rst low
    LH154Q01_C_PORT->ODR &= ~(1U << LH154Q01_RST);
    //wait 10ms 
    delay_ms(10);
    //pull rst high
    LH154Q01_C_PORT->ODR |= (1U << LH154Q01_RST);
    //wait 10ms 
    delay_ms(10);
    //enable 3V0
    LH154Q01_V_PORT->ODR |= (1U << LH154Q01_3V0);
    //wait 10ms 
    delay_ms(10);
}

void LH154Q01_start(){
    const uint8_t SLPOUT[]={1,0x11};
    const uint8_t DISPOFF[]={1,0x28};
    const uint8_t INV[]={1,0x21};
    const uint8_t DISPON[]={1,0x29};
    const uint8_t RGB[]={2,0x3A ,0x77};//24bit
    const uint8_t FLIP[]={2,0x36 ,0x08};//COLOR
    
   
    
    //SSD_WritePacket(SLPOUT);
    ssd2828_MIPI_write_DCS_short_np(0x11);
    delay_ms(100);
    //SSD_WritePacket(RGB);
    ssd2828_MIPI_write_DCS_short_p(0x3A, 0x77);
    //SSD_WritePacket(DISPON);
    ssd2828_MIPI_write_DCS_short_np(0x29);
    
}



void LH154Q01_init_GPIO(){
    //GPIO B already enabled
    //set 1V8 3V0 as outputs
    LH154Q01_V_PORT->MODER |=   (1U << (LH154Q01_1V8*2)) | (1U << (LH154Q01_3V0*2));
    //set to push pull
    LH154Q01_V_PORT->OTYPER &=  ~(1U << (LH154Q01_1V8*2)) & ~(1U << (LH154Q01_3V0*2));
    //set speed to low
    LH154Q01_V_PORT->OSPEEDR &= ~(3U << (LH154Q01_1V8*2)) & ~(3U << (LH154Q01_3V0*2));
    //set LOW
    LH154Q01_V_PORT->ODR &=     ~(1U << LH154Q01_1V8) & ~(1U << LH154Q01_3V0);


    //set RST BACKLIGHT as outputs
    LH154Q01_C_PORT->MODER |=   (1U << (LH154Q01_BACKLIGHT*2)) | (1U << (LH154Q01_RST*2));
    //set to push pull
    LH154Q01_C_PORT->OTYPER &=  ~(1U << (LH154Q01_BACKLIGHT*2)) & ~(1U << (LH154Q01_RST*2));
    //set speed to low
    LH154Q01_C_PORT->OSPEEDR &= ~(3U << (LH154Q01_BACKLIGHT*2)) & ~(3U << (LH154Q01_RST*2));
    //set LOW
    LH154Q01_C_PORT->ODR &=     ~(1U << LH154Q01_BACKLIGHT) & ~(1U << LH154Q01_RST);
}


void LH154Q01_backlight(uint8_t i){
    if(i == 1){
        LH154Q01_V_PORT->ODR |= (1U << LH154Q01_BACKLIGHT);
    }else{
        LH154Q01_V_PORT->ODR &= ~(1U << LH154Q01_BACKLIGHT);
    }
}

