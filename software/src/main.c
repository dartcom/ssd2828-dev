#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "../system/stm32f4xx.h"

#include "../libs/delay.h"
#include "../libs/spi.h"

#include "ssd2828.h"
#include "clk.h"

#define _1V2_EN         GPIO_ODR_OD2
#define _1V8_EN         GPIO_ODR_OD1
#define _3V0_EN         GPIO_ODR_OD12
#define _3V3_EN         GPIO_ODR_OD0
#define _BACKLIGHT_EN   GPIO_ODR_OD13

#define _RST            GPIO_ODR_OD15

void gpio_init(){
    //enable GPIOC and GPIOA GPIOB GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    //set C13 to output
    GPIOC->MODER |= (0x1<<GPIO_MODER_MODER13_Pos);
    //C13 to push pull
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT13;
    //set C13 speed to low
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR13;
    //set C13 to LOW
    GPIOC->ODR |= GPIO_ODR_OD13;

    //set B0 B1 B2 B12 B13 as outputs
    GPIOB->MODER |= (0x1<<GPIO_MODER_MODER0_Pos) | (0x1<<GPIO_MODER_MODER1_Pos) | (0x1<<GPIO_MODER_MODER2_Pos) | (0x1<<GPIO_MODER_MODER12_Pos) | (0x1<<GPIO_MODER_MODER13_Pos) | (0x1<<GPIO_MODER_MODER15_Pos);
    //set to push pull
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT0 & ~GPIO_OTYPER_OT1 & ~GPIO_OTYPER_OT2 & ~GPIO_OTYPER_OT12 & ~GPIO_OTYPER_OT13 & ~GPIO_OTYPER_OT15; 
    //set speed to low
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR0 & ~GPIO_OSPEEDER_OSPEEDR1 & ~GPIO_OSPEEDER_OSPEEDR2 & ~GPIO_OSPEEDER_OSPEEDR12 & ~GPIO_OSPEEDER_OSPEEDR13  & ~GPIO_OSPEEDER_OSPEEDR15; 
    //set C13 to LOW
    GPIOB->ODR &= ~GPIO_ODR_OD0 & ~GPIO_ODR_OD1 & ~GPIO_ODR_OD2 & ~GPIO_ODR_OD12 & ~GPIO_ODR_OD13& ~GPIO_ODR_OD15;



}


int main(){
    uint16_t i = 0, tmp = 0;
    SystemClock_Config();
    gpio_init();
    delay_init();
    spi_init();
    
    //enable 1V2
    GPIOB->ODR |= _1V2_EN;
    //wait 10ms
    delay_ms(10);
    //enable 3V3
    GPIOB->ODR |= _3V3_EN;
    //wait 20ms 
    delay_ms(20);
    

    delay_ms(100);
    ssd2828_init();



    //enable 1V8
    GPIOB->ODR |= _1V8_EN;
    //pull rst high
    GPIOB->ODR |= _RST;
    //wait 250ms 
    delay_ms(250);
    //pull rst low
    GPIOB->ODR &= ~_RST;
    //wait 10ms 
    delay_ms(10);
    //pull rst high
    GPIOB->ODR |= _RST;
    //wait 10ms 
    delay_ms(10);
    //enable 3V0
    GPIOB->ODR |= _3V0_EN;
    //wait 10ms 
    delay_ms(100);

   


    tmp = (1<<CFGR_HCLK_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);

    ssd2828_SPI_write_reg(VCR_REG, tmp);

    ssd2828_SPI_write_reg(PCR_REG, tmp);

    ssd2828_SPI_write_reg(PLCR_REG, 0x0000);
    tmp = (4U<<PLCR_NS_POS) | (1U<<PLCR_MS_POS);    //target 96Mbps
    ssd2828_SPI_write_reg(PLCR_REG, tmp);

    ssd2828_SPI_write_reg(CCR_REG, 0x0000);
    tmp = (1U<<CCR_LPD_POS);    //target 96Mbps
    ssd2828_SPI_write_reg(CCR_REG, tmp);

    tmp = (1<<PCR_PEN_POS);
    ssd2828_SPI_write_reg(PCR_REG, tmp);

    ssd2828_SPI_write_reg(LCFR_REG, tmp);

    tmp = (3U<<DAR1_HZD_POS) | (0U<<DAR1_HPD_POS);
    ssd2828_SPI_write_reg(DAR1_REG, tmp);

    delay_ms(100);

    tmp = (6U<<DAR2_CXD_POS) | (0U<<DAR2_CPD_POS);
    ssd2828_SPI_write_reg(DAR2_REG, tmp);

    tmp = (4U<<DAR3_CPTD_POS) | (1U<<DAR3_CPED_POS);
    ssd2828_SPI_write_reg(DAR3_REG, tmp);

    tmp = (2U<<DAR4_CTD_POS) | (3U<<DAR4_HTD_POS);
    ssd2828_SPI_write_reg(DAR4_REG, tmp);





    ssd2828_state_HS();
    

    ssd2828_MIPI_write_DCS(0x11,NULL, 0);
    delay_ms(240);


    ssd2828_MIPI_write_DCS(0x29,NULL, 0);
    delay_ms(240);


   
    
   
    while(1){
        GPIOB->ODR |= _BACKLIGHT_EN;
        if(i == 0x2828){
            GPIOC->ODR ^= GPIO_ODR_OD13;
            
        }
        i = ssd2828_get_id();
        delay_ms(100);
    }
    return 0;
}