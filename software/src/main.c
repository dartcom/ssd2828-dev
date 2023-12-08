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
    uint16_t i = 0;
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


    //enable EOT packet on end of HS transmission and check the CRC from the slave
    ssd2828_SPI_write_reg(CFGR_REG,(uint16_t)(1<<CFGR_EOT_POS) | (1<<CFGR_ECD_POS));
    
    //select 1 data lane mode
    ssd2828_SPI_write_reg(LCFR_REG,0x0000);
    

    ssd2828_SPI_write_reg(VCR_REG,0x0000);
    //set vertical and horizontal back porch
    ssd2828_SPI_write_reg(VICR2_REG,0x0214);
    //set vertical and horizontal front porch
    ssd2828_SPI_write_reg(VICR3_REG,0x0428);
    //set horizontal resolution
    ssd2828_SPI_write_reg(VICR5_REG,240);
    //set vertical resoution 
    ssd2828_SPI_write_reg(VICR4_REG,240);
    
    //power down the PLL to configure it
    ssd2828_SPI_write_reg(PCR_REG,0x0000);

    //set PLL FR to 00 
    //set MS to 1
    //set NS to 4 to target 96mhz
    ssd2828_SPI_write_reg(PLCR_REG,0x0000);
    ssd2828_SPI_write_reg(PLCR_REG,(3U<<PLCR_NS_POS));

    //set LP state speed
    ssd2828_SPI_write_reg(CCR_REG,0x0003);
    
    //enable PLL
    ssd2828_SPI_write_reg(PCR_REG,0x0001);





    //rgb mode
    ssd2828_SPI_write_reg(VICR6_REG,0b00000001|(0b00000000<<8));

    ssd2828_SPI_write_reg(LCFR_REG,0x0000);



    delay_ms(100);



    ssd2828_SPI_write_reg(TMR_REG,0x0600);

    ssd2828_SPI_write_reg(CFGR_REG,0x024b);


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

    
    ssd2828_MIPI_write_short_DCS(0x11,0x00,1);
    delay_ms(150);
    ssd2828_MIPI_write_short_DCS(0x29,0x00,1);
   
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