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

#define _RST            GPIO_ODR_OD14

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


void fill_line(uint8_t r, uint8_t g, uint8_t b){
    uint32_t i;
    ssd2828_SPI_write_reg(PSCR1_REG, ((240*3)+1)&0x0FFF);
    ssd2828_SPI_write_cmd(PDR_REG);//0xBF
    ssd2828_SPI_write_data(0x3C);
    for(i = 0; i < 240; i++){
        ssd2828_SPI_write_data(r);
        ssd2828_SPI_write_data(g);
        ssd2828_SPI_write_data(b);
    }
}

void fill_screen(uint8_t r, uint8_t g, uint8_t b){
    uint32_t i;
    for(i = 0; i < 240; i++){
        fill_line(r,g,b);
    }
}

int main(){
    uint16_t i = 0, tmp = 0;
    const uint8_t DISPON[]={1,0x29};
    const uint8_t BLANK[]={1,0x19};
    SystemClock_Config();
    gpio_init();
    delay_init();
    spi_init();
    delay_ms(1500);
    
    //enable 1V2
    GPIOB->ODR |= _1V2_EN;
    //wait 10ms
    delay_ms(10);
    //enable 3V3
    GPIOB->ODR |= _3V3_EN;
    //wait 20ms 
    delay_ms(20);
    
    ssd2828_init();
    ssd2828_write_cfg();
    
    
    //enable 1V8
    GPIOB->ODR |= _1V8_EN;
    //pull rst high
    GPIOB->ODR |= _RST;
    //wait 250ms 
    delay_ms(300);
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
    delay_ms(10);

   
    
    lcd_init();
    //ssd2828_set_cfg();

   
    

    GPIOB->ODR |= _BACKLIGHT_EN;
  
    
   
   
    while(1){

        fill_screen(0xFF, 0, 0);
        fill_screen(0, 0xFF, 0);
        fill_screen(0, 0, 0xFF);

        i = ssd2828_get_id();
        if(i == 0x2828){
            GPIOC->ODR ^= GPIO_ODR_OD13;
            
        }
        
        delay_ms(100);
        
        
    }
    return 0;
}