#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "../system/stm32f4xx.h"

#include "../libs/delay.h"
#include "../libs/spi.h"

#include "ssd2828.h"
#include "LH154Q01.h"
#include "VS035ZSM.h"
#include "clk.h"



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
    uint16_t i = 0;
    SystemClock_Config();
    gpio_init();
    delay_init();
    spi_init();
    delay_ms(1500);
    
    
    
    ssd2828_init();
    //LH154Q01_init();
    //LH154Q01_SSD2828_write_cfg();
    //LH154Q01_start();
    //LH154Q01_backlight(1);
    VS035ZSM_init();
    VS035ZSM_init_BACKLIGHT();
    VS035ZSM_SSD2828_write_cfg();
    VS035ZSM_start();
    
    //VS035ZSM_backlight(1);
 
    GPIOC->ODR ^= GPIO_ODR_OD13;
   
    while(1){

        //fill_screen(0xFF, 0, 0);
        //fill_screen(0, 0xFF, 0);
        //fill_screen(0, 0, 0xFF);

        i = ssd2828_get_id();
        if(i == 0x2828){
            GPIOC->ODR ^= GPIO_ODR_OD13;
        }
        
        delay_ms(100);
    }
    return 0;
}