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

    //set B0 B1 B2 as outputs
    GPIOB->MODER |= (0x1<<GPIO_MODER_MODER0_Pos) | (0x1<<GPIO_MODER_MODER1_Pos) | (0x1<<GPIO_MODER_MODER2_Pos);
    //set to push pull
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT0 & ~GPIO_OTYPER_OT1 & ~GPIO_OTYPER_OT2; 
    //set speed to low
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR0 & ~GPIO_OSPEEDER_OSPEEDR1 & ~GPIO_OSPEEDER_OSPEEDR2; 
    //set C13 to LOW
    GPIOB->ODR &= ~GPIO_ODR_OD0 & ~GPIO_ODR_OD1 & ~GPIO_ODR_OD2;

}


int main(){
    uint16_t i = 0;
    SystemClock_Config();
    gpio_init();
    delay_init();
    spi_init();
    
    //enable 1V2
    GPIOB->ODR |= GPIO_ODR_OD2;
    //wait 10ms
    delay_ms(10);
    //enable 3V3
    GPIOB->ODR |= GPIO_ODR_OD0;
    //wait 20ms 
    delay_ms(20);
    //enable 1V8
    GPIOB->ODR |= GPIO_ODR_OD1;

    delay_ms(100);
    ssd2828_init();

   

    while(1){
        GPIOC->ODR ^= GPIO_ODR_OD13;
        i = ssd2828_get_id();
        delay_ms(100);
    }
    return 0;
}