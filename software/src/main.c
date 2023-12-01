#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "../system/stm32f4xx.h"

void gpio_init(){
    //enable GPIOC and GPIOA GPIOB GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    //set A0 to input
    GPIOA->MODER &= ~GPIO_MODER_MODER0_0 & ~GPIO_MODER_MODER0_1;
    //set C13 to output
    GPIOC->MODER |= (0x1<<GPIO_MODER_MODER13_Pos);
    //set A0 with internal pull up
    GPIOA->PUPDR |= (0x1<<GPIO_PUPDR_PUPD0_Pos);
    //C13 to push pull
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT13;
    //set C13 speed to low
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR13;
    //set C13 to LOW
    GPIOC->ODR |= GPIO_ODR_OD13;

  
}

int main(){
    SystemInit();
    SystemCoreClockUpdate();
    gpio_init();
    uint32_t i = 0;
    while(1){
        i++;
        if(i == 840000){
            GPIOC->ODR ^= GPIO_ODR_OD13;
            i = 0;
        }

    }
    return 0;
}