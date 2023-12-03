#include "delay.h"


volatile uint64_t ms_ticks;
volatile uint64_t us_ticks;

void delay_ms(uint32_t delay){
    
}

void delay_us(uint32_t delay){
    uint64_t start;
    start = us_ticks;
    while ((us_ticks + start) < (us_ticks + delay)){
        __NOP();
    }
}

void delay_init(){
    SysTick->CTRL |= (1 << SysTick_CTRL_TICKINT_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);
    //set for 1us 
    SysTick->LOAD = SystemCoreClock / 1000000;
    SysTick->VAL = 0x00000000;
    

    SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos);
    NVIC_SetPriority(SysTick_IRQn, 0x25);
    NVIC_EnableIRQ(SysTick_IRQn);
}

void SysTick_Handler(){
    us_ticks++;
    GPIOC->ODR ^= GPIO_ODR_OD13;
}