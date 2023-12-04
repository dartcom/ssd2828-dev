#include "delay.h"


volatile uint64_t ms_ticks;
volatile uint64_t us_ticks;

void delay_ms(uint32_t delay_ms){
    ms_ticks = 0;
    while (delay_ms > ms_ticks){
        __NOP();
    }
}

void delay_us(uint32_t delay_us){
    us_ticks = 0;
    while (delay_us > us_ticks){
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
    if((us_ticks == 1000)){
        ms_ticks++;
        
        us_ticks = 0;
    }
    
}