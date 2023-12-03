#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "../system/stm32f4xx.h"

#include "../libs/delay.h"

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

#define TIMEOUT 15000

uint32_t HAL_RCC_GetSysClockFreq(void){
  uint32_t pllm = 0U, pllvco = 0U, pllp = 0U;
  uint32_t sysclockfreq = 0U;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (RCC->CFGR & RCC_CFGR_SWS)
  {
    case RCC_CFGR_SWS_HSI:  /* HSI used as system clock source */
    {
      sysclockfreq = ((uint32_t)16000000);
       break;
    }
    case RCC_CFGR_SWS_HSE:  /* HSE used as system clock  source */
    {
      sysclockfreq = ((uint32_t)25000000);
      break;
    }
    case RCC_CFGR_SWS_PLL:  /* PLL used as system clock  source */
    {
      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
      SYSCLK = PLL_VCO / PLLP */
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      if(((uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC)) != RCC_PLLCFGR_PLLSRC_HSI)
      {
        /* HSE used as PLL clock source */
        pllvco = (uint32_t) ((((uint64_t) ((uint32_t)25000000) * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (uint32_t) ((((uint64_t) ((uint32_t)16000000) * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
      }
      pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U) *2U);

      sysclockfreq = pllvco/pllp;
      break;
    }
    default:
    {
      sysclockfreq = ((uint32_t)16000000);
      break;
    }
  }
  return sysclockfreq;
}

uint32_t SystemClock_Config(){  
    uint32_t i, tmp;
    RCC->AHB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS_1;

    //set HSE oscillator ON
    RCC->CR |= RCC_CR_HSEON;
    //wait for HSE to get ready
    i = 0;
    while((RCC->CR & RCC_CR_HSERDY) == 0){
        i++;
        if(i == TIMEOUT){return 1;}
    }

    //disable main PLL
    RCC->CR &= ~RCC_CR_PLLON;
    //wait to get ready
    i = 0;
    while((RCC->CR & RCC_CR_PLLRDY) != 0){
        i++;
        if(i == TIMEOUT){return 1;}
    }
    //configure main PLL
    //set HSE as main PLL source
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC;
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk & ~RCC_PLLCFGR_PLLN_Msk & ~RCC_PLLCFGR_PLLP_Msk & ~RCC_PLLCFGR_PLLQ_Msk;
    //set main PLL M divider to 25
    RCC->PLLCFGR |= (25U << RCC_PLLCFGR_PLLM_Pos);
    //set main PLL N multiplier to 336
    RCC->PLLCFGR |= (336U << RCC_PLLCFGR_PLLN_Pos);
    //set main PLL P divider to 4
    RCC->PLLCFGR |= (1U << RCC_PLLCFGR_PLLP_Pos);
    //set main PLL Q divider to 4
    RCC->PLLCFGR |= (4U << RCC_PLLCFGR_PLLQ_Pos);
   
    //enable main PLL
    RCC->CR |= RCC_CR_PLLON;
    //wait for PLL to get ready
    i = 0;
    while((RCC->CR & RCC_CR_PLLRDY) == 0){
        i++;
        if(i == TIMEOUT){return 1;}
    }


    if((FLASH->ACR & FLASH_ACR_LATENCY) < 2U){
        tmp = FLASH->ACR;
        tmp &= ~FLASH_ACR_LATENCY_Msk;
        tmp |= (2U << FLASH_ACR_LATENCY_Pos);
        FLASH->ACR = tmp;
        if((FLASH->ACR & FLASH_ACR_LATENCY) != 2){
            return 1;
        }
    }
    

    //set APBx dividers to their maximum
    RCC->CFGR |= (0x1C00U << RCC_CFGR_PPRE1_Pos);
    RCC->CFGR |= (0x1C00U << RCC_CFGR_PPRE2_Pos);
    //set HCLK divider
    RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;

    //check if PLL is ready
    if((RCC->CR & RCC_CR_PLLRDY) == 0){
        return 1;
    }
    //set PLL as source for SYSCLK
    RCC->CFGR |= (2U << RCC_CFGR_SW_Pos);
    //wait for the PLL source to apply
    i = 0;
    while(((RCC->CFGR & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos) != 2U){
        i++;
        if(i == TIMEOUT){return 1;}
    }
    if((FLASH->ACR & FLASH_ACR_LATENCY) > 2U){
        tmp = FLASH->ACR;
        tmp &= ~FLASH_ACR_LATENCY_Msk;
        tmp |= (2U << FLASH_ACR_LATENCY_Pos);
        FLASH->ACR = tmp;
        if((FLASH->ACR & FLASH_ACR_LATENCY) != 2){
            return 1;
        }
    }

    //set PCLK1
    tmp = (uint32_t)RCC->CFGR;
    tmp &= ~RCC_CFGR_PPRE1_Msk;
    tmp |= (RCC_CFGR_PPRE1_DIV2 << RCC_CFGR_PPRE1_Pos);
    RCC->CFGR = tmp;
    
    //set PCLK2
    tmp = (uint32_t)RCC->CFGR;
    tmp &= ~RCC_CFGR_PPRE2_Msk;
    tmp |= (RCC_CFGR_PPRE1_DIV1 << RCC_CFGR_PPRE2_Pos);
    RCC->CFGR = tmp;

    SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE)>> RCC_CFGR_HPRE_Pos];

    return 0;
}

int main(){
    uint64_t i = 0;
    SystemClock_Config();
    gpio_init();
    delay_init();
    
    while(1){
         

    }
    return 0;
}