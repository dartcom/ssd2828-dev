#include "VS035ZSM.h"


void VS035ZSM_SSD2828_write_cfg(){
    //disable the PLL
    ssd2828_SPI_write_reg(PCR_REG, 0x0000);
    //set PLL freq to 336MHZ 
    ssd2828_SPI_write_reg(PLCR_REG, 0x0000 | (1U << PLCR_FR_POS) | (1U << PLCR_MS_POS) | (14U << PLCR_NS_POS));
    //set divider for low speed to 8MHZ
    ssd2828_SPI_write_reg(CCR_REG, 0x0000 | (4U << CCR_LPD_POS));
    //enable the PLL
    ssd2828_SPI_write_reg(PCR_REG, 0x0000 | (1U << PCR_PEN_POS));
    //wait 2 ms to stabilize
    delay_ms(2);
    



    //set VSYNC HSYNC period
    ssd2828_SPI_write_reg(VICR1_REG, 0x0000 );
    //set VBP HBP

    //set VFP HFP

    //set horizontal active

    //set vertical active

    //set color depth 24bit


    //set lanes number to 4
    ssd2828_SPI_write_reg(LCFR_REG, 0x0003);
    
    //set MIPI packet format
    //enable EOT DCS 
    //disable Read Video
    ssd2828_SPI_write_reg(CFGR_REG, 0x0000 | (1U << CFGR_EOT_POS) | (1U << CFGR_DCS_POS));
    //set virtual channel 0x0000
    ssd2828_SPI_write_reg(VCR_REG, 0x0000);
}

void VS035ZSM_init(){
    VS035ZSM_init_GPIO();

    //set MIPI lanes to LP-00 state unitl reset goes high


    //enable VDDI 1V8
    VS035ZSM_V_PORT->ODR |= (1U << VS035ZSM_1V8);
    //wait 12ms
    delay_ms(12);
    //enable AVDD & AVEE
    VS035ZSM_V_PORT->ODR |= (1U << VS035ZSM_BIAS);
    //wait 35ms 
    delay_ms(15);
    //pull RST high
    VS035ZSM_C_PORT->ODR |= (1U << VS035ZSM_RST);
    //set MIPI lanes to LP-11

    //wait 15ms
    delay_ms(15);
   

}

void VS035ZSM_start(){

}

void VS035ZSM_init_GPIO(){
    //enable GPIOA GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    //set 1V8 BIAS as outputs
    VS035ZSM_V_PORT->MODER |=   (1U << (VS035ZSM_1V8*2)) | (1U << (VS035ZSM_BIAS*2));
    //set to push pull
    VS035ZSM_V_PORT->OTYPER &=  ~(1U << VS035ZSM_1V8) & ~(1U << VS035ZSM_BIAS);
    //set speed to low
    VS035ZSM_V_PORT->OSPEEDR &= ~(3U << (VS035ZSM_1V8*2)) & ~(3U << (VS035ZSM_BIAS*2));
    //set LOW
    VS035ZSM_V_PORT->ODR &=     ~(1U << VS035ZSM_1V8) & ~(1U << VS035ZSM_BIAS);


    //set RST as output
    VS035ZSM_C_PORT->MODER |=   (1U << (VS035ZSM_RST*2));
    //set to push pull
    VS035ZSM_C_PORT->OTYPER &=  ~(1U << VS035ZSM_RST);
    //set speed to low
    VS035ZSM_C_PORT->OSPEEDR &= ~(3U << (VS035ZSM_RST*2));
    //set LOW
    VS035ZSM_C_PORT->ODR &=     ~(1U << VS035ZSM_RST);


    //set BACKLIGHT as PWM output
    VS035ZSM_C_PORT->MODER |=   (0x2U << (VS035ZSM_BACKLIGHT*2));
    //set to push pull
    VS035ZSM_C_PORT->OTYPER &=  ~(1U << VS035ZSM_BACKLIGHT);
    //set for high speed
    VS035ZSM_C_PORT->OSPEEDR |= (3U << (VS035ZSM_BACKLIGHT*2));
    //set alternate funtion for PA8, to AF1
    VS035ZSM_C_PORT->AFR[1] |= (0b0001 << ((VS035ZSM_BACKLIGHT - 8U) * 4));
}

void VS035ZSM_init_BACKLIGHT(){
    //enable TIM1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    
    //working with channel 1
    //counter set as counting up and reloading to 0 when overflowing
    //set to PWM mode 1 
    TIM1->CCMR1 |= (0b110 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    //APB2 clk is @84MHZ
    //set prescaler for 10KHZ as counter clock
    TIM1->PSC = (8400U - 1U);
    //set frequency for 60hz
    TIM1->ARR = 166U;
    //set duty cycle for 10%
    TIM1->CCR1 = 16U;
    //enable output for channel 1
    TIM1->CCER = (1U << TIM_CCER_CC1E_Pos);
    //enable channel 1 output on the BDTR register 
    TIM1->BDTR = TIM_BDTR_AOE | TIM_BDTR_MOE;
    //enable autoreloading for the CCR1 register
    TIM1->CR1 |= TIM_CR1_ARPE;
    //set direction for up counter
    TIM1->CR1 &= ~TIM_CR1_DIR & ~TIM_CR1_CMS_Msk;

    TIM1->EGR |= TIM_EGR_UG;

}

void VS035ZSM_backlight(uint8_t i){
    if(i == 1){
        //enable backlight PWM
        TIM1->CR1 |= TIM_CR1_CEN;
    }else{
        //disable backlight PWM
        TIM1->CR1 &= ~TIM_CR1_CEN;
    } 
}