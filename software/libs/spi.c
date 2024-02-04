#include "spi.h"



void spi_gpio_init(){
    //enable GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    //set A4 A5 A6 A7 alternate function
    GPIOA->MODER |= (0x2<<GPIO_MODER_MODER4_Pos) | (0x2<<GPIO_MODER_MODER5_Pos) | (0x2<<GPIO_MODER_MODER6_Pos) | (0x2<<GPIO_MODER_MODER7_Pos);
    //set A4 A5 A6 A7 to push pull
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT4 &  ~GPIO_OTYPER_OT5 & ~GPIO_OTYPER_OT6 & ~GPIO_OTYPER_OT7;
    //set A4 A5 A6 A7 speed high
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR6_1;
    //set A4 to AF0 
    GPIOA->AFR[0] |= (5<<GPIO_AFRL_AFSEL4_Pos);
    //set A5 to AF0 
    GPIOA->AFR[0] |= (5<<GPIO_AFRL_AFSEL5_Pos);
    //set A6 to AF0 
    GPIOA->AFR[0] |= (5<<GPIO_AFRL_AFSEL6_Pos);
    //set A7 to AF0 
    GPIOA->AFR[0] |= (5<<GPIO_AFRL_AFSEL7_Pos);
}

void spi_init(){
    spi_gpio_init();
    //enable SPI1 in RCC
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    //set baudrate to fclk/64
    SPI1->CR1 |= (0b101 << SPI_CR1_BR_Pos) | SPI_CR1_MSTR;
    //set CPOL CPHA to 0
    SPI1->CR1 &= ~SPI_CR1_CPHA & ~SPI_CR1_CPOL;
    //set 8-bit data frame
    SPI1->CR1 &= ~SPI_CR1_DFF;
    //set MSB  first
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
    //enable the SS pin
    SPI1->CR2 |= SPI_CR2_SSOE;

}


void spi_exchange_8(uint8_t data_in, uint8_t *data_out){
    uint8_t tmp;
    SPI1->CR1 |= SPI_CR1_SPE ;
    //wait for SPI1 to be available
    while((SPI1->SR & SPI_SR_BSY_Msk) != 0){
        __NOP();
    }
    //write data on TX register
    SPI1->DR = data_in;
    //wait for TX buffer to get empty and RX to fill up
    while( ((SPI1->SR & SPI_SR_RXNE_Msk) == 0)){
        __NOP();
    }
    tmp = SPI1->DR;
    if(data_out != NULL){
        *data_out = tmp;
    }
    SPI1->CR1 &= ~SPI_CR1_SPE ;
}
