#include "ssd2828.h"

void ssd2828_gpio_init(){
    //enable GPIOA 
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    //set SHUT RST DC to output
    GPIOA->MODER |= (0x1<<GPIO_MODER_MODER1_Pos) | (0x1<<GPIO_MODER_MODER2_Pos) | (0x1<<GPIO_MODER_MODER3_Pos);
    //set SHUT RST DC to push pull
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT1 & ~GPIO_OTYPER_OT2 & ~GPIO_OTYPER_OT3;
    //set SHUT RST DC speed to low
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR1 & ~GPIO_OSPEEDER_OSPEEDR2 & ~GPIO_OSPEEDER_OSPEEDR3;
    //set SHUT DC to LOW
    GPIOA->ODR &= ~SHUT & ~DC;
    GPIOA->ODR |= RST;
}

void ssd2828_init(){
    ssd2828_gpio_init();
    //hold rst low for 15ms
    GPIOA->ODR &= ~RST;
    delay_ms(15);
    GPIOA->ODR |= RST;
}

void ssd2828_SPI_write_reg(uint8_t reg, uint16_t val){
    uint8_t input[3] = {reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8U)};
    
    GPIOA->ODR &= ~DC;
    spi_exchange_8(input[0], NULL);
    GPIOA->ODR |= DC;
    spi_exchange_8(input[1], NULL);
    spi_exchange_8(input[2], NULL);
}

void ssd2828_SPI_read_reg(uint8_t reg, uint16_t *val){
    uint8_t input[4] = {reg, 0xFA, 0x00, 0x00};
    uint8_t output[2];

    GPIOA->ODR &= ~DC;
    spi_exchange_8(input[0], NULL);
    spi_exchange_8(input[1], NULL);
    GPIOA->ODR |= DC;
    spi_exchange_8(input[2], &output[0]);
    spi_exchange_8(input[3], &output[1]);

    *val = (uint16_t)output[0] + (uint16_t)(output[1] << 8U);
}

uint16_t ssd2828_get_id(){
    uint16_t val;
    ssd2828_SPI_read_reg(DIR_REG, &val);
    return val;
}

void ssd2828_SPI_write_cmd(uint8_t data){
    GPIOA->ODR &= ~DC;
    spi_exchange_8(&data, NULL);
    GPIOA->ODR |= DC;
}

void ssd2828_SPI_write_data(uint8_t data){
    spi_exchange_8(&data, NULL);
}

void ssd2828_SPI_read_data(uint8_t *data){
    spi_exchange_8(0x00, data);
}

void GP_COMMAD_PA(uint16_t num){
  ssd2828_SPI_write_cmd(0xBC);
  ssd2828_SPI_write_data(num & 0xFF);  
  ssd2828_SPI_write_data((num>>8) & 0xFF); 
  ssd2828_SPI_write_cmd(0xBF);
}

void ssd2828_MIPI_write_DCS_short(uint8_t reg, uint16_t *data, uint32_t len){
    uint16_t tmp = 0;
    //set REN to 0, DCS to 1, LPE to 0
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp |= (1U<<CFGR_DCS_POS);
    tmp &= ~(1U<<CFGR_REN_POS) & ~(1U<<CFGR_LPE_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);
    //set virtual channel to 0
    ssd2828_SPI_write_reg(VCR_REG, 0x0000);
    //set payload length on PSCR1
    ssd2828_SPI_write_reg(PSCR1_REG, len);
    //write on PDR
    ssd2828_SPI_write_data(PDR_REG);
    //write reg
    ssd2828_SPI_write_data(reg);
    //write rest of data
    for(uint32_t i = 0; i<len; i++){
        ssd2828_SPI_write_data(data[i]);
    }
}

