#include "ssd2828.h"


void ssd2828_SHUT_1(){
    SSD2828_C_PORT->ODR |= SSD2828_SHUT;
}

void ssd2828_SHUT_0(){
    SSD2828_C_PORT->ODR &= ~SSD2828_SHUT;
}

void ssd2828_gpio_init(){
    //enable GPIOA GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    //set SHUT RST DC to output
    SSD2828_C_PORT->MODER |=    (0x1 << (SSD2828_SHUT*2)) | (0x1 << (SSD2828_DC*2)) | (0x1 << (SSD2828_RST*2));
    //set SHUT RST DC to push pull
    SSD2828_C_PORT->OTYPER &=   ~(0x1 << (SSD2828_SHUT*2)) & ~(0x1 << (SSD2828_DC*2)) & ~(0x1 << (SSD2828_RST*2));
    //set SHUT RST DC speed to low
    SSD2828_C_PORT->OSPEEDR &=  ~(0x3U << (SSD2828_SHUT*2)) & ~(0x3U << (SSD2828_DC*2)) & ~(0x3U << (SSD2828_RST*2));
    //set SHUT DC to LOW
    SSD2828_C_PORT->ODR &=      ~(1 << SSD2828_SHUT) & ~(1 << SSD2828_DC);
    SSD2828_C_PORT->ODR |=      (1 << SSD2828_RST);

    //set 1V2 3V3 as outputs
    SSD2828_V_PORT->MODER |=    (1U << (SSD2828_1V2*2)) | (1U << (SSD2828_3V3*2));
    //set to push pull
    SSD2828_V_PORT->OTYPER &=   ~(1U << (SSD2828_1V2*2)) & ~(1U << (SSD2828_3V3*2));
    //set speed to low
    SSD2828_V_PORT->OSPEEDR &=  ~(3U << (SSD2828_1V2*2)) & ~(3U << (SSD2828_3V3*2));
    //set LOW
    SSD2828_V_PORT->ODR &=      ~(1U << SSD2828_1V2) & ~(1U << SSD2828_3V3);

}

void ssd2828_init(){
    ssd2828_gpio_init();
    ssd2828_SHUT_1();

    //enable 1V2
    SSD2828_V_PORT->ODR |= (1 << SSD2828_1V2);
    //wait 10ms
    delay_ms(10);
    //enable 3V3
    SSD2828_V_PORT->ODR |= (1 << SSD2828_3V3);
    //wait 20ms 
    delay_ms(20);

    SSD2828_C_PORT->ODR |= (1 << SSD2828_RST);
    delay_ms(100);
    //hold rst low for 20ms
    SSD2828_C_PORT->ODR &= ~(1 << SSD2828_RST);
    delay_ms(205);
    SSD2828_C_PORT->ODR |= (1 << SSD2828_RST);
    delay_ms(200);
}

void ssd2828_SPI_write_reg(uint8_t reg, uint16_t val){
    uint8_t input[3] = {reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8U)};
    
    SSD2828_C_PORT->ODR &= ~(1 << SSD2828_DC);
    spi_exchange_8(input[0], NULL);
    SSD2828_C_PORT->ODR |= (1 << SSD2828_DC);
    spi_exchange_8(input[1], NULL);
    spi_exchange_8(input[2], NULL);
}

void ssd2828_SPI_read_reg(uint8_t reg, uint16_t *val){
    uint8_t input[4] = {reg, 0xFA, 0x00, 0x00};
    uint8_t output[2];

    SSD2828_C_PORT->ODR &= ~(1 << SSD2828_DC);
    spi_exchange_8(input[0], NULL);
    spi_exchange_8(input[1], NULL);
    SSD2828_C_PORT->ODR |= (1 << SSD2828_DC);
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
    SSD2828_C_PORT->ODR &= ~(1 << SSD2828_DC);
    spi_exchange_8(data, NULL);
    SSD2828_C_PORT->ODR |= (1 << SSD2828_DC);
}

void ssd2828_SPI_write_data(uint8_t data){
    spi_exchange_8(data, NULL);
}

void ssd2828_SPI_read_data(uint8_t *data){
    spi_exchange_8(0x00, data);
}

void ssd2828_MIPI_write_DCS_short_np(uint8_t cmd){
    uint16_t tmp = 0;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp |= (1 << CFGR_DCS_POS);
    tmp &= ~(1 << CFGR_LPE_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);
   
    //set payload length
    ssd2828_SPI_write_reg(PSCR1_REG, 1);
    ssd2828_SPI_write_reg(PSCR2_REG, 0x0000);
    ssd2828_SPI_write_reg(PSCR3_REG, 1);
    //write on PDR
    ssd2828_SPI_write_cmd(PDR_REG);
    //write reg
    ssd2828_SPI_write_data(cmd);
}

void ssd2828_MIPI_write_DCS_short_p(uint8_t cmd, uint8_t param){
    uint16_t tmp = 0;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp |= (1 << CFGR_DCS_POS);
    tmp &= ~(1 << CFGR_LPE_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);
    
    //set payload length
    ssd2828_SPI_write_reg(PSCR1_REG, 2);
    ssd2828_SPI_write_reg(PSCR2_REG, 0x0000);
    ssd2828_SPI_write_reg(PSCR3_REG, 2);
    //write on PDR
    ssd2828_SPI_write_cmd(PDR_REG);
    //write reg
    ssd2828_SPI_write_data(cmd);
    ssd2828_SPI_write_data(param);
}

void ssd2828_MIPI_write_DCS_long_p(uint8_t cmd, uint8_t *params, uint32_t len){
    uint16_t tmp = 0;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp |= (1 << CFGR_DCS_POS) | (1 << CFGR_LPE_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);
    
    //set payload length
    ssd2828_SPI_write_reg(PSCR1_REG, len+1);
    ssd2828_SPI_write_reg(PSCR2_REG, (len+1) >> 16);
    ssd2828_SPI_write_reg(PSCR3_REG, len+1);
    //write on PDR
    ssd2828_SPI_write_cmd(PDR_REG);
    //write reg
    ssd2828_SPI_write_data(cmd);
    for(uint32_t i = 0; i < len; i++){
        ssd2828_SPI_write_data(params[i]);
    }
}

void ssd2828_MIPI_write_generic_short_np(uint8_t cmd){
    uint16_t tmp = 0;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp &= ~(1 << CFGR_LPE_POS) & ~(1 << CFGR_DCS_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);
   
    //set payload length
    ssd2828_SPI_write_reg(PSCR1_REG, 1);
    ssd2828_SPI_write_reg(PSCR2_REG, 0x0000);
    ssd2828_SPI_write_reg(PSCR3_REG, 1);
    //write on PDR
    ssd2828_SPI_write_cmd(PDR_REG);
    //write reg
    ssd2828_SPI_write_data(cmd);
}

void ssd2828_MIPI_write_generic_short_p(uint8_t cmd, uint8_t param){
    uint16_t tmp = 0;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp &= ~(1 << CFGR_LPE_POS) & ~(1 << CFGR_DCS_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);
    
    //set payload length
    ssd2828_SPI_write_reg(PSCR1_REG, 2);
    ssd2828_SPI_write_reg(PSCR2_REG, 0x0000);
    ssd2828_SPI_write_reg(PSCR3_REG, 2);
    //write on PDR
    ssd2828_SPI_write_cmd(PDR_REG);
    //write reg
    ssd2828_SPI_write_data(cmd);
    ssd2828_SPI_write_data(param);
}

void ssd2828_MIPI_write_generic_long_p(uint8_t cmd, uint8_t *params, uint32_t len){
    uint16_t tmp = 0;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp |= (1 << CFGR_LPE_POS);
    tmp &= ~(1 << CFGR_DCS_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);
    
    //set payload length
    ssd2828_SPI_write_reg(PSCR1_REG, len+1);
    ssd2828_SPI_write_reg(PSCR2_REG, (len+1) >> 16);
    ssd2828_SPI_write_reg(PSCR3_REG, len+1);
    //write on PDR
    ssd2828_SPI_write_cmd(PDR_REG);
    //write reg
    ssd2828_SPI_write_data(cmd);
    for(uint32_t i = 0; i < len; i++){
        ssd2828_SPI_write_data(params[i]);
    }
}

void ssd2828_set_HS(){
    uint16_t tmp = 0;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp |= (1 << CFGR_HS_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);
}

void ssd2828_set_LP(){
    uint16_t tmp = 0;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp &= ~(1 << CFGR_HS_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);
}



void ssd2828_BIST_ON(){
    uint16_t tmp;
    ssd2828_SPI_read_reg(TMR_REG, &tmp);
    tmp |= (1 << TMR_VBIST_EN_POS) | (1 << TMR_VBIST_SRT_POS);
    ssd2828_SPI_write_reg(TMR_REG, tmp);   
}

void ssd2828_BIST_OFF(){
    uint16_t tmp;
    ssd2828_SPI_read_reg(TMR_REG, &tmp);
    tmp &= ~(1 << TMR_VBIST_EN_POS) & ~(1 << TMR_VBIST_SRT_POS);
    ssd2828_SPI_write_reg(TMR_REG, tmp); 
}

void ssd2828_VID_ON(){
    uint16_t tmp;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp |= (1U << CFGR_VEN_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp); 
}

void ssd2828_VID_OFF(){
    uint16_t tmp;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    tmp &= ~(1U << CFGR_VEN_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp); 
}