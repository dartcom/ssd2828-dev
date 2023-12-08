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

void ssd2828_SPI_write(uint8_t *buffer, uint32_t len){
    for(uint32_t i = 0; i < len; i++){
        spi_exchange_8(buffer[i], NULL);
    }
}

void ssd2828_SPI_read(uint8_t *buffer, uint32_t len){
    for(uint32_t i = 0; i < len; i++){
        spi_exchange_8(0x00, &buffer[i]);
    }
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

void ssd2828_MIPI_write_long_generic(uint8_t reg, uint8_t* data, uint32_t len){
    ssd2828_SPI_write_reg(CFGR_REG, 0x0302);
    ssd2828_SPI_write_reg(VCR_REG, 0x0000);
    ssd2828_SPI_write_reg(PSCR1_REG, (uint16_t)len);
    uint8_t tmp0[1] = {PDR_REG};
    ssd2828_SPI_write(tmp0, 1);
    uint8_t tmp1[1] = {1<<8 | reg};
    ssd2828_SPI_write(tmp1, 1);
    uint8_t tmp2[1];
    for(uint32_t i = 0; i < len-1; i++){
        tmp2[0] = 1 << 8 | data[i];
        ssd2828_SPI_write(tmp2, 1);
    }
}

void ssd2828_MIPI_write_long_DCS(uint8_t reg, uint16_t* data, uint32_t len){
    ssd2828_SPI_write_reg(0xB7, 0x0050);
    ssd2828_SPI_write_reg(0xB8, 0x0000);
    ssd2828_SPI_write_reg(0xBC, len);
    uint8_t tmp0[1]={0xBF};
    ssd2828_SPI_write(tmp0, 1);
    uint8_t tmp1[1]={1 << 8 |reg};
    ssd2828_SPI_write(tmp1, 1);
    uint8_t tmp2[1];
    for(uint32_t i = 0; i < len-1; i++){
        tmp2[0] = 1 << 8 | data[i];
        ssd2828_SPI_write(tmp2, 1);
    }
}

void ssd2828_MIPI_write_short_generic(uint8_t reg,uint16_t data,int len){
    uint8_t tmp[2]={data & 0xFF, data>>8};
    ssd2828_MIPI_write_long_generic(reg, tmp, len);
}
void ssd2828_MIPI_write_short_DCS(uint8_t reg,uint16_t data,int len){
    uint8_t tmp[2]={data & 0xFF, data>>8};
    ssd2828_MIPI_write_long_DCS(reg, tmp, len);
}

/*
uint16_t SSD_MIPI_ReadGeneric(uint8_t reg,uint16_t *len, uint16_t *status){
    ssd2828_SPI_write_reg(0xb7,0x0382);
    ssd2828_SPI_write_reg(0xc1,0x0002);
    ssd2828_SPI_write_reg(0xc0,0x0001);

    ssd2828_SPI_write_reg(0xbc,0x0001);
    ssd2828_SPI_write_reg(0xbf,reg,;
    //if(len!=0){
        *len = ssd2828_SPI_read_reg(0xc2);
    //}
    //if(status!=0){  //0 - no error; 1 - read not ready
        if((ssd2828_SPI_read_reg(0xc6)&1)==0){
            *status=1;
        }else{
            *status=0;
        }
        if(ssd2828_SPI_read_reg(0xc3)!=0){
            *status=2;
        }else{
            *status=0;
        }
    //}
    return ssd2828_SPI_read_reg(0xff);
}
uint16_t SSD_MIPI_ReadDCS(uint8_t reg,uint16_t *len, uint16_t *status){
    ssd2828_SPI_write_reg(0xb7,0x03c2);
    ssd2828_SPI_write_reg(0xc1,0x00ff);
    ssd2828_SPI_write_reg(0xc0,0x0002);

    ssd2828_SPI_write_reg(0xbc,0x0001);
    ssd2828_SPI_write_reg(0xbf,reg);
    //if(len!=0){
        *len = ssd2828_SPI_read_reg(0xc2);
    //}
    //if(status!=0){
        if((ssd2828_SPI_read_reg(0xc6)&1)==0){
            *status=1;
        }else{
            *status=0;
        }
        if(ssd2828_SPI_read_reg(0xc3)!=0){
            *status=2;
        }else{
            *status=0;
        }
    //}
    return ssd2828_SPI_read_reg(0xff);
}
*/