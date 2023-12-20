#include "ssd2828.h"

enum STATE state;

void ssd2828_state_LP(){
    state = LP;
}

void ssd2828_state_HS(){
    state = HS;
}

void ssd2828_state_VD(){
    state = VD;
}

void ssd2828_SHUT_1(){
    GPIOA->ODR |= SHUT;
}

void ssd2828_SHUT_0(){
    GPIOA->ODR &= ~SHUT;
}

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
    ssd2828_SHUT_1();
    GPIOA->ODR |= RST;
    delay_ms(100);
    //hold rst low for 20ms
    GPIOA->ODR &= ~RST;
    delay_ms(205);
    GPIOA->ODR |= RST;
    delay_ms(200);
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
    spi_exchange_8(data, NULL);
    GPIOA->ODR |= DC;
}

void ssd2828_SPI_write_data(uint8_t data){
    spi_exchange_8(data, NULL);
}

void ssd2828_SPI_read_data(uint8_t *data){
    spi_exchange_8(0x00, data);
}





void ssd2828_MIPI_write_DCS_short_np(uint8_t DCS){
    uint16_t tmp = 0;
    
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    switch(state){
        case LP:
            ssd2828_SPI_write_cmd(CFGR_REG);
            ssd2828_SPI_write_data(0x50);
            ssd2828_SPI_write_data(0x02);
        case HS:
            ssd2828_SPI_write_cmd(CFGR_REG);
            ssd2828_SPI_write_data(0x50&0XEF|0X03);
            ssd2828_SPI_write_data(0x02);
            break;
        case VD:
            ssd2828_SPI_write_cmd(CFGR_REG);
            ssd2828_SPI_write_data(0x50&0XEF|0X0B);
            ssd2828_SPI_write_data(0x02|0x01);
            break;
        default:

            break;
    };
    
    delay_us(10);
    //set virtual channel to 0
    ssd2828_SPI_write_reg(VCR_REG, 0x0000);
    //set payload length
    ssd2828_SPI_write_reg(PSCR1_REG, 1);
    ssd2828_SPI_write_reg(PSCR2_REG, 0x0000);
    ssd2828_SPI_write_reg(PSCR3_REG, 1);
    //write on PDR
    ssd2828_SPI_write_cmd(PDR_REG);
    //write reg
    ssd2828_SPI_write_data(DCS);
}

void ssd2828_MIPI_write_DCS_short_p(uint8_t DCS, uint16_t param){
    uint16_t tmp = 0;
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    switch(state){
        case LP:
            ssd2828_SPI_write_cmd(CFGR_REG);
            ssd2828_SPI_write_data(0x50);
            ssd2828_SPI_write_data(0x02);
        case HS:
            ssd2828_SPI_write_cmd(CFGR_REG);
            ssd2828_SPI_write_data(0x50&0XEF|0X03);
            ssd2828_SPI_write_data(0x02);
            break;
        case VD:
            ssd2828_SPI_write_cmd(CFGR_REG);
            ssd2828_SPI_write_data(0x50&0XEF|0X0B);
            ssd2828_SPI_write_data(0x02|0x01);
            break;
        default:

            break;
    };
    //set virtual channel to 0
    ssd2828_SPI_write_reg(VCR_REG, 0x0000);
    //set payload length
    ssd2828_SPI_write_reg(PSCR1_REG, 2);
    ssd2828_SPI_write_reg(PSCR2_REG, 0x0000);
    ssd2828_SPI_write_reg(PSCR3_REG, 2);
    //write on PDR
    ssd2828_SPI_write_cmd(PDR_REG);
    //write reg
    ssd2828_SPI_write_data(DCS);
    ssd2828_SPI_write_data(param);
}

void ssd2828_MIPI_write_DCS_long_p(uint8_t DCS, uint16_t *params, uint32_t len){
    uint16_t tmp = 0;
    //set REN 0, DCS 1, LPE 1, EOT 1, HCLK 1, HS 0, CKE 0
    ssd2828_SPI_read_reg(CFGR_REG, &tmp);
    switch(state){
        case LP:
            ssd2828_SPI_write_cmd(CFGR_REG);
            ssd2828_SPI_write_data(0x10);
            ssd2828_SPI_write_data(0x06);
        case HS:
            ssd2828_SPI_write_cmd(CFGR_REG);
            ssd2828_SPI_write_data(0x10&0XEF|0X03);
            ssd2828_SPI_write_data(0x06);
            break;
        case VD:
            ssd2828_SPI_write_cmd(CFGR_REG);
            ssd2828_SPI_write_data(0x10&0XEF|0X0B);
            ssd2828_SPI_write_data(0x06|0x01);
            break;
        default:

            break;
    };
    //set virtual channel to 0
    ssd2828_SPI_write_reg(VCR_REG, 0x0000);
    //set payload length
    ssd2828_SPI_write_reg(PSCR1_REG, len+1);
    ssd2828_SPI_write_reg(PSCR2_REG, (len+1) >> 16);
    ssd2828_SPI_write_reg(PSCR3_REG, len+1);
    //write on PDR
    ssd2828_SPI_write_cmd(PDR_REG);
    //write reg
    ssd2828_SPI_write_data(DCS);
    for(uint32_t i = 0; i < len; i++){
        ssd2828_SPI_write_data(params[i]);
    }
}

void ssd2828_MIPI_write_generic_short_np(uint8_t generic){

}

void ssd2828_MIPI_write_generic_short_p(uint8_t generic, uint16_t param){

}

void ssd2828_MIPI_write_generic_long_p(uint8_t generic, uint16_t *params, uint32_t len){

}


void ssd2828_MIPI_write_DCS(uint8_t DCS, uint16_t *params, uint32_t len){
    switch (len){
        case 0:
            ssd2828_MIPI_write_DCS_short_np(DCS);
            break;
        case 1:
            ssd2828_MIPI_write_DCS_short_p(DCS, *params);
            break;
        default:
            ssd2828_MIPI_write_DCS_long_p(DCS, params, len);
            break;
    }
}

void ssd2828_MIPI_write_generic(uint8_t generic, uint16_t *params, uint32_t len){
    switch (len){
        case 0:
            ssd2828_MIPI_write_generic_short_np(generic);
            break;
        case 1:
            ssd2828_MIPI_write_generic_short_p(generic, *params);
            break;
        default:
            ssd2828_MIPI_write_generic_long_p(generic, params, len);
            break;
    }
}


void ssd2828_write_cfg(){
    //VFP 11
    ssd2828_SPI_write_reg(VICR6_REG, 0x0000| (3U << VICR6_VPF_POS) |(2U<<VICR6_VM_POS) |(1U<<VICR6_CBM_POS) |(1U<<VICR6_NVB_POS));//0xB6
    //lanes
    ssd2828_SPI_write_reg(LCFR_REG, 0x0000);//0xDE 0-1:LS,15-2:NC
    //PNB 1
    ssd2828_SPI_write_reg(TR_REG, 0x0000|(1 << TR_PNB_POS));//0xD6 0:CO,1:END,7-2:PNB,8:FLM,13-9:EIC,15-14:TM/FL0
    //
    ssd2828_SPI_write_reg(PCR_REG, 0x0000);//0xB9 0:PEN,12-1:NC,13:SYS_DIS,15-14:SYSD
    //FR 10 | MS 1 | NS 1
    ssd2828_SPI_write_reg(PLCR_REG, 0x0000|(2U << PLCR_FR_POS) | (1U << PLCR_MS_POS) | (10U << PLCR_NS_POS));//0xBA MIPI��������//7-0:NS,12-8:MS,13:NC,15-14:FR
    //
    ssd2828_SPI_write_reg(CCR_REG, 0x0000|(36U << CCR_LPD_POS));//0xBB 5-0:LPD,15-6:NC
    //
    ssd2828_SPI_write_reg(PCR_REG, 0x0000|(1U << PCR_PEN_POS));//0xB9 0:PEN,12-1:NC,13:SYS_DIS,15-14:SYSD
    //
    ssd2828_SPI_write_reg(VCR_REG, 0x0000);//0xB8 1-0:VC1,3-2:VC2,5-4:VCE,7-6:VCM,15-8:NC
    //(SSD_CFGR)0:HS,1:CKE,2:SLP,3:VEN,4:HCLK,5:CSS,6:DCS,7:REN,8:ECD,9EOT,10:LPE,11:TXD,15-12:NC
    //EOT 1 | ECD 1
    ssd2828_SPI_write_reg(CFGR_REG, 0x0300);//0xB7

}

void ssd2828_set_cfg(){

    // start high speed mipi signal
    //EOT 1 | ECD 1 | VEN 1 | HS 1
    ssd2828_SPI_write_reg(CFGR_REG, 0x0309);
    // SSD2828 BIST mode
    ssd2828_SPI_write_reg(TMR_REG, 0x0000|(1 << TMR_VBIST_EN_POS) | (1 << TMR_VBIST_SRT_POS));
    
}

void ssd2828_write_lcd_params(){
    ssd2828_SPI_write_reg(VICR1_REG, (LCD_VSPW<<8)|LCD_HSPW);
    ssd2828_SPI_write_reg(VICR2_REG, (LCD_VBPD<<8)|LCD_HBPD);
    ssd2828_SPI_write_reg(VICR3_REG, (LCD_VFPD<<8)|LCD_HFPD);
    ssd2828_SPI_write_reg(VICR4_REG, LCD_XSIZE_TFT);
    ssd2828_SPI_write_reg(VICR5_REG, LCD_YSIZE_TFT);
}


void SSD_WritePacket(const uint8_t *PData)
{
    uint8_t i;
    uint16_t number;
    uint8_t temp;
    
    number = (uint16_t)(*PData); //�ֽ���
    PData++;
    ssd2828_SPI_write_reg(PSCR1_REG, number&0x0FFF);//0xBC //�����ϲ�����4096��ʵ��������Ϊ�ֽڲ�����255
    ssd2828_SPI_write_cmd(PDR_REG);//0xBF
    for(i=0; i<number; i++)
    {
        temp = *PData;
        PData++;
        ssd2828_SPI_write_data(temp);
    }
}

void lcd_init(){
    const uint8_t SLPOUT[]={1,0x11};
    const uint8_t DISPOFF[]={1,0x28};
    const uint8_t INV[]={1,0x21};
    const uint8_t DISPON[]={1,0x29};
    const uint8_t RGB[]={2,0x3A ,0x77};//24bit
    const uint8_t FLIP[]={2,0x36 ,0x08};//COLOR
    
   

    SSD_WritePacket(SLPOUT);
    delay_ms(200);
    SSD_WritePacket(RGB);
    delay_ms(10);
    SSD_WritePacket(FLIP);
    delay_ms(10);
    //SSD_WritePacket(INV);
    delay_ms(10);

}