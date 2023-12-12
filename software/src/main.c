#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "../system/stm32f4xx.h"

#include "../libs/delay.h"
#include "../libs/spi.h"

#include "ssd2828.h"
#include "clk.h"

#define _1V2_EN         GPIO_ODR_OD2
#define _1V8_EN         GPIO_ODR_OD1
#define _3V0_EN         GPIO_ODR_OD12
#define _3V3_EN         GPIO_ODR_OD0
#define _BACKLIGHT_EN   GPIO_ODR_OD13

#define _RST            GPIO_ODR_OD15

void gpio_init(){
    //enable GPIOC and GPIOA GPIOB GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    //set C13 to output
    GPIOC->MODER |= (0x1<<GPIO_MODER_MODER13_Pos);
    //C13 to push pull
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT13;
    //set C13 speed to low
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR13;
    //set C13 to LOW
    GPIOC->ODR |= GPIO_ODR_OD13;

    //set B0 B1 B2 B12 B13 as outputs
    GPIOB->MODER |= (0x1<<GPIO_MODER_MODER0_Pos) | (0x1<<GPIO_MODER_MODER1_Pos) | (0x1<<GPIO_MODER_MODER2_Pos) | (0x1<<GPIO_MODER_MODER12_Pos) | (0x1<<GPIO_MODER_MODER13_Pos) | (0x1<<GPIO_MODER_MODER15_Pos);
    //set to push pull
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT0 & ~GPIO_OTYPER_OT1 & ~GPIO_OTYPER_OT2 & ~GPIO_OTYPER_OT12 & ~GPIO_OTYPER_OT13 & ~GPIO_OTYPER_OT15; 
    //set speed to low
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR0 & ~GPIO_OSPEEDER_OSPEEDR1 & ~GPIO_OSPEEDER_OSPEEDR2 & ~GPIO_OSPEEDER_OSPEEDR12 & ~GPIO_OSPEEDER_OSPEEDR13  & ~GPIO_OSPEEDER_OSPEEDR15; 
    //set C13 to LOW
    GPIOB->ODR &= ~GPIO_ODR_OD0 & ~GPIO_ODR_OD1 & ~GPIO_ODR_OD2 & ~GPIO_ODR_OD12 & ~GPIO_ODR_OD13& ~GPIO_ODR_OD15;



}


int main(){
    uint16_t i = 0, tmp = 0;
    SystemClock_Config();
    gpio_init();
    delay_init();
    spi_init();
    
    //enable 1V2
    GPIOB->ODR |= _1V2_EN;
    //wait 10ms
    delay_ms(10);
    //enable 3V3
    GPIOB->ODR |= _3V3_EN;
    //wait 20ms 
    delay_ms(20);
    

    delay_ms(100);
    ssd2828_init();

   

    //SPI_WriteCmd(CFGR_REG);
    //SPI_WriteData(0x50);  // 50=TX_CLK 70=PCLK
    //SPI_WriteData(0x00);  // Configuration Register
    tmp = (1<<CFGR_HCLK_POS);
    ssd2828_SPI_write_reg(CFGR_REG, tmp);

    //SPI_WriteCmd(VCR_REG);
    //SPI_WriteData(0x00);
    //SPI_WriteData(0x00);  // VC(Virtual ChannelID) Control Register
    tmp = 0;
    ssd2828_SPI_write_reg(VCR_REG, tmp);

    //SPI_WriteCmd(PCR_REG);
    //SPI_WriteData(0x00);  // 1=PLL disable
    //SPI_WriteData(0x00);
    tmp = 0;
    ssd2828_SPI_write_reg(PCR_REG, tmp);

    //SPI_WriteCmd(PLCR_REG);   // PLL=(TX_CLK/MS)*NS 
    //SPI_WriteData(0x20);  // 14,D7-0=NS(0x01 : NS=1)
    //SPI_WriteData(0x82);  // 42,D15-14=PLL00=62.5-125 01=126-250 10=251-500 11=501-1000  DB12-8=MS(01:MS=1)
    ssd2828_SPI_write_reg(PLCR_REG, 0x0000);
    tmp = (4U<<PLCR_NS_POS) | (1U<<PLCR_MS_POS);    //target 96Mbps
    ssd2828_SPI_write_reg(PLCR_REG, tmp);

    //SPI_WriteCmd(CCR_REG);   // LP Clock Divider LP clock = 400MHz / LPD / 8 = 480 / 8/ 8 = 7MHz
    //SPI_WriteData(0x07);  // D5-0=LPD=0x1 ¨C Divide by 2
    //SPI_WriteData(0x00);
    ssd2828_SPI_write_reg(CCR_REG, 0x0000);
    tmp = (1U<<CCR_LPD_POS);    //target 96Mbps
    ssd2828_SPI_write_reg(CCR_REG, tmp);

    //SPI_WriteCmd(PCR_REG);
    //SPI_WriteData(0x01);  // 1=PLL enable
    //SPI_WriteData(0x00);
    tmp = (1<<PCR_PEN_POS);
    ssd2828_SPI_write_reg(PCR_REG, tmp);

    //MIPI lane configuration

    //SPI_WriteCmd(LCFR_REG);
    //SPI_WriteData(0x00);  // 11=4LANE 10=3LANE 01=2LANE 00=1LANE
    //SPI_WriteData(0x00);
    tmp = 0;
    ssd2828_SPI_write_reg(LCFR_REG, tmp);

    SPI_WriteCmd(DAR1_REG);
    SPI_WriteData(0x02);
    SPI_WriteData(0x23);  // p1: HS-Data-zero  p2: HS-Data- prepare  --> 8031 issue

    delay(100);

    SPI_WriteCmd(DAR2_REG);
    SPI_WriteData(0x01);  // CLK Prepare
    SPI_WriteData(0x23);  // Clk Zero

    SPI_WriteCmd(DAR3_REG);   // local_write_reg(addr=0xCB,data=0x0510)
    SPI_WriteData(0x10);  // Clk Post
    SPI_WriteData(0x05);  // Clk Per

    SPI_WriteCmd(DAR4_REG);   // local_write_reg(addr=0xCC,data=0x100A)
    SPI_WriteData(0x05);  // HS Trail
    SPI_WriteData(0x10);  // Clk Trail

    //LCD driver initialization

    SPI_WriteCmd(CFGR_REG);
    SPI_WriteData(0x50);  // 10=TX_CLK 30=PCLK
    SPI_WriteData(0x02);

    SPI_WriteCmd(PSCR2_REG);
    SPI_WriteData(0x00);
    SPI_WriteData(0x00);

    GP_COMMAD_PA(1);
    SPI_WriteData(0x10); 


    GP_COMMAD_PA(2);       
    SPI_WriteData(0xCD);
    SPI_WriteData(0xAA);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x41);
    SPI_WriteData(0x34);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x30);
    SPI_WriteData(0x00);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x39);
    SPI_WriteData(0x11);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x32);
    SPI_WriteData(0x00);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x33);
    SPI_WriteData(0x38);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x35);
    SPI_WriteData(0x24);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x4F);
    SPI_WriteData(0x35);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x4E);
    SPI_WriteData(0x27);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x41);
    SPI_WriteData(0x56);

    GP_COMMAD_PA(9);
    SPI_WriteData(0x55);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);

    GP_COMMAD_PA(17);
    SPI_WriteData(0x56);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);
    SPI_WriteData(0x00);
    SPI_WriteData(0x0F);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x65);
    SPI_WriteData(0x08);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x3A);
    SPI_WriteData(0x08);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x36);
    SPI_WriteData(0x49);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x67);
    SPI_WriteData(0x82);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x69);
    SPI_WriteData(0x20);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x6C);
    SPI_WriteData(0x80);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x6D);
    SPI_WriteData(0x01);

    GP_COMMAD_PA(20);
    SPI_WriteData(0x53);
    SPI_WriteData(0x1F);
    SPI_WriteData(0x19);
    SPI_WriteData(0x15);
    SPI_WriteData(0x11);
    SPI_WriteData(0x11);
    SPI_WriteData(0x11);
    SPI_WriteData(0x12);
    SPI_WriteData(0x14);
    SPI_WriteData(0x15);
    SPI_WriteData(0x11);
    SPI_WriteData(0x0D);
    SPI_WriteData(0x0B);
    SPI_WriteData(0x0B);
    SPI_WriteData(0x0D);
    SPI_WriteData(0x0C);
    SPI_WriteData(0x0C);
    SPI_WriteData(0x08);
    SPI_WriteData(0x04);
    SPI_WriteData(0x00);

    GP_COMMAD_PA(20);
    SPI_WriteData(0x54);
    SPI_WriteData(0x1F);
    SPI_WriteData(0x19);
    SPI_WriteData(0x15);
    SPI_WriteData(0x11);
    SPI_WriteData(0x11);
    SPI_WriteData(0x11);
    SPI_WriteData(0x13);
    SPI_WriteData(0x15);
    SPI_WriteData(0x16);
    SPI_WriteData(0x11);
    SPI_WriteData(0x0D);
    SPI_WriteData(0x0C);
    SPI_WriteData(0x0C);
    SPI_WriteData(0x0E);
    SPI_WriteData(0x0C);
    SPI_WriteData(0x0C);
    SPI_WriteData(0x08);
    SPI_WriteData(0x04);
    SPI_WriteData(0x00);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x6B);
    SPI_WriteData(0x00);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x58);
    SPI_WriteData(0x00);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x73);
    SPI_WriteData(0xF0);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x76);
    SPI_WriteData(0x40);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x77);
    SPI_WriteData(0x04);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x74);
    SPI_WriteData(0x17);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x5E);
    SPI_WriteData(0x03);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x68);
    SPI_WriteData(0x10);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x6A);
    SPI_WriteData(0x00);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x28);
    SPI_WriteData(0x31);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x29);
    SPI_WriteData(0x21);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x63);
    SPI_WriteData(0x04);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x27);
    SPI_WriteData(0x00);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x7C);
    SPI_WriteData(0x80);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x2E);
    SPI_WriteData(0x05);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x4C);
    SPI_WriteData(0x80);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x50);
    SPI_WriteData(0xC0);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x78);
    SPI_WriteData(0x6E);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x2D);
    SPI_WriteData(0x31);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x49);
    SPI_WriteData(0x00);

    GP_COMMAD_PA(2);
    SPI_WriteData(0x4D);
    SPI_WriteData(0x00); 

    GP_COMMAD_PA(1);
    SPI_WriteData(0x11);
    delay(120);
    GP_COMMAD_PA(1);
    SPI_WriteData(0x29);
    delay(120);

    SPI_WriteCmd(CFGR_REG);
    SPI_WriteData(0x50);
    SPI_WriteData(0x00);  // Configuration Register

    SPI_WriteCmd(VCR_REG);
    SPI_WriteData(0x00);
    SPI_WriteData(0x00);  // VC(Virtual ChannelID) Control Register

    SPI_WriteCmd(PCR_REG);
    SPI_WriteData(0x00);  // 1=PLL disable
    SPI_WriteData(0x00);

    SPI_WriteCmd(PLCR_REG);
    SPI_WriteData(0x20);  // 14,D7-0=NS(0x01 : NS=1)
    SPI_WriteData(0x82);  // 42,D15-14=PLL00=62.5-125 01=126-250 10=251-500 11=501-1000  DB12-8=MS(01:MS=1)

    SPI_WriteCmd(CCR_REG);   // LP Clock Divider LP clock = 400MHz / LPD / 8 = 480 / 8/ 8 = 7MHz
    SPI_WriteData(0x07);  // D5-0=LPD=0x1 ¨C Divide by 2
    SPI_WriteData(0x00);

    SPI_WriteCmd(PCR_REG);
    SPI_WriteData(0x01);  // 1=PLL disable
    SPI_WriteData(0x00);

    SPI_WriteCmd(DAR1_REG);
    SPI_WriteData(0x02);
    SPI_WriteData(0x23);  // p1: HS-Data-zero  p2: HS-Data- prepare  --> 8031 issue
    delay(100);

    SPI_WriteCmd(DAR2_REG);
    SPI_WriteData(0x01);  // CLK Prepare
    SPI_WriteData(0x23);  // Clk Zero

    SPI_WriteCmd(DAR3_REG);   // local_write_reg(addr=0xCB,data=0x0510)
    SPI_WriteData(0x10);  // Clk Post
    SPI_WriteData(0x05);  // Clk Per

    SPI_WriteCmd(DAR4_REG);   // local_write_reg(addr=0xCC,data=0x100A)
    SPI_WriteData(0x05);  // HS Trail
    SPI_WriteData(0x10);  // Clk Trail

    SPI_WriteCmd(HTTR2_REG);
    SPI_WriteData(0x00);
    SPI_WriteData(0x00);     

    //RGB interface configuration

    SPI_WriteCmd(VICR1_REG);
    SPI_WriteData(0x18);  // HSPW 07
    SPI_WriteData(0x02);  // VSPW 05

    SPI_WriteCmd(VICR2_REG);
    SPI_WriteData(0xa0);  // HBPD 0x64=100
    SPI_WriteData(0x0a);  // VBPD 8 ¼õÐ¡ÏÂÒÆ

    SPI_WriteCmd(VICR3_REG);
    SPI_WriteData(0xa0);  // HFPD 8
    SPI_WriteData(0x0c);  // VFPD 10

    SPI_WriteCmd(VICR4_REG);   // Horizontal active period 720=02D0
    SPI_WriteData(0x90);  // 013F=319 02D0=720
    SPI_WriteData(0x01);

    SPI_WriteCmd(VICR5_REG);   // Vertical active period 1280=0500
    SPI_WriteData(0x00);  // 01DF=479 0500=1280
    SPI_WriteData(0x05);


    SPI_WriteCmd(VICR6_REG);   // RGB CLK  16BPP=00 18BPP=01
    SPI_WriteData(0x0b);  // D7=0 D6=0 D5=0  D1-0=11 ¨C 24bpp   //07
    SPI_WriteData(0x00);  // D15=VS D14=HS D13=CLK D12-9=NC D8=0=Video with blanking packet. 00-F0

    //MIPI lane configuration

    SPI_WriteCmd(LCFR_REG);
    SPI_WriteData(0x03);  // 11=4LANE 10=3LANE 01=2LANE 00=1LANE
    SPI_WriteData(0x00);

    SPI_WriteCmd(TR_REG);   // 05=BGR  04=RGB
    SPI_WriteData(0x05);  // D0=0=RGB 1:BGR D1=1=Most significant byte sent first
    SPI_WriteData(0x00);

    SPI_WriteCmd(ACR4_REG);
    SPI_WriteData(0x58);
    SPI_WriteData(0x00);


    SPI_WriteCmd(CFGR_REG);
    SPI_WriteData(0x6B);
    SPI_WriteData(0x02);

    //enable 1V8
    GPIOB->ODR |= _1V8_EN;
    //pull rst high
    GPIOB->ODR |= _RST;
    //wait 250ms 
    delay_ms(250);
    //pull rst low
    GPIOB->ODR &= ~_RST;
    //wait 10ms 
    delay_ms(10);
    //pull rst high
    GPIOB->ODR |= _RST;
    //wait 10ms 
    delay_ms(10);
    //enable 3V0
    GPIOB->ODR |= _3V0_EN;
    //wait 10ms 
    delay_ms(100);
    
    ssd2828_MIPI_write_short_DCS(0x11,0x00,1);
    delay_ms(150);
    ssd2828_MIPI_write_short_DCS(0x29,0x00,1);
    delay_ms(150);
    ssd2828_MIPI_write_short_DCS(0x3A,0x55,1);
    end:
    while(1){
        GPIOB->ODR |= _BACKLIGHT_EN;
        if(i == 0x2828){
            GPIOC->ODR ^= GPIO_ODR_OD13;
            
        }
        i = ssd2828_get_id();
        delay_ms(100);
    }
    return 0;
}