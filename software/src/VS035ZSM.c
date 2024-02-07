#include "VS035ZSM.h"


void VS035ZSM_SSD2828_write_cfg(){
    //disable the PLL
    ssd2828_SPI_write_reg(PCR_REG, 0x0000);
    //set PLL freq to 336MHZ 
    //ssd2828_SPI_write_reg(PLCR_REG, 0x0000 | (2U << PLCR_FR_POS) | (1U << PLCR_MS_POS) | (14U << PLCR_NS_POS));
    //set divider for low speed to 8.4MHZ
    //ssd2828_SPI_write_reg(CCR_REG, 0x0000 | (4U << CCR_LPD_POS));


	//set PLL freq to 912MHZ 
    ssd2828_SPI_write_reg(PLCR_REG, 0x0000 | (3U << PLCR_FR_POS) | (1U << PLCR_MS_POS) | (38U << PLCR_NS_POS));
    //set divider for low speed to 8.4MHZ
    ssd2828_SPI_write_reg(CCR_REG, 0x0000 | (11U << CCR_LPD_POS));


    //enable the PLL
    ssd2828_SPI_write_reg(PCR_REG, 0x0000 | (1U << PCR_PEN_POS));
    //wait 2 ms to stabilize
    delay_ms(5);
    



    //set VSYNC HSYNC period
    ssd2828_SPI_write_reg(VICR1_REG, 0x0000 | (VS035ZSM_VSYNC << VICR1_VSA_POS) | (VS035ZSM_VSYNC << VICR1_HSA_POS) );
    //set VBP HBP
	ssd2828_SPI_write_reg(VICR2_REG, 0x0000 | (VS035ZSM_VBP << VICR2_VBP_POS) | (VS035ZSM_HBP << VICR2_HBP_POS) );
    //set VFP HFP
	ssd2828_SPI_write_reg(VICR3_REG, 0x0000 | (VS035ZSM_VFP << VICR3_VFP_POS) | (VS035ZSM_HFP << VICR3_HFP_POS) );
    //set horizontal active
	ssd2828_SPI_write_reg(VICR4_REG, 0x0000 | (VS035ZSM_H_ACTIVE << VICR4_HACT_POS) );
    //set vertical active
	ssd2828_SPI_write_reg(VICR5_REG, 0x0000 | (VS035ZSM_V_ACTIVE << VICR5_VACT_POS) );
    //set color depth 24bit and burst mode
	ssd2828_SPI_write_reg(VICR6_REG, 0x0000 | (3U << VICR6_VPF_POS) | (2U << VICR6_VM_POS) );

    //set lanes number to 4
    ssd2828_SPI_write_reg(LCFR_REG, 0x0003);
    
    //set MIPI packet format
    //enable EOT DCS HS
    //disable Read Video
    ssd2828_SPI_write_reg(CFGR_REG, 0x0000 | (1U << CFGR_EOT_POS) | (1U << CFGR_DCS_POS) | (1U << CFGR_HS_POS));
    //set virtual channel 0x0000
    ssd2828_SPI_write_reg(VCR_REG, 0x0000);

	//set MIPI delay timings
	//set HS Zero Delay and HS Prepare Delay
	//ssd2828_SPI_write_reg(DAR1_REG, 0x0000 | (12U << DAR1_HZD_POS) | (2U << DAR1_HPD_POS));
	//set CLK Zero Delay and CLK Prepare Delay
	//ssd2828_SPI_write_reg(DAR2_REG, 0x0000 | (25U << DAR2_CZD_POS) | (2U << DAR2_CPD_POS));
	//set CLK Pre Delay and CLK Post Delay
	//ssd2828_SPI_write_reg(DAR3_REG, 0x0000 | (4U << DAR3_CPED_POS) | (24U << DAR3_CPTD_POS));
	//set CLK Trail Delay and HS Trail Delay
	//ssd2828_SPI_write_reg(DAR4_REG, 0x0000 | (6U << DAR4_CTD_POS) | (8U << DAR4_HTD_POS));
	//set TA Go Delay and TA GET Delay
	//ssd2828_SPI_write_reg(DAR6_REG, 0x0000 | (4U << DAR6_TGO_POS) | (5U << DAR6_TGET_POS));





	//set MIPI delay timings
	//set HS Zero Delay and HS Prepare Delay
	ssd2828_SPI_write_reg(DAR1_REG, 0x0000 | (32U << DAR1_HZD_POS) | (14U << DAR1_HPD_POS));
	//set CLK Zero Delay and CLK Prepare Delay
	ssd2828_SPI_write_reg(DAR2_REG, 0x0000 | (68U << DAR2_CZD_POS) | (10U << DAR2_CPD_POS));
	//set CLK Pre Delay and CLK Post Delay
	ssd2828_SPI_write_reg(DAR3_REG, 0x0000 | (9U << DAR3_CPED_POS) | (65U << DAR3_CPTD_POS));
	//set CLK Trail Delay and HS Trail Delay
	ssd2828_SPI_write_reg(DAR4_REG, 0x0000 | (15U << DAR4_CTD_POS) | (20U << DAR4_HTD_POS));
	//set TA Go Delay and TA GET Delay
	ssd2828_SPI_write_reg(DAR6_REG, 0x0000 | (5U << DAR6_TGO_POS) | (6U << DAR6_TGET_POS));

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
    delay_ms(35);
    //pull RST high
    VS035ZSM_C_PORT->ODR |= (1U << VS035ZSM_RST);
    //set MIPI lanes to LP-11

    //wait 15ms
    delay_ms(25);
   

}

void VS035ZSM_start(){
	
    

	//HSSRAM parameter
	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xFF;
		short_dcs.param2 = 0xE0;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xFF, 0xE0);

	//RELOAD
	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xFB;
		short_dcs.param2 = 0x01;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xFB, 0x01);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0x53;
		short_dcs.param2 = 0x22;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0x53, 0x22);

	//nitial Code Release: CDM2
	//Command Page
	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xFF;
		short_dcs.param2 = 0x25;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xFF, 0x25);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xFB;
		short_dcs.param2 = 0x01;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xFB, 0x01);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0x65;
		short_dcs.param2 = 0x01;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0x65, 0x01);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0x66;
		short_dcs.param2 = 0x50;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0x66, 0x50);

	//SETTING 10% DUTY CYCLE
	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0x67;
		short_dcs.param2 = 0x55;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0x67, 0x55);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xC4;
		short_dcs.param2 = 0x90;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xC4, 0x90);

	//Command Page
	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xFF;
		short_dcs.param2 = 0x26;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xFF, 0x26);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xFB;
		short_dcs.param2 = 0x01;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xFB, 0x01);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0x02;
		short_dcs.param2 = 0xB5;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0x02, 0xB5);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0x4D;
		short_dcs.param2 = 0x8B;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0x4D, 0x8B);

	//Initial Code Release: CDM2
	//Command Page
	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xFF;
		short_dcs.param2 = 0x10;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xFF, 0x10);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xFB;
		short_dcs.param2 = 0x01;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xFB, 0x01);

	//User Command Set
	//PROBABLY THE VESA DSC SETTING
	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xC0;
		short_dcs.param2 = 0x80;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xC0, 0x80);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		long_dcs.mipi_port = dcs_port_seq[i];
		long_dcs.data_type = DATALONG_GEN_WRITE;
		long_dcs.word_count = 0x0005;
		long_dcs.pData = &dsc_long1[0];
		mipi_packet_send_long(&long_dcs);
	}*/
	uint8_t dsc_long1[4] = {0x00, 0x0A, 0x00, 0x0A};
	//ssd2828_MIPI_write_generic_long_p(0x3B, dsc_long1, 4);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		long_dcs.mipi_port = dcs_port_seq[i];
		long_dcs.data_type = DATALONG_GEN_WRITE;
		long_dcs.word_count = 0x0005;
		long_dcs.pData = &dsc_long2[0];
		mipi_packet_send_long(&long_dcs);
	}*/
	uint8_t dsc_long2[4] = {0x00, 0x0A, 0x00, 0x0A};
	//ssd2828_MIPI_write_generic_long_p(0xBE, dsc_long2, 4);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xBB;
		short_dcs.param2 = 0x13;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xBB, 0x13);


	//SETTING BA REGISTER FOR DUAL PORT OPERATION 0x30
	//SET TO 0x07 FOR SINGLE PORT 
	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0xBA;
		short_dcs.param2 = 0x03;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xBA, 0x07);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0x35;
		short_dcs.param2 = 0x00;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0x35, 0x00);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port = dcs_port_seq[i];
		short_dcs.data_type = DATASHORT_DCS_WRITE_1;
		short_dcs.param1 = 0x36;
		short_dcs.param2 = 0x00;
		short_dcs.pData = NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0x36, 0x00);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		long_dcs.mipi_port = dcs_port_seq[i];
		long_dcs.data_type = DATALONG_GEN_WRITE;
		long_dcs.word_count = 0x0005;
		long_dcs.pData = &dsc_long3[0];
		mipi_packet_send_long(&long_dcs);
	}*/
	uint8_t dsc_long3[4] = {0x00, 0x00, 0x06, 0x40};
	//ssd2828_MIPI_write_generic_long_p(0x2B, dsc_long3, 4);


	
	//Backlight should be turned on here but looks like it still works if its enabled earlier

	// Sleep Out
	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		dcs.mipi_port = dcs_port_seq[i];
		dcs.data_type = DATASHORT_DCS_WRITE_0;
		dcs.param1 = DCS_exit_sleep_mode;
		dcs.param2 = 0;
		dcs.pData = NULL;
		mipi_packet_send_short(&dcs);
		delay_ms(5);
	}*/
	ssd2828_MIPI_write_DCS_short_np(0x11);
	delay_ms(200);



	// Display data transfer
	// Display On
	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port		= dcs_port_seq[i];
		short_dcs.data_type		= DATASHORT_DCS_WRITE_1;
		short_dcs.param1		= 0xFF;
		short_dcs.param2		= 0x10;
		short_dcs.pData			= NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_p(0xFF, 0x10);

	/*
	for(uint8_t i = 0; i < MIPI_TOTAL_PORT; i++){
		short_dcs.mipi_port		= dcs_port_seq[i];
		short_dcs.data_type		= DATASHORT_DCS_WRITE_0;
		short_dcs.param1		= DCS_set_display_on;
		short_dcs.param2		= 0;
		short_dcs.pData			= NULL;
		mipi_packet_send_short(&short_dcs);
	}*/
	ssd2828_MIPI_write_DCS_short_np(0x29);

	// Wait, >= 40ms
	delay_ms(40);

	// Wait, >= 40ms
	delay_ms(40);
	
	// Turn ON backlight
	// CHICAGO_PANEL_V33_ON();
	
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
    TIM1->CCMR1 |= (0b111 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    //APB2 clk is @84MHZ
    //set prescaler for 10KHZ as counter clock
    TIM1->PSC = (8400U - 1U);
    //set frequency for 60hz
    TIM1->ARR = 166U;
    //set duty cycle for 10%
    TIM1->CCR1 = TIM1->ARR - (TIM1->ARR / 10);
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

void VS035ZSM_vid(){
	ssd2828_SPI_write_reg(CFGR_REG, 0x0000 | (1U << CFGR_HS_POS) | (1U << CFGR_CKE_POS) | (1U << CFGR_VEN_POS) | (1U << CFGR_DCS_POS) | (1U << CFGR_EOT_POS) | (1U << CFGR_ECD_POS));
	ssd2828_SPI_write_reg(TMR_REG, 0x0000 | (1U << TMR_VBIST_EN_POS) | (1U << TMR_VBIST_SRT_POS)); 
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