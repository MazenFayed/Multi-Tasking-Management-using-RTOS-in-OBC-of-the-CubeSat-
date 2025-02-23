/*
 * stm32f446xx_dac.c


 *
 *  Created on: Feb 11, 2022
 *      Author: MAZEN FAYED
 */


#include "stm32f446xx_adc.h"

#include <stdint.h>
#include <stddef.h>


GPIO_RegDef_t *pGPIOA3 = GPIOA ;
DAC_RegDef_t *PDAC1 = DAC ;


#define CR_EN                       (1U<<0)                  //ENABLE CHANNEL 1
#define CR_TEN1                     (1U<<2)                 // ENABLE TRIGGER FOR CHANNEL 1
#define CR_TSEL                                            //CHOOSE SOFTWARE TRIGGER IN BITS 3,4,5



void pa4_dac1_ch1_init(void)
{
	 GPIOA_PCLK_EN() ;                                //ENABLE CLOCK TO GPIOA

	 pGPIOA3->MODER |= (1U<<2);                         //SET MODE TO GPIOA 1 AS ANALOG TO BE DAC_OUT PIN
	 pGPIOA3->MODER |= (1U<<3);

	 DAC_PCLK_EN() ;                                  //ENABLE DAC CLOCK

	 DAC->DAC_CR |= CR_EN  ;                            // ENABLE CHANNEL 1



}



void dac_write( uint32_t p)
{

	DAC->DAC_DHR12R1 = p ;
}

