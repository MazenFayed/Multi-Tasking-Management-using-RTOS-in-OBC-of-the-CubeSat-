/*
 * stm32f446xx_adc.c


 *
 *  Created on: Feb 11, 2022
 *      Author: MAZEN FAYED
 */


#include "stm32f446xx_adc.h"

#include <stdint.h>
#include <stddef.h>

#define ADC1_CH1                     (1U<<0)              //PUT NUMBER OF CHANNEL ONE IN SEQUENTIAL REGSTER
#define ADC_SEQ_LEN_1                0x00                 // SET LENGTH OF SERVED CHANNELS TO ONLY ONE CHANNEL
#define CR2_ADON                     (1U<<0)               // ENABLE ADC1
#define CR2_SWSTART                  (1U<<30)              //START CONVERSION
#define SR_EOC                       (1U<<1)                // END OF CONVERSION FLAG
#define CR2_CONT                     (1U<<1)                //TO ENABLE CONTIOOS CONVERSION



GPIO_RegDef_t *pGPIOA2 = GPIOA ;
	ADC_RegDef_t *pADC1 = ADC1 ;


	GPIO_RegDef_t *pGPIOA_ADC2 = GPIOA ;
		ADC_RegDef_t *pADC2 = ADC2 ;


   /*
    *
    * SINGLE CONVERSION CODE
    *
    * */


void pa1_adc1_ch1_init(void)
{


	 GPIOA_PCLK_EN() ;                                //ENABLE CLOCK TO GPIOA

	 pGPIOA2->MODER |= (1U<<2);                         //SET MODE TO GPIOA 1 AS ANALOG
	 pGPIOA2->MODER |= (1U<<3);

	 ADC1_PCLK_EN() ;                                 //ENABLE CLOCK TO ADC 1

	 ADC1->ADC_SQR3 = ADC1_CH1 ;                        // SET SEQUENTIAL START CHANNEL

	 ADC1->ADC_SQR1 = ADC_SEQ_LEN_1 ;                   // SET NUMBER OF CHANNELS

	 ADC1->ADC_CR2 |= CR2_ADON ;                                //ENABLE ADC 1



}

void pa4_adc2_ch4_init(void)
{


	 GPIOA_PCLK_EN() ;                                //ENABLE CLOCK TO GPIOA

	 pGPIOA_ADC2->MODER |= (1U<<8);                         //SET MODE TO GPIOA 4 AS ANALOG
	 pGPIOA_ADC2->MODER |= (1U<<9);

	 ADC2_PCLK_EN() ;                                 //ENABLE CLOCK TO ADC 1

	 ADC2->ADC_SQR3 = ADC1_CH1 ;                        // SET SEQUENTIAL START CHANNEL

	 ADC2->ADC_SQR1 = ADC_SEQ_LEN_1 ;                   // SET NUMBER OF CHANNELS

	 ADC2->ADC_CR2 |= CR2_ADON ;                                //ENABLE ADC 1



}

void start_conversion(void)
{

	/*
	 *
    *ADC1->ADC_CR2 |= CR2_CONT ;               //ENABLE CONTINOUS CONVERSION AUTOMATICALLY
    *
    */


	ADC1->ADC_CR2 |= CR2_SWSTART ;                          //START ADC CONVERSION
	ADC2->ADC_CR2 |= CR2_SWSTART ;                          //START ADC CONVERSION


}



uint32_t adc1_read (void)
{
	while(!(ADC1->ADC_SR & SR_EOC )){}                  //WAIT FOR CONVERSION TO BE COMPLETION

	return(ADC1->ADC_DR );                              //READ CONVERTED RESULT

}

uint32_t adc2_read (void)
{
	while(!(ADC2->ADC_SR & SR_EOC )){}                  //WAIT FOR CONVERSION TO BE COMPLETION

	return(ADC2->ADC_DR );                              //READ CONVERTED RESULT

}

