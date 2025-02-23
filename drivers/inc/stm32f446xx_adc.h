/*
 * stm32f446xx_adc.h


 *
 *  Created on: Feb 11, 2022
 *      Author: MAZEN FAYED
 */




#ifndef INC_STM32F446XX_ADC_H_
#define INC_STM32F446XX_ADC_H_



#include "stm32f446xx_driver.h"
#include <stdint.h>
#include <stddef.h>


void pa1_adc1_ch1_init(void) ;
void start_conversion(void);
uint32_t adc1_read (void) ;
uint32_t adc2_read (void) ;




#endif /* INC_STM32F446XX_ADC_H_ */
