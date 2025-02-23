/*
 * stm32f446xx_dac.h

 *
 *  Created on: Feb 11, 2022
 *      Author: MAZEN FAYED
 */

#ifndef INC_STM32F446XX_DAC_H_
#define INC_STM32F446XX_DAC_H_

#include "stm32f446xx_driver.h"
#include <stdint.h>
#include <stddef.h>


void pa4_dac1_ch1_init(void) ;
void dac_write( uint32_t p);

#endif /* INC_STM32F446XX_DAC_H_ */
