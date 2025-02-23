/*
 * stm32f446xx_rcc.h
 *
 *  Created on: Feb 24, 2021
 *      Author: Cubesat58
 */

#ifndef INC_STM32F446XX_RCC_H_
#define INC_STM32F446XX_RCC_H_

#include "stm32f446xx_driver.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

void RCC_DeInit(void) ;
#endif /* INC_STM32F446XX_RCC_H_ */
