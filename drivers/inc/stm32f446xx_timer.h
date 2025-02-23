/*
 * stm32f446xx_timer_driver.h

 *
 *  Created on: Feb 8, 2022
 *      Author: MAZEN FAYED
 */

#ifndef INC_STM32F446XX_TIMER_DRIVER_H_
#define INC_STM32F446XX_TIMER_DRIVER_H_

#include "stm32f446xx_driver.h"

void tim2_1hz_init(void);

void tim2_pa5_output_compare(void)    ;

#define  SR_UIF       (1U<<0)               // FLAG BIT WHEN TIMER RWLOADED AGAIN

#endif /* INC_STM32F446XX_TIMER_DRIVER_H_ */
