/*
 * 004gpio_freq.c
 *
 *  Created on: Jan 23, 2021
 *      Author: Cubesat58
 */

#include "stm32f446xx_driver.h"
void delay (int N)
{
	for(uint32_t i =0 ; i<N*1000;i++);
}
void DRV_Init_Pin(void)
{
	GPIO_Handle_t gpioled ;
	gpioled.pGPIOx=GPIOA;
	gpioled.GPIO_Pinconfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	gpioled.GPIO_Pinconfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
	gpioled.GPIO_Pinconfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	gpioled.GPIO_Pinconfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	gpioled.GPIO_Pinconfig.GPIO_PinPuPdControl=GPIO_NO_PUPD; // could by GPIO_PIN_PU  with GPIO_OP_TYPE_OD but need external high resistor
	GPIO_PeriClockControl(GPIOA,Enable);
	GPIO_Init(&gpioled);
}
int main()
{
	DRV_Init_Pin();
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		//delay(1000);
	}




	return 0 ;
}
