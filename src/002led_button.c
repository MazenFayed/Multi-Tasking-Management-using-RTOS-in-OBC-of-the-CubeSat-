/*
 * 002led_button.c
 *
 *  Created on: Jan 23, 2021
 *      Author: Cubesat58
 */
#include "stm32f446xx_driver.h"

#define LOW 0
#define BTN_PRESSED LOW

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}
void DRV_Init_Pin(void)
{
	GPIO_Handle_t GpioLed;

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//GPIO_PeriClockControl(GPIOA,Enable);

	GPIO_Init(&GpioLed);
}
void DRV_Btn_Init(void)
{
	GPIO_Handle_t  GPIOBtn;
	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//GPIO_PeriClockControl(GPIOC,Enable);
	GPIO_Init(&GPIOBtn);}

int main(void)
{

	DRV_Init_Pin();
	DRV_Btn_Init();

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			//delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		}
	}
	return 0;
}
