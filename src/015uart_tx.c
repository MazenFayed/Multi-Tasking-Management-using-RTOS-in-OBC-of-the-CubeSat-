/*
 * uart_tx.c
 *
 *  Created on: Jan 22, 2019
 *      Author: admin
 */

#include<stdio.h>
#include<string.h>
#include "stm32f446xx_driver.h"

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart2_handle;

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode =USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void 	USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_AlTFN;
	usart_gpios.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_Pinconfig.GPIO_PinAltFunMode =7;

	//USART2 TX
	usart_gpios.GPIO_Pinconfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	//USART2 RX
	usart_gpios.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);


}

void GPIO_ButtonInitt(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioLed.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,1);

	GPIO_Init(&GpioLed);

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

	GPIO_ButtonInitt();

	USART2_GPIOInit();

    USART2_Init();

	//HAL_UART_Transmit(&usart2_handle,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

   USART_PeripheralControl(USART2,1);

    while(1)
    {
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));

			HAL_UART_Transmit(&usart2_handle,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);



    }

	return 0;
}
