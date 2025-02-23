/*
 * 009spi_cmd_handling_it.c
 *
 *  Created on: Jan 23, 2021
 *      Author: Cubesat58
 */

#include<stdio.h>
#include<string.h>
#include "stm32f446xx_driver.h"



//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9

SPI_Handle_t SPI1handle;


uint8_t RcvBuff[100];

uint8_t ReadByte;

uint8_t RxContFlag = Reset;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if(ackbyte == (uint8_t)0xF5)
	{
		//ack
		return 1;
	}

	return 0;
}

int main(void)
{


	//this function is used to initialize the GPIO User Button
	GPIO_ButtonInit();
	//this function is used to initialize the GPIO User LED
	GPIOLED_Init();
	//this function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI_Inits(SPI1,SPI_BUS_CONFIG_FD,SPI_DEVICE_MODE_MASTER,SPI_SCLK_SPEED_DIV8,SPI_DFF_8BITS,SPI_SSM_DI);


	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI1,Enable);

	SPI_IRQInterruptConfig(IRQ_NO_SPI1,Enable);

	//wait till button is pressed
	while(  GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

	//to avoid button de-bouncing related issues 200ms of delay
	delay();


	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI1,Enable);

	RxContFlag = set;

	while(RxContFlag == set)
	{
	   while ( ! (SPI_ReceiveDataIT(&SPI1handle,&ReadByte,1) == SPI_READY) );
	}


	//lets confirm SPI is not busy
	while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI1,Disable);

	return 0;

}

void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&SPI1handle);
}



void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i =0;
	static uint8_t  rcv_start = 0;
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		if(ReadByte == 0XF1)
		{
			rcv_start = 1;
		}else
		{
			if(rcv_start)
			{
				if(ReadByte == '\r')
				{
					RxContFlag = Reset;
					rcv_start =0;
					RcvBuff[i++] = ReadByte; //place the \r
					i=0;
				}else
				{
					RcvBuff[i++] = ReadByte;

				}
			}
		}


	}

}


