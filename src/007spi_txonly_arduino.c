/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Jan 23, 2021
 *      Author: Cubesat58
 */
#include "stm32f446xx_driver.h"
#include <string.h>


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}
/*
 * A4 ->  SPI1_NSS
 * A5 ->  SPI1_SCK
 * A6 ->  SPI1_MISO
 * A7 ->  SPI1_MOSI
 */


void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_AlTFN;
	SPIPins.GPIO_Pinconfig.GPIO_PinAltFunMode = GPIO_MODE_IT_RT;
	SPIPins.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	//SCLK
	SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);
/*
	//MISO
	SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);

*/
	//NSS
	SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);


}

void SPI1_Inits(void)
{

	SPI_Handle_t SPI1handle;

	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;//generates sclk of 8MHz
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //H/W slave management enabled for NSS pin

	SPI_Init(&SPI1handle);
}

int main ()
{
		GPIO_ButtonInit();

		char user_data[] = "An Arduino Uno board is best suited for beginners who have just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiasts who require a lot of I/O pins for their projects";



		//this function is used to initialize the GPIO pins to behave as SPI1 pins
		SPI1_GPIOInits();

		//This function is used to initialize the SPI1 peripheral parameters
		SPI1_Inits();
		/*
		* making SSOE 1 does NSS output enable.
		* The NSS pin is automatically managed by the hardware.
		* i.e when SPE=1 , NSS will be pulled to low
		* and NSS pin will be high when SPE=0
		*/
		SPI_SSOEConfig(SPI1,Enable);
		while(1)
		{
			//wait till button is pressed
			while(   GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );
			//to avoid button de-bouncing related issues 200ms of delay
			delay();

			//enable the SPI2 peripheral
			SPI_PeripheralControl(SPI1,Enable);

			//first send length information
			uint8_t dataLen = strlen(user_data);
			SPI_SendData(SPI1,&dataLen,1);

			//to send data
			SPI_SendData(SPI1,(uint8_t*)user_data,strlen(user_data));

			//lets confirm SPI is not busy
			while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );

			//Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI1,Disable);
		}


	return 0 ;
}
