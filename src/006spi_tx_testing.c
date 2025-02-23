/*
 * 006spi_tx_testing.c
 *
 *  Created on: Jan 11, 2021
 *      Author: Cubesat58
 */
#include "stm32f446xx_spi_driver.h"
#include <string.h>

/*
 * A4 ->  SPI1_NSS
 * A5 ->  SPI1_SCK
 * A6 ->  SPI1_MISO
 * A7 ->  SPI1_MOSI
 */


/*
 * B9 ->  SPI1_NSS
 * B10 ->  SPI1_SCK
 * B14 ->  SPI1_MISO
 * B15 ->  SPI1_MOSI
 * Alternate functionality  5
 */
void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_AlTFN;
	SPIPins.GPIO_Pinconfig.GPIO_PinAltFunMode = GPIO_MODE_IT_RT;
	SPIPins.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

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


	//NSS
	SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
*/

}

void SPI1_Inits(void)
{

	SPI_Handle_t SPI1handle;

	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;//generates sclk of 8MHz
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //software slave management enabled for NSS pin
	SPI_Init(&SPI1handle);
}
int main ()
{
	char user_data[] = "Hello world";

		//this function is used to initialize the GPIO pins to behave as SPI1 pins
		SPI1_GPIOInits();


		//This function is used to initialize the SPI1 peripheral parameters
		SPI1_Inits();

		//enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1,Enable);

		//this makes NSS signal internally high and avoids MODF error
		SPI_SSIConfig(SPI1,Enable);

		//to send data
		SPI_SendData(SPI1,(uint8_t*)user_data,strlen(user_data));

		//lets confirm SPI is not busy
		//while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		//SPI_PeripheralControl(SPI1,Disable);

		while(1);
	return 0 ;
}
