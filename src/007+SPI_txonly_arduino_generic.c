/*
 * 007+SPI_txonly_arduino_generic.c
 *
 *  Created on: Feb 23, 2021
 *      Author: Cubesat58
 */

#include "stm32f446xx_driver.h"
#include <string.h>


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main ()
{
		GPIO_ButtonInit();

		char user_data[] = "An Arduino Uno board is best suited for beginners who have just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiasts who require a lot of I/O pins for their projects";
		//This function is used to initialize the SPI1 peripheral parameters
		SPI_Inits(SPI1,SPI_BUS_CONFIG_FD,SPI_DEVICE_MODE_MASTER,SPI_SCLK_SPEED_DIV8,SPI_DFF_8BITS,SPI_SSM_DI);
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
