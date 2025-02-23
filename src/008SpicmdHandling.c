/*
 * 008SpiHandling.c
 *
 *  Created on: Jan 15, 2021
 *      Author: Cubesat58
 */


#include "stm32f446xx_spi_driver.h"
#include <string.h>
#include <stdlib.h>
#include<stdio.h>

extern void initialise_monitor_handles();

/*
 * A4 ->  SPI1_NSS
 * A5 ->  SPI1_SCK
 * A6 ->  SPI1_MISO
 * A7 ->  SPI1_MOSI
 */


//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//Arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9

void delay(void)
{
	for(uint32_t i = 0 ; i < 500/2 ; i ++);
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
	SPIPins.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);


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
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //software slave management enabled for NSS pin

	SPI_Init(&SPI1handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOCtn,GpioLed;

	//this is btn gpio configuration
	GPIOCtn.pGPIOx = GPIOC;
	GPIOCtn.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOCtn.GPIO_Pinconfig.GPIO_PinMode = GPIO_MOD_INPUT;
	GPIOCtn.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOCtn.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOCtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,Enable);

	GPIO_Init(&GpioLed);

}


uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if(ackbyte == (uint8_t)0xF5)
	{
		//ACK
		return 1;
	}

	return 0;
}

int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI1_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI1_Inits();

	printf("SPI Init. done\n");

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
		while(  GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI1,Enable);

	    //1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI1,&commandcode,1);
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//send arguments
			SPI_SendData(SPI1,args,2);
			printf("COMMAND_LED_CTRL Executed\n");
		}
		//end of COMMAND_LED_CTRL

		//2. CMD_SENOSR_READ   <analog pin number(1) >

		//wait till button is pressed
		while(  GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI1,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI1,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI1,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI1,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI1,&analog_read,1);
			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}

		//3.  CMD_LED_READ 	 <pin no(1) >

		//wait till button is pressed
		while(  GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI1,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI1,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI1,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI1,&dummy_write,1);

			uint8_t led_status;
			SPI_ReceiveData(SPI1,&led_status,1);
			printf("COMMAND_READ_LED %d\n",led_status);

		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while(  GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI1,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackbyte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI1,args,1); //sending length

			//send message
			SPI_SendData(SPI1,message,args[0]);

			printf("COMMAND_PRINT Executed \n");

		}

		//5. CMD_ID_READ
		//wait till button is pressed
		while(  GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI1,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackbyte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI1,&dummy_write,1);
				SPI_ReceiveData(SPI1,&id[i],1);
			}

			id[11] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}




		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI1,Disable);

		printf("SPI Communication Closed\n");
	}

	return 0;

}
