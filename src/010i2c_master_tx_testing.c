/*
 * 01I2C_Master_tx_testing.c
 *
 *  Created on: Jan 21, 2021
 *      Author: Cubesat58
 */

#include "stm32f446xx_driver.h"
#include <string.h>
#include<string.h>


/*
 * I2C1
 * PB6 ->  SCL
 * PB9 ->  SDA

 */

#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";


void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_AlTFN;
	I2CPins.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_Pinconfig.GPIO_PinAltFunMode = GPIO_MODE_IT_FT;
	I2CPins. GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}
int main(void)
{

	GPIO_ButtonInit();
	GPIOLED_Init();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,Enable);

	while(1)
	{
		//wait till button is pressed
		GPIO_WriteToOutputPin(GPIOA,GPIO_PIN_NO_5,0);
		while( GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );
		GPIO_WriteToOutputPin(GPIOA,GPIO_PIN_NO_5,1);

		//to avoid button de-bouncing related issues 200ms of delay
		delay();


		//send some data to the slave
		I2C_MasterSendData(&I2C1Handle,some_data,strlen((char*)some_data),SLAVE_ADDR,I2C_ENABLE_SR);

	}

}

