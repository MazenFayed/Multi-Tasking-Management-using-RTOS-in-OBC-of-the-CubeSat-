/*
 * 01I2C_Master_tx_testing.c
 *
 *  Created on: Jan 21, 2021
 *      Author: Cubesat58
 */

#include "stm32f446xx_driver.h"
#include <string.h>
#include<string.h>


#define MY_ADDR 0x61

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

int main(void)
{

	GPIO_ButtonInit();
	GPIOLED_Init();
	//i2c peripheral configuration

	I2C1Handle=I2C_Inits(I2C1,I2C_ACK_ENABLE,MY_ADDR,I2C_FM_DUTY_2,I2C_SCL_SPEED_SM);

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

