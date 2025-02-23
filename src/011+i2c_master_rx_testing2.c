/*
 * 011i2c_master_rx_testing.c
 *
 *  Created on: Jan 23, 2021
 *      Author: Cubesat58
 */

#include "stm32f446xx_driver.h"
#include <string.h>
#include<string.h>
extern void initialise_monitor_handles();

/*
 * I2C1
 * PB6 ->  SCL
 * PB9 ->  SDA

 */

#define MY_ADDR 0x61
#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

int main(void)
{

	uint8_t commandcode;

	uint8_t len;

	initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();
	GPIOLED_Init();
	//i2c peripheral configuration

	I2C1Handle=I2C_Inits(I2C1,I2C_ACK_ENABLE,MY_ADDR,I2C_FM_DUTY_2,I2C_SCL_SPEED_SM);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,Enable);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(  GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = 0x51;

		I2C_MasterSendData(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle,&len,1,SLAVE_ADDR,I2C_ENABLE_SR);

		commandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR);


		I2C_MasterReceiveData(&I2C1Handle,rcv_buf,len,SLAVE_ADDR,I2C_DISABLE_SR);

		rcv_buf[len+1] = '\0';

		printf("Data : %s",rcv_buf);

	}

}
