/*
 * i2c.c
 *
 *  Created on: Apr 18, 2022
 *      Author: comp 59
 */
#include "stm32f446xx_driver.h"
#include <stdint.h>
#include <stdio.h>

void delay_ms (int delay );
void i2c_init (void);
int i2c_read (char saddr , char maddr , char *data);
void USART2_INIT (void);

GPIO_RegDef_t *pGPIO_USART=GPIOA;
USART_Handle_t usart2_handle;

GPIO_RegDef_t *pGPIOBB = GPIOB ;
I2C_Handle_t I2C1Handle;
GPIO_RegDef_t *pGPIOAA = GPIOA ;
uint8_t rcv_buf[32];
#define SLAVE_ADDR  0x68


int main()
{
	i2c_init();
	USART2_INIT();
	RCC->AHB1ENR |= 1;

	pGPIOAA->MODER &= ~0x00000C00;
	pGPIOAA->MODER |=  0x00000400;

	char data;

	while(1)
	{
		i2c_read(SLAVE_ADDR,0,&data);

		if(data & 1)
			pGPIOAA->ODR |=  0x00000020;
		else
			pGPIOAA->ODR  &= ~0x00000020;
		delay_ms(10);

		I2C_MasterReceiveData(&I2C1Handle,rcv_buf,32,SLAVE_ADDR,I2C_DISABLE_SR);
					USART_SendData(&usart2_handle,(uint8_t*)rcv_buf,32);
	}


}

void i2c_init (void)
{
	//scl pb8
	//sda pb9
	RCC->AHB1ENR |=2;
	RCC->APB1ENR |=0x00200000;

	pGPIOBB->MODER &= ~0x000F0000;
	pGPIOBB->MODER |=  0x000A0000;

	pGPIOBB->AFR[1] &= ~0x000000FF;
	pGPIOBB->AFR[1] |=  0x00000044;

	pGPIOBB->OTYPER |= 0x00000300;

	pGPIOBB->PUPDR  &= ~0x000F0000;
	pGPIOBB->PUPDR  |=  0x00050000;

	I2C1Handle.pI2Cx =I2C1 ;
	I2C1Handle.pI2Cx->CR1  =   0x8000;
	I2C1Handle.pI2Cx->CR1  &= ~0x8000;
	I2C1Handle.pI2Cx->CR2  =   0x0010 ;
	I2C1Handle.pI2Cx->CCR  =    80 ;
	I2C1Handle.pI2Cx->TRISE =    17;
	I2C1Handle.pI2Cx->CR1 |=    0x0001 ;


}
int i2c_read (char saddr , char maddr , char *data)
{
	volatile int tmp ;
	while (I2C1Handle.pI2Cx->SR2 & 2);

	I2C1Handle.pI2Cx->CR1 |=0x100;
	while(!(I2C1Handle.pI2Cx->SR1 & 1)){};

	I2C1Handle.pI2Cx->DR = saddr<<1 ;
	while(!(I2C1Handle.pI2Cx->SR1 & 2)){};
	tmp =I2C1Handle.pI2Cx->SR2 ;

	while(!(I2C1Handle.pI2Cx->SR1 & 0x80)){};
	I2C1Handle.pI2Cx->DR=maddr;
	while(!(I2C1Handle.pI2Cx->SR1 & 0x80)){};

	I2C1Handle.pI2Cx->CR1 |=0x100;
	while(!(I2C1Handle.pI2Cx->SR1 & 1)){};
	I2C1Handle.pI2Cx->DR = saddr<<1 | 1 ;

	while(!(I2C1Handle.pI2Cx->SR1 & 2)){};
	I2C1Handle.pI2Cx->CR1 &= ~0x400;
	tmp =I2C1Handle.pI2Cx->SR2 ;

	I2C1Handle.pI2Cx->CR1 |=0x200;

	while(!(I2C1Handle.pI2Cx->SR1 & 0x40)){};

		*data++ = I2C1Handle.pI2Cx->DR ;

return 0 ;

}

void delay_ms (int delay )
{
	int i,j ;
	for (j=delay; j>0 ; j--)
	{
		for(i=0;i<3195;i++);
	}
}
void USART2_INIT (void)
	{

	RCC->AHB1ENR |=1;
	RCC->APB1ENR |=0x20000;

	usart2_handle.pUSARTx = USART2;

		//set pa2 & pa3 to alf type uart_tx and rx(af07)
	pGPIO_USART->AFR[0] =0x7700 ;
	pGPIO_USART->MODER |=0x00A0 ;
		usart2_handle.pUSARTx->BRR =0x0683 ;  //9600 @16mhz

		usart2_handle.pUSARTx->CR1 =0x000C ; //enable tx and rx

		usart2_handle.pUSARTx->CR1 |= 0x2000 ;

	}


