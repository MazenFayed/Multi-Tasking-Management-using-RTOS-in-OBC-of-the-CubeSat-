/*

 * UART.c
 *
 *  Created on: Apr 15, 2022
 *      Author: MAZEN FAYED
 */


#include "stm32f446xx_driver.h"
#include <stdint.h>
#include <stdio.h>

USART_Handle_t usart2_handle;

GPIO_RegDef_t *pGPIOA11 = GPIOA ;
void delay_ms (int delay );
void USART2_INIT (void);
void USART_WRITE (int ch);
uint32_t USART_READ (int ch);


char msg[1024] = "UART Tx testing...\n\r";
#define MY_ADDR 0x61;
#define SLAVE_ADDR  0x68


int main(void)
{
	USART2_INIT();
	while(1)
	{
		USART_WRITE('H');
		USART_WRITE('I');
		delay_ms(10);
		USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));

	}
}

void USART2_INIT (void)
{
	/*/GPIOA_PCLK_EN()	;                                         //ENABLE GPIOA CLOCK
	USART2_PCLK_EN();/*/
RCC->AHB1ENR |=1;
RCC->APB1ENR |=0x20000;

usart2_handle.pUSARTx = USART2;

	//set pa2 to alf type uart_tx and rx(af07)
	pGPIOA11->AFR[0] =0x0700 ;
	pGPIOA11->MODER |=0x0020 ;
	usart2_handle.pUSARTx->BRR =0x0683 ;  //9600 @16mhz

	usart2_handle.pUSARTx->CR1 =0x008 ; //enable tx and rx

	usart2_handle.pUSARTx->CR1 |= 0x2000 ;

}
void USART_WRITE (int ch)
{
	//wait while tx buffer is empty
	while ( !(usart2_handle.pUSARTx->SR & 0x0080)){}
	usart2_handle.pUSARTx->DR =(ch & 0xFF) ;

}

uint32_t USART_READ (int ch)
{
	//wait while Rx buffer is empty
	while ( !(usart2_handle.pUSARTx->SR & 0x0020)){}

	return (usart2_handle.pUSARTx->DR) ;
}


void delay_ms (int delay )
{
	int i,j ;
	for (j=delay; j>0 ; j--)
	{
		for(i=0;i<3195;i++);
	}
}

