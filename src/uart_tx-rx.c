/*

 * uart_tx-rx.c
 *
 *  Created on: Apr 16, 2022
 *      Author: MAZEN FAYED
 */

#include "stm32f446xx_driver.h"
#include <stdint.h>
#include <stdio.h>

USART_Handle_t usart2_handle1;

GPIO_RegDef_t *pGPIOAA = GPIOA ;
void delay_ms (int delay );
void USART2_INIT (void);
void USART_WRITE (int ch);
uint32_t USART_READ (int ch);
char msg[1024] = "UART Tx testing YA MAZEN...\n\r";


int main(void)
{
	USART2_INIT();
	while(1)
	{
		USART_WRITE('H');
		USART_WRITE('I');
		delay_ms(10);
		USART_SendData(&usart2_handle1,(uint8_t*)msg,strlen(msg));

	}
}

void USART2_INIT (void)
{
	/*/GPIOA_PCLK_EN()	;                                         //ENABLE GPIOA CLOCK
	USART2_PCLK_EN();/*/
RCC->AHB1ENR |=1;
RCC->APB1ENR |=0x20000;

usart2_handle1.pUSARTx = USART2;

	//set pa2 to alf type uart_tx and rx(af07)
pGPIOAA->AFR[0] =0x7700 ;
pGPIOAA->MODER |=0x00A0 ;
	usart2_handle1.pUSARTx->BRR =0x0683 ;  //9600 @16mhz

	usart2_handle1.pUSARTx->CR1 =0x000C ; //enable tx and rx

	usart2_handle1.pUSARTx->CR1 |= 0x2000 ;

}
void USART_WRITE (int ch)
{
	//wait while tx buffer is empty
	while ( !(usart2_handle1.pUSARTx->SR & 0x0080)){}
	usart2_handle1.pUSARTx->DR =(ch & 0xFF) ;

}

uint32_t USART_READ (int ch)
{
	//wait while Rx buffer is empty
	while ( !(usart2_handle1.pUSARTx->SR & 0x0020)){}

	return (usart2_handle1.pUSARTx->DR) ;
}


void delay_ms (int delay )
{
	int i,j ;
	for (j=delay; j>0 ; j--)
	{
		for(i=0;i<3195;i++);
	}
}


