/*
 * usart_rx.c
 *
 *  Created on: Apr 18, 2022
 *      Author: comp 59
 */

#include "stm32f446xx_driver.h"
#include <stdint.h>
#include <stdio.h>


USART_Handle_t usart2_handle1;

GPIO_RegDef_t *pGPIOAA = GPIOA ;
void delay_ms (int delay );
void USART2_INIT (void);
void led_play(int value);
char USART_READ (void);
char msg[1024] = "UART Tx testing YA MAZEN...\n\r";


int main(void)
{
	RCC->AHB1ENR |=1;
	pGPIOAA->MODER |= 0x400;

	USART2_INIT();
	char ch;

	while(1)
	{
      ch=USART_READ();
      led_play(ch);

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
pGPIOAA->AFR[0] =0x7000 ;
pGPIOAA->MODER |=0x0080 ;
	usart2_handle1.pUSARTx->BRR =0x0683 ;  //9600 @16mhz

	usart2_handle1.pUSARTx->CR1 =0x0004 ; //enable rx

	usart2_handle1.pUSARTx->CR1 |= 0x2000 ;

}


char USART_READ ()
{
	//wait while Rx buffer is empty
	while ( !(usart2_handle1.pUSARTx->SR & 0x0020)){}

	return (usart2_handle1.pUSARTx->DR) ;
}
void led_play(int value)
{
	value %=16;
	for(;value>0;value--)
	{
		pGPIOAA->BSRR =0x20;
		delay_ms(100);
		pGPIOAA->BSRR =0x00200000;
		delay_ms(100);
	}
	delay_ms(100);
}

void delay_ms (int delay )
{
	int i,j ;
	for (j=delay; j>0 ; j--)
	{
		for(i=0;i<3195;i++);
	}
}


