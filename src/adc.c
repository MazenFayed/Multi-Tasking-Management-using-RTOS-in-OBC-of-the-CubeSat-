/*

 * adc.c
 *
 *  Created on: Apr 15, 2022
 *      Author: MAZEN FAYED
 */

#include "stm32f446xx_driver.h"
#include <stdint.h>

static void uart_set_baudrate (	USART_RegDef_t *USARTx ,uint32_t periphclk ,uint32_t BaudRate  );
static uint16_t compute_uart_div (uint32_t periphclk , uint32_t BaudRate);
void uar2_tx_init(void);


#define GPIOAEN     (1U<<0)
#define UART2EN      (1U<<17)
#define CR1_TE        (1U<<3)
#define CR1_UE        (1U<<13)

#define SYS_FREQ      16000000
#define APB1_CLK        SYS_FREQ
#define  UART_BAUDRATE   115200

GPIO_RegDef_t *pGPIOA10 = GPIOA ;
USART_RegDef_t *pUSART2 = USART2 ;


int main (void)
{
	uint32_t sensor_value ;

	//uar2_tx_init();
	pa1_adc1_ch1_init();
	start_conversion();
	while(1)
	{
		sensor_value=adc1_read();

	}
}
void uar2_tx_init(void)
{
	GPIOA_PCLK_EN()	;                                         //ENABLE GPIOA CLOCK

	//set pa2 to alf
	pGPIOA10->MODER &=~ (1U<<4) ;
	pGPIOA10->MODER |=  (1U<<5) ;

	//set pa2 to alf type uart_tx (af07)
	pGPIOA10->AFR[0] |= (1U<<8);
	pGPIOA10->AFR[0] |= (1U<<9);
	pGPIOA10->AFR[0] |= (1U<<10);
	pGPIOA10->AFR[0] &=~  (1U<<11);

	//enable clk to uart2
	USART2_PCLK_EN();

	//CONFIGURE BAUDRATE
	uart_set_baudrate (pUSART2 ,APB1_CLK ,UART_BAUDRATE);

	//CONFIGURE USART TRANSMISSION

	pUSART2->CR1 =CR1_TE ;


	//ENABLE UART MODULE
	pUSART2->CR1 =CR1_UE ;


}

static void uart_set_baudrate (	USART_RegDef_t *USARTx ,uint32_t periphclk ,uint32_t BaudRate  )
{
	USARTx->BRR = compute_uart_div(periphclk , BaudRate);
}

static uint16_t compute_uart_div (uint32_t periphclk , uint32_t BaudRate)
{
	return((periphclk + (BaudRate/2U))/BaudRate) ;
}
