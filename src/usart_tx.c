/*
 * usart_tx.c
 *
 *  Created on: Apr 14, 2022
 *      Author: comp 59
 */


#include "stm32f446xx_driver.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

void USART_Init_CONF( );

USART_Handle_t *pUSART_3 ;
char user_msg[200]="maaaaaaaaaaaaaazen ...\n\r";

GPIO_RegDef_t *pGPIOA12 = GPIOA ;
void USART_WRITE (int ch) ;


char *user_data = "The application is running\r\n";
USART_Handle_t usart2_handle;

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode =USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}
void 	USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_AlTFN;
	usart_gpios.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_Pinconfig.GPIO_PinAltFunMode =7;

	//USART2 TX
	usart_gpios.GPIO_Pinconfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	//USART2 RX
	usart_gpios.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);


}


int main (void)
{
	USART_Init_CONF( );
	USART2_GPIOInit();

       while(1)
       {


                printf("hello");
    	   		USART_WRITE('H');
    	   		USART_WRITE('I');
    			USART_SendData(&pUSART_3,(uint8_t*)user_msg,strlen(user_msg));




       }


}
void USART_Init_CONF()
 {
	RCC->AHB1ENR |=1;
	RCC->APB1ENR |=0x20000;

	pUSART_3->pUSARTx= USART2 ;
		//set pa2 to alf type uart_tx and rx(af07)
		pGPIOA12->AFR[0] =0x0700 ;
		pGPIOA12->MODER |=0x0020 ;
		pUSART_3->pUSARTx->BRR =0x0683 ;  //9600 @16mhz

		pUSART_3->pUSARTx->CR1 =0x0008 ; //enable tx and rx

		pUSART_3->pUSARTx->CR1 |= 0x2000 ;

 }
void USART_WRITE (int ch)
{
	//wait while tx buffer is empty
	while ( !(pUSART_3->pUSARTx->SR & 0x0080)){}
	pUSART_3->pUSARTx->DR =(ch & 0xFF) ;

}
