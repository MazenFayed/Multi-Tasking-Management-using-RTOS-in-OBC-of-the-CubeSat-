/*
 * main.c

 *
 *  Created on: Mar 15, 2022
 *      Author: MAZEN FAYED
 */

#include "stm32f446xx_driver.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

//prototype
static void I2C_handler (void *params );
static void ADC_handler (void *params  );
 void SetupGpioAf(GPIO_RegDef_t* PORT,	GPIO_Handle_t *pGPIO ,uint8_t GPIO_PinNumber ,uint32_t AF_NO) ;
static void USART1_handler (void *params );
static void USART2_handler (void *params );
static void PWM_handler (void *params );
void USART_Init_CONF( USART_Handle_t *usart_handle , USART_RegDef_t *PORT);

/*/#ifdef USE_SEMIHOSTING
	extern void initialise_monitor_handles();
#endif/*/

#define AVAILABLE  1
#define NOTAVAILABLE  0

//to store data
uint32_t ADC1_DATA ;
uint32_t ADC2_DATA ;

//you can by all means add more messages
char *msg_USART[3] = {"hihihihihihi123", "Hello How are you ?" , "Today is Monday !"};

//reply from AMU will be stored here
char rx_buf_USART[1024] ;
uint32_t cnt_USART = 0;



//to control switching between tasks
uint8_t ACCESS_KEY =AVAILABLE;


//MASTER ADDRESS OF I2C
#define MY_ADDR 0x61
//SLAVE ADDRESS OF I2C
#define SLAVE_ADDR  0x68
//rcv buffer
uint8_t rcv_i2c_buf[32];
//LENGTH OF RECIEVED MSG OVER I2C
uint8_t len =0;




TaskHandle_t xTaskHanndle1 =NULL;
TaskHandle_t xTaskHanndle2 =NULL;
TaskHandle_t xTaskHanndle3 =NULL;
TaskHandle_t xTaskHanndle4 =NULL;
TaskHandle_t xTaskHanndle5 =NULL;


I2C_Handle_t I2C1Handle;
USART_Handle_t usart1_handle;
USART_Handle_t usart2_handle;

int main (void)
{

	GPIO_Handle_t pGPIO_USART1;
	GPIO_Handle_t pGPIO_USART2;



/*/#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
	printf("This is the Hello World example code \n");
#endif/*/

	DWT->CTRL|=(1<<0); //Enable CYCCNT in DWT_CTRL ÈÈíÚÏ ÇáÇíÝäÊÓ Çááí ÎáÕÊ

	//reset the rcc clock to default state
	// HSI ON , PLL OFF ,HSE OFF ,SYSCLK =16 MHZ ,CPUCLK =16MHZ
	RCC_DeInit();

	//Update SystemCoreClock variable
	SystemCoreClock =16000000 ;

	//make gpio alf
	SetupGpioAf(GPIOA,&pGPIO_USART1,GPIO_PIN_NO_9,AF7);            //USART1_TX
	SetupGpioAf(GPIOA,&pGPIO_USART1,GPIO_PIN_NO_10,AF7);          //USART1_RX
	SetupGpioAf(GPIOA,&pGPIO_USART2,GPIO_PIN_NO_2,AF7);          //USART2_TX
	SetupGpioAf(GPIOA,&pGPIO_USART2,GPIO_PIN_NO_3,AF7);         //USART2_RX
	void pa1_adc1_ch1_init(void) ;                                  //ADC1
	void pa4_adc2_ch4_init(void)  ;                               //ADC2
	void tim2_pa5_OPM_output_compare_PWM_conf();
    USART_PeripheralControl(USART1,AVAILABLE);
    USART_PeripheralControl(USART2,AVAILABLE);
    USART_Init_CONF( &usart1_handle , USART1);
    USART_Init_CONF( &usart2_handle , USART2);
    USART_IRQInterruptConfig(IRQ_NO_USART2,AVAILABLE);




	//i2c peripheral configuration

		I2C1Handle=I2C_Inits(I2C1,I2C_ACK_ENABLE,MY_ADDR,I2C_FM_DUTY_2,I2C_SCL_SPEED_SM);

		//enable the i2c peripheral
		I2C_PeripheralControl(I2C1,Enable);

		//ack bit is made 1 after PE=1
		I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);


	//Start Recording
		SEGGER_SYSVIEW_Conf();
	 	SEGGER_SYSVIEW_Start();

	// LET'S CREATE 2 TASKS
	xTaskCreate( I2C_handler , "I2C" , configMINIMAL_STACK_SIZE , NULL , 2 , &xTaskHanndle1) ;

	xTaskCreate( ADC_handler , "ADC" , configMINIMAL_STACK_SIZE , NULL , 2 , &xTaskHanndle2) ;

	xTaskCreate( USART1_handler , "USART1" , configMINIMAL_STACK_SIZE , NULL , 2 , &xTaskHanndle3) ;

	xTaskCreate( USART2_handler , "USART2" , configMINIMAL_STACK_SIZE , NULL , 2 , &xTaskHanndle4) ;

	xTaskCreate( PWM_handler , "PWM" , configMINIMAL_STACK_SIZE , NULL , 2 , &xTaskHanndle5) ;




	// start the scheduler
	vTaskStartScheduler();

}



static void I2C_handler (void *params )
{
	while(1)
	{
		if(ACCESS_KEY==AVAILABLE)
		{
			ACCESS_KEY=NOTAVAILABLE;
				I2C_MasterReceiveData(&I2C1Handle,rcv_i2c_buf,len,SLAVE_ADDR,I2C_DISABLE_SR);
				ACCESS_KEY=AVAILABLE;
			 taskYIELD();
		}

	}

}

static void ADC_handler (void *params  )
{
    while(1)
    {
    	if(ACCESS_KEY==AVAILABLE)
    			{
    				ACCESS_KEY=NOTAVAILABLE;
    				ADC1_DATA=adc1_read();
    				ADC2_DATA=adc2_read();
    				ACCESS_KEY=AVAILABLE;
    				taskYIELD();
    			}


    }
}

static void USART1_handler (void *params )
{
	while(1)
	{
              if(ACCESS_KEY==AVAILABLE)
    			{
    				ACCESS_KEY=NOTAVAILABLE;
    				USART_ReceiveData(&usart1_handle,(uint8_t*)rx_buf_USART,strlen(msg_USART[cnt_USART]));
    		    	USART_SendData(&usart2_handle,(uint8_t*)msg_USART[cnt_USART],strlen(msg_USART[cnt_USART]));

    				ACCESS_KEY=AVAILABLE;
    				taskYIELD();
    			}
	}
}

static void USART2_handler (void *params )
{
	while(1)
	{
           if(ACCESS_KEY==AVAILABLE)
    			{
    				ACCESS_KEY=NOTAVAILABLE;
    				USART_ReceiveData(&usart1_handle,(uint8_t*)rx_buf_USART,strlen(msg_USART[cnt_USART]));
    				USART_SendData(&usart2_handle,(uint8_t*)msg_USART[cnt_USART],strlen(msg_USART[cnt_USART]));

    				 ACCESS_KEY=AVAILABLE;
    				taskYIELD();
    			}
	}
}

static void PWM_handler (void *params )
{
	while(1)
		{
	          if(ACCESS_KEY==AVAILABLE)
	    			{
	    				ACCESS_KEY=NOTAVAILABLE;
	    				void tim2_pa5_PWM(float t) ;
	    				ACCESS_KEY=AVAILABLE;
	    				taskYIELD();
	    			}
		}
}

 void SetupGpioAf(GPIO_RegDef_t* PORT,	GPIO_Handle_t *pGPIO ,uint8_t GPIO_PinNumber ,uint32_t AF_NO)
{

	pGPIO->pGPIOx = PORT ;
	pGPIO->GPIO_Pinconfig.GPIO_PinNumber = GPIO_PinNumber ;
	pGPIO->GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_AlTFN;
	pGPIO->GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	pGPIO->GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pGPIO->GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // could by GPIO_PIN_PU  with GPIO_OP_TYPE_OD but need external high resistor
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,AF_NO);
		GPIO_Init(pGPIO);
}



 void USART_Init_CONF( USART_Handle_t *usart_handle , USART_RegDef_t *PORT)
 {
 	usart_handle->pUSARTx = PORT;
 	usart_handle->USART_Config.USART_Baud = USART_STD_BAUD_115200;
 	usart_handle->USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
 	usart_handle->USART_Config.USART_Mode = USART_MODE_TXRX;
 	usart_handle->USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
 	usart_handle->USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
 	usart_handle->USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
 	USART_Init(usart_handle);
 }
