/*
 * RTOS.c
 *
 *  Created on: Apr 16, 2022
 *      Author: comp 59
 */


#include "stm32f446xx_driver.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "TJ_MPU6050.h"
#include "stdlib.h"
#include <stdlib.h>

#define AVAILABLE  1
#define NOTAVAILABLE  0
void gps_neww();
void GPS_Start();
void USART1_Init_CONF( );
static void task2_handler(void* parameters);
static void task1_handler(void* parameters);
void USART2_INIT (void);
//void i2c1_init(void);
//void i2c_read (char saddr , char maddr , char *data) ;


uint8_t buff[255];
char buffStr[255];
char nmeaSnt[80];
char GPS_MSG[80];
char *latRaw;
char latDg[2];
char latMS[7];
char *hemNS;
// longitude in degrees (0° at the Prime Meridian to +180° eastward and −180° westward)
// that is why 3
char *lonRaw;
char lonDg[3];
char lonMS[8];//char lonMS[7];
char *hemEW;
uint8_t MSG1[43] ;
uint8_t Telemetry[81] ;
uint8_t flag ;
char *utcRaw; // raw UTC time from the NMEA sentence in the hhmmss format
char strUTC[8]; // UTC time in the readable hh:mm:ss format
char hH[2]; // hours
char mM[2]; // minutes
char sS[2]; // seconds
uint8_t cnt;
volatile unsigned char Gpsdata;             // for incoming serial data
unsigned int finish =0;            // indicate end of message
unsigned int pos_cnt=0;            // position counter
unsigned int lat_cnt=0;            // latitude data counter
unsigned int log_cnt=0;            // longitude data counter
unsigned int flg    =0;            // GPS flag
unsigned int com_cnt=0;            // comma counter
unsigned char lat[20];             // latitude array
unsigned char lg[20];              // longitude array
int i=0;
unsigned char dir,dir1;


//Global variables
char user_msg[200]="maaaaaaaaaaaaaazen ...\n\r";
uint8_t sensor[4];
uint8_t UART_ACCESS_KEY =AVAILABLE;
GPIO_RegDef_t *pGPIO_USART=GPIOA;
GPIO_RegDef_t *I2CPins =GPIOB;

USART_Handle_t usart1_handle;
USART_Handle_t usart2_handle;
I2C_Handle_t I2C1Handle;
RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;
uint8_t send_x[4] ;
uint8_t send_y[4] ;
uint8_t send_z[4] ;



uint32_t ADC1_DATA ;
uint32_t ADC2_DATA ;
char msg[1024] = "UART Tx testing task1:\n\r";
char msg3[1024] = "UART Tx testing task2:\n\r";
char ACCELL[1024]="THIS IS THE ACCEL READ:\n\r";
char GYRO[1024]="THIS IS THE GYRO READ:\n\r";

char brak[1024] = "\n\r";

char msg1[1024] = "THIS IS THE ADC1 READ:\r\n";
char adc2[1024] = "THIS IS THE ADC2 READ:\r\n";
char gps[1024] = "THIS IS THE GPS READ:\r\n";


uint8_t rcv_buf[32];
uint8_t len ;
uint8_t commandcode;
char data[1024]="%d" ;
uint8_t rcv_buf[32]="mazen";
char buffer[8];



#define MY_ADDR 0x61;
#define SLAVE_ADDR  0x68


TaskHandle_t task1_handle=NULL;
TaskHandle_t task2_handle=NULL;
int main(void)
{


	USART1_Init_CONF();
	USART2_INIT ();
	pa1_adc1_ch1_init() ;                                  //ADC1
	pa4_adc2_ch4_init()  ;                               //ADC2
	start_conversion();

	 tim2_pa5_OPM_output_compare_PWM_conf();
	 tim2_pa5_PWM( 20);


	//i2c1_init();


	DWT->CTRL|=(1<<0); //Enable CYCCNT in DWT_CTRL Ø¨Ø¨ÙŠØ¹Ø¯ Ø§Ù„Ø§ÙŠÙ�Ù†ØªØ³ Ø§Ù„Ù„ÙŠ Ø®Ù„ØµØª
	//1-Resets the RCC clock configuration to the default reset state.
	//2-HSI ON and used as system clock source
	//3- HSE, PLL and PLLI2S OFF
	RCC_DeInit();
	//2-Update SystemCoreClock variable

	sprintf(user_msg,"This is the Hello World example code \r\n")	;

	//3- Creating task1 and task2
	xTaskCreate(task1_handler, "Task-1", configMINIMAL_STACK_SIZE, NULL, 2, &task1_handle);
	xTaskCreate(task2_handler, "Task-2", configMINIMAL_STACK_SIZE, NULL, 2, &task2_handle);



	//4-start the freeRTOS scheduler this is like round robin as both of the m hase the same periority
	vTaskStartScheduler();
}

// Task handlers should never return
static void task1_handler(void* parameters)
{


	while(1)
	{
		if(UART_ACCESS_KEY==AVAILABLE)
		{
			UART_ACCESS_KEY=NOTAVAILABLE;

		//USART_ReceiveData(&usart2_handle,(uint8_t*)user_msg,strlen(user_msg));

			USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));
			USART_SendData(&usart2_handle,(uint8_t*)brak,strlen(brak));


			ADC1_DATA=adc1_read();

			itoa(ADC1_DATA, buffer, 10);
		    USART_SendData(&usart2_handle,(uint8_t*)msg1,strlen(msg1));
			USART_SendData(&usart2_handle,(uint8_t*)buffer,sizeof(buffer));
			USART_SendData(&usart2_handle,(uint8_t*)brak,strlen(brak));

			MPU6050_Get_Accel_Scale(&myAccelScaled);

						USART_SendData(&usart2_handle,(uint8_t*)ACCELL,strlen(ACCELL));
						itoa(myAccelScaled.x, buffer, 10);
						USART_SendData(&usart2_handle,(uint8_t*)buffer,sizeof(buffer));
						USART_SendData(&usart2_handle,(uint8_t*)brak,strlen(brak));
						itoa(myAccelScaled.y, buffer, 10);
						USART_SendData(&usart2_handle,(uint8_t*)buffer,sizeof(buffer));
						USART_SendData(&usart2_handle,(uint8_t*)brak,strlen(brak));
						itoa(myAccelScaled.z, buffer, 10);
						USART_SendData(&usart2_handle,(uint8_t*)buffer,sizeof(buffer));
						USART_SendData(&usart2_handle,(uint8_t*)brak,strlen(brak));

						MPU6050_Get_Gyro_Scale(&myGyroScaled);


						USART_SendData(&usart2_handle,(uint8_t*)GYRO,strlen(GYRO));
						itoa(myGyroScaled.x, buffer, 10);
						USART_SendData(&usart2_handle,(uint8_t*)buffer,sizeof(buffer));
						USART_SendData(&usart2_handle,(uint8_t*)brak,strlen(brak));
						itoa(myGyroScaled.y, buffer, 10);
						USART_SendData(&usart2_handle,(uint8_t*)buffer,sizeof(buffer));
						USART_SendData(&usart2_handle,(uint8_t*)brak,strlen(brak));
						itoa(myGyroScaled.z, buffer, 10);
						USART_SendData(&usart2_handle,(uint8_t*)buffer,sizeof(buffer));
						USART_SendData(&usart2_handle,(uint8_t*)brak,strlen(brak));



						//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);
						GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
						delay_ms(10);

			UART_ACCESS_KEY=AVAILABLE;
			taskYIELD();
		}

	}

}


static void task2_handler(void* parameters)
{
	while(1)
	{

		if(UART_ACCESS_KEY==AVAILABLE)
		{
			UART_ACCESS_KEY=NOTAVAILABLE;
			USART_SendData(&usart2_handle,(uint8_t*)msg3,strlen(msg3));

		USART_ReceiveData(&usart1_handle,(uint8_t*)nmeaSnt,strlen(nmeaSnt));
	//	user_msg+="\r\n";
		   USART_SendData(&usart2_handle,(uint8_t*)gps,strlen(gps));
			USART_SendData(&usart2_handle,(uint8_t*)nmeaSnt,strlen(nmeaSnt));

		  // GPS_Start();
		   // gps_neww();

			USART_SendData(&usart2_handle,(uint8_t*)brak,strlen(brak));

			ADC2_DATA=adc1_read();
			itoa(ADC2_DATA, buffer, 10);
			USART_SendData(&usart2_handle,(uint8_t*)adc2,strlen(adc2));

			USART_SendData(&usart2_handle,(uint8_t*)buffer,sizeof(buffer));
			USART_SendData(&usart2_handle,(uint8_t*)brak,strlen(brak));


			//I2C_MasterReceiveData(&I2C1Handle,rcv_buf,32,SLAVE_ADDR,I2C_DISABLE_SR);
			//USART_SendData(&usart2_handle,(uint8_t*)rcv_buf,32);

			UART_ACCESS_KEY=AVAILABLE;
			taskYIELD();

		}
	}
}
void USART1_Init_CONF( )
 {
	 RCC->AHB1ENR |=1;
	 RCC->APB2ENR |=0x00010;

	 usart1_handle.pUSARTx = USART1;

	 	//set pa9 & pa10 to alf type uart_tx and rx(af07)
	 pGPIO_USART->AFR[1] |=0x0770 ;
	 pGPIO_USART->MODER |=0x280000 ; //pa9 tx  pa10 rx
	 usart1_handle.pUSARTx->BRR =0x0683 ;  //9600 @16mhz

	 usart1_handle.pUSARTx->CR1 =0x000C ; //enable tx and rx

	 usart1_handle.pUSARTx->CR1 |= 0x2000 ;

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

void GPS_Start()
{

			 memset(buffStr, 0, 255);
			// sprintf(buffStr, "%s", buff);
				USART_SendData(&usart2_handle,(uint8_t*)buffStr,sizeof(buffStr));

			 char *token, *string;
			 string = strdup(buffStr);
			 while ((token = strsep(&string, "\n")) != NULL)
			 {
				 memset(nmeaSnt, 0,71);
				 memset(GPS_MSG, 0,71);
				 //sprintf(nmeaSnt, "%s", token);
					USART_SendData(&usart2_handle,(uint8_t*)nmeaSnt,sizeof(nmeaSnt));

				//HAL_UART_Transmit(&uartHandler2, (uint8_t*)buffStr,(uint16_t)sizeof(buffStr), 255);
					USART_SendData(&usart2_handle,(uint8_t*)buffStr,sizeof(buffStr));

				if ((strstr(nmeaSnt, "GPGGA") != 0))//GPGGA
				{
					//strcpy(GPS_MSG,nmeaSnt);
					cnt = 0;  // splitting the good NMEA sentence into the tokens by the comma delimiter
					//for (char *pV = strtok(nmeaSnt, ","); pV != NULL; pV = strtok(NULL, ","))
					char *pV = strtok(nmeaSnt, ",");
					while(pV !=NULL)
					{
					 switch (cnt)

					 {
						case 1://1
							utcRaw = strdup(pV); //9
							break;
						case 2://2
							latRaw = strdup(pV);
							break;
						case 3://3
							hemNS = strdup(pV);
							break;
						case 4://4
							lonRaw = strdup(pV);
							break;
						case 5://5
							hemEW = strdup(pV);
							break;
					 }
					 cnt++;
					 pV = strtok(NULL, ",");
					}
				 memcpy(latDg, &latRaw[0], 2);
				 latDg[2] = '\0';
				 memcpy(latMS, &latRaw[2], 7);
				 latMS[7] = '\0';
				 memcpy(lonDg, &lonRaw[0], 3);
				 lonDg[3] = '\0';
				 memcpy(lonMS, &lonRaw[3], 7);
				 lonMS[7] = '\0';
				 char strLonMS[7];
				 sprintf(strLonMS, "%s", lonMS);
				 //converting the UTC time in the hh:mm:ss format
				 memcpy(hH, &utcRaw[0], 2);
				 hH[2] = '\0';
				 memcpy(mM, &utcRaw[2], 2);
				 mM[2] = '\0';
				 memcpy(sS, &utcRaw[4], 2);
				 sS[2] = '\0';
				 strcpy(strUTC, hH);
				 strcat(strUTC, ":");
				 strcat(strUTC, mM);
				 strcat(strUTC, ":");
				 strcat(strUTC, sS);
				 strUTC[8] = '\0';
			 }



		  }
			USART_SendData(&usart2_handle,(uint8_t*)Telemetry,sizeof(Telemetry));

		  //HAL_UART_Transmit(&uartHandler2, (uint8_t*) MSG1,sizeof(MSG1),HAL_MAX_DELAY);


		  }

void gps_neww()
{

	Gpsdata=nmeaSnt;
	flg = 1;

	if(finish == 0){
	if( Gpsdata=='$' && pos_cnt == 0)  // finding GPRMC header
	         pos_cnt=1;
	       if( Gpsdata=='G' && pos_cnt == 1)
	         pos_cnt=2;
	       if( Gpsdata=='P' && pos_cnt == 2)
	         pos_cnt=3;
	       if( Gpsdata=='R' && pos_cnt == 3)
	         pos_cnt=4;
	       if( Gpsdata=='M' && pos_cnt == 4)
	         pos_cnt=5;
	       if(Gpsdata=='C'  &&  pos_cnt == 5 )
					 //data[i]=Gpsdata;i++;
	         pos_cnt=6;
	       if(pos_cnt==6    &&  Gpsdata ==','){  // count commas in message
	         com_cnt++;
	         flg=0;
	       }
					if(Gpsdata=='N'||Gpsdata=='S'){dir=Gpsdata;}
					if(Gpsdata=='E'||Gpsdata=='W'){dir1=Gpsdata;}
	       if(com_cnt==3 && flg==1){
	        lat[lat_cnt++] =  Gpsdata;        // latitude
	        flg=0;
	       }

	       if(com_cnt==5 && flg==1){
	         lg[log_cnt++] =  Gpsdata;        // longitude
	         flg=0;
	       }

	       if( Gpsdata == '*' && com_cnt >= 5 && flg == 1){

					 lat[lat_cnt]=dir;
					 lat[lat_cnt+1] ='\0';				 // end of GPRMC message
					 lg[log_cnt]  =dir1;
	         lg[log_cnt+1]  ='\0';
	         lat_cnt = 0;
	         log_cnt = 0;
	         flg     = 0;
	         finish  = 1;
	         com_cnt = 0;
						i=0;
	      }

	}
	USART_SendData(&usart2_handle,(uint8_t*)lat,sizeof(lat));
	USART_SendData(&usart2_handle,(uint8_t*)lg,sizeof(lg));

}

