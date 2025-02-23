#include "stm32f446xx_driver.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "TJ_MPU6050.h"
void i2c1_init(void);
void i2c_read (char saddr , char maddr , char *data) ;
void USART2_INIT (void);
GPIO_RegDef_t *pGPIO_USART=GPIOA;
I2C_Handle_t I2C1Handle;
GPIO_RegDef_t *I2CPins =GPIOB;
USART_Handle_t usart2_handle;

uint8_t accel[4] ;
uint8_t Gyro[4];
#define MY_ADDR 0x61;
#define SLAVE_ADDR  0x68
uint8_t rcv_buf[32]="mazen";
RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;

int main(void)
{
	/* USER CODE BEGIN 1 */
		MPU_ConfigTypeDef myMpuConfig;
	  /* USER CODE END 1 */
		//1. Initialise the MPU6050 module and I2C
	MPU6050_Init(&I2C1Handle);
	//2. Configure Accel and Gyro parameters
		myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
		myMpuConfig.ClockSource = Internal_8MHz;
		myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
		myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
		myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
		MPU6050_Config(&myMpuConfig);
	USART2_INIT ();
	i2c1_init();
	//I2C_MasterReceiveData(&I2C1Handle,rcv_buf,32,SLAVE_ADDR,I2C_DISABLE_SR);
	//USART_SendData(&usart2_handle,(uint8_t*)rcv_buf,32);
	 while (1)
	  {
	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
	//		//Raw data
	//		MPU6050_Get_Accel_RawData(&myAccelRaw);
	//		MPU6050_Get_Gyro_RawData(&myGyroRaw);

		 //raw data
		 /*/
		 RawData_Def AccelRData;
		 	MPU6050_Get_Accel_RawData(&AccelRData);
		 	accel=AccelRData.x +AccelRData.y + AccelRData.z;
			USART_SendData(&usart2_handle,(uint8_t*)accel,32);/*/
			//Scaled data
			MPU6050_Get_Accel_Scale(&myAccelScaled);
			accel[0]=myAccelScaled.x;
			accel[1]=myAccelScaled.y;
			accel[2]=myAccelScaled.z;


			USART_SendData(&usart2_handle,(uint8_t*)accel,32);

			MPU6050_Get_Gyro_Scale(&myGyroScaled);
			Gyro[0]=myGyroScaled.x;
			Gyro[1]=myGyroScaled.y;
			Gyro[2]=myGyroScaled.z;
			USART_SendData(&usart2_handle,(uint8_t*)Gyro,32);


			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
			delay_ms(10);
	  }
}

void i2c1_init (void)
	{
		//scl pb8
		//sda pb9
		RCC->AHB1ENR |=2;
		RCC->APB1ENR |=0x00200000;

		I2CPins->MODER &= ~0x000F0000;
		I2CPins->MODER |=  0x000A0000;

		I2CPins->AFR[1] &= ~0x000000FF;
		I2CPins->AFR[1] |=  0x00000044;

		I2CPins->OTYPER |= 0x00000300;

		I2CPins->PUPDR  &= ~0x000F0000;
		I2CPins->PUPDR  |=  0x00050000;

		I2C1Handle.pI2Cx =I2C1 ;
		I2C1Handle.pI2Cx->CR1  =   0x8000;
		I2C1Handle.pI2Cx->CR1  &= ~0x8000;
		I2C1Handle.pI2Cx->CR2  =   0x0010 ;
		I2C1Handle.pI2Cx->CCR  =    80 ;
		I2C1Handle.pI2Cx->TRISE =    17;
		I2C1Handle.pI2Cx->CR1 |=    0x0001 ;
		I2C1Handle.I2C_Config.I2C_SCLSpeed = 100000;
		I2C1Handle.I2C_Config.I2C_FMDutyCycle =I2C_FM_DUTY_2 ;
		I2C1Handle.pI2Cx->OAR1 |=0 ;
		 /* USER CODE END I2C1_Init 1 */
		/*/
		  hi2c1.Instance = I2C1;
		  hi2c1.Init.ClockSpeed = 100000;
		  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
		  hi2c1.Init.OwnAddress1 = 0;
		  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		  hi2c1.Init.OwnAddress2 = 0;
		  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
/*/

	}
/*/void i2c_read (char saddr , char maddr , char *data)
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



}/*/
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
