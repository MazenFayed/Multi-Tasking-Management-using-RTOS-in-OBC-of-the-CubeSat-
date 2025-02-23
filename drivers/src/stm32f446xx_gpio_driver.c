/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Dec 31, 2020
 *      Author: Cubesat58
 */


#include "stm32f446xx_gpio_driver.h"


/*********************************************************************************
 * 					Implementing Generic functios for SPI
 *********************************************************************************
 */

/*********************************************************************************
 * @Fun						-GPIO_ButtonInit
 *
 * @Brief					-this function makes you uses push button on the board
 *
 * @Prime[in]				- none
 *
 * @return					- none
 *
 * @note					-none
 */
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GPIOBtn);

}


/*********************************************************************************
 * @Fun						-GPIO_ButtonInit
 *
 * @Brief					-this function makes you uses push button on the board
 *
 * @Prime[in]				- none
 *
 * @return					- none
 *
 * @note					-none
 */
void GPIOLED_Init(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_Pinconfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,Enable);

	GPIO_Init(&GpioLed);

}

/*********************************************************************************
 * @Fun						-GPIO_PeriClockControl
 *
 * @Brief					-this function enables or disables peripheral clock for the given GPIO Port
 *
 * @Prime[in]				-Base address of the GPIO peripheral
 *
 * @Prime[in]				-EN or DI in macros definition
 *
 * @return					- none
 *
 * @note					-none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi==Enable)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN()	;
		}else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN()	;
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN()	;
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN()	;
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN()	;
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN()	;
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN()	;
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN()	;
		}

	}

	else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DI()	;
		}else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DI()	;
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DI()	;
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DI()	;
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DI()	;
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_DI()	;
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_DI()	;
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DI()	;
		}

	}

}
/*
 * InIt and DeInit
 */


/*********************************************************************************
 * @Fun						-GPIO_Init
 *
 * @Brief					- this function Initializes GPIO peripheral
 *
 * @Prime[in]				- Base address of the GPIO peripheral
 * @Prime[in]				-
 *
 * @return					- none
 *
 * @note					-none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp =0 ;

	// Enable peripheral clock
		GPIO_PeriClockControl(pGPIOHandle->pGPIOx,Enable);

	//1- configure the mode of GPIO pin

	if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode<=GPIO_MODE_ANALOG)
	{

		temp =(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &=~(0x3<<(2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)); // clearing
		pGPIOHandle->pGPIOx->MODER|=temp;  // |= is used to only affect the required pin number put not affecting other pins.

	}
	else
	{
		// this part will Code later (interrupt mode)
		if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode ==GPIO_MODE_IT_FT )
				{
					//1. configure the FTSR
					EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
					//Clear the corresponding RTSR bit
					EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);

				}else if (pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode ==GPIO_MODE_IT_RT )
				{
					//1 . configure the RTSR
					EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
					//Clear the corresponding RTSR bit
					EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);

				}else if (pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
				{
					//1. configure both FTSR and RTSR
					EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
					//Clear the corresponding RTSR bit
					EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
				}

				//2. configure the GPIO port selection in SYSCFG_EXTICR
				uint8_t temp1 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber / 4 ;
				uint8_t temp2 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber % 4;
				uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
				SYSCFG_PCLK_EN();
				SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

				//3 . enable the exti interrupt delivery using IMR
				EXTI->IMR |= 1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber;
	}

			//2-configure the speed
				temp =0 ;
				temp =(pGPIOHandle->GPIO_Pinconfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
				pGPIOHandle->pGPIOx->OSPEEDER &=~(0x3<<(2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)); // clearing
				pGPIOHandle->pGPIOx->OSPEEDER|=temp;
			//3-configure PUPD settings

			temp =0 ;
			temp =(pGPIOHandle->GPIO_Pinconfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->PUPDR &=~(0x3<<(2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)); // clearing
			pGPIOHandle->pGPIOx->PUPDR|=temp;
	//4-configure the output type
			temp =0 ;
			temp =(pGPIOHandle->GPIO_Pinconfig.GPIO_PinOPType<<(pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->OTYPER &=~(0x1<<pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber); // clearing
			pGPIOHandle->pGPIOx->OTYPER|=temp;
	//5-configure alternate	functionality
			if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode==GPIO_MODE_AlTFN)
			{
				//configure the alt function registers.
				uint8_t temp1, temp2;
				temp1=pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber/8;
				temp2=pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber%8;
				pGPIOHandle->pGPIOx->AFR[temp1]&=~(0xF<<(4*temp2));  // clearing
				pGPIOHandle->pGPIOx->AFR[temp1]|=(pGPIOHandle->GPIO_Pinconfig.GPIO_PinAltFunMode<<(4*temp2));
			}
}
/*********************************************************************************
 * @Fun						-GPIO_DeInit
 *
 * @Brief					- this function DeInitializes GPIO peripheral
 *
 * @Prime[in]				- Base address of the GPIO peripheral
 * @Prime[in]				-
 *
 * @return					- none
 *
 * @note					-none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
			if(pGPIOx==GPIOA)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if(pGPIOx==GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if(pGPIOx==GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if(pGPIOx==GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if(pGPIOx==GPIOF)
			{
				GPIOF_REG_RESET();
			}
			else if(pGPIOx==GPIOG)
			{
				GPIOG_REG_RESET();
			}
			else if(pGPIOx==GPIOH)
			{
				GPIOH_REG_RESET();
			}
}
/*
 * Data Reading and Writing
 */

/*********************************************************************************
 * @Fun						-GPIO_ReadFromInputPin
 *
 * @Brief					- this function Read GPIO peripheral pin state
 *
 * @Prime[in]				- Base address of the GPIO peripheral
 * @Prime[in]				- pin number
 *
 * @return					- uint8_t
 *
 * @note					-none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PIN_Numper)
{
	uint8_t value;
	value =(uint8_t)((pGPIOx->IDR>>PIN_Numper) & 0x00000001 );

	return value ;
}

/*********************************************************************************
 * @Fun						-GPIO_ReadFromInputPort
 *
 * @Brief					- this function read from input port(content) GPIO peripheral
 *
 * @Prime[in]				- Base address of the GPIO peripheral
 * @Prime[in]				-
 *
 * @return					- uint16_t
 *
 * @note					-none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value ;
	value =(uint16_t)pGPIOx->IDR ;
	return value;

}
/*********************************************************************************
 * @Fun						-GPIO_WriteToOutputPin
 *
 * @Brief					- this function writes onto GPIO peripheral pin
 *
 * @Prime[in]				- Base address of the GPIO peripheral
 * @Prime[in]				-pin number
 * @Prime[in]				- pin state
 *
 * @return					- none
 *
 * @note					-none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PIN_Numper,uint8_t value)
{
	if(value==GPIO_PIN_SET)
	{
		//write 1 to the output data regester to the bit field corresponding to the pin numer
		pGPIOx->ODR|=(1<<PIN_Numper);
	}
	else
	{
		//write 0
		pGPIOx->ODR &=~(1<<PIN_Numper);
	}
}
/*********************************************************************************
 * @Fun						-GPIO_WriteToOutputPort
 *
 * @Brief					- this function writes onto GPIO peripheral (value)
 *
 * @Prime[in]				- Base address of the GPIO peripheral
 * @Prime[in]				-value
 * @Prime[in]				-
 *
 * @return					- none
 *
 * @note					-none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
	pGPIOx->ODR=value;

}
/*********************************************************************************
 * @Fun						-GPIO_ToggleOutputPin
 *
 * @Brief					- this function switches state on GPIO pin
 *
 * @Prime[in]				- Base address of the GPIO peripheral
 * @Prime[in]				-pin number
 * @Prime[in]				-
 *
 * @return					- none
 *
 * @note					-none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PIN_Numper)
{
	pGPIOx->ODR^=(1<<PIN_Numper);
}

/*
 * IRQ configuration and  ISR Handling
 */

/*********************************************************************************
 * @Fun						-GPIO_IRQConfiq
 *
 * @Brief					- this function Enables interrupts to some GPIO peripherals
 *
 * @Prime[in]				- IRQNumber
 * @Prime[in]				-IRQ_periority
 * @Prime[in]				- EnorDi
 *
 * @return					- none
 *
 * @note					-none
 */
void GPIO_IRQInterruptConfiq(uint8_t IRQNumber,uint8_t EnorDi)// for interrupt configuration and configure IRQ pin like enabling it or setting up
{

	if(EnorDi == Enable)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
			}
		}


}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + (iprx*4) ) |=  ( IRQPriority << shift_amount );

}

/*********************************************************************************
 * @Fun						-GPIO_IRQHandling
 *
 * @Brief					- this function Handles interrupts to GPIO
 *
 * @Prime[in]				- PIN_Numper
 * @Prime[in]				-
 * @Prime[in]				-
 *
 * @return					- none
 *
 * @note					-none
 */
void GPIO_IRQHandling(uint8_t PIN_Numper)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PIN_Numper))
	{
		//clear
		EXTI->PR |= ( 1 << PIN_Numper);
	}
}


void GPIO_PinAFConfig(GPIO_RegDef_t* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
{
  uint32_t temp = 0x00;
  uint32_t temp_2 = 0x00;


  temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4)) ;
  GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4)) ;
  temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
  GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;
}
