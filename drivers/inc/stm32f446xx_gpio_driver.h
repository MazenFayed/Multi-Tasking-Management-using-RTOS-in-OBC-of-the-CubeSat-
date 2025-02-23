/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Dec 31, 2020
 *      Author: Cubesat58
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_


#include "stm32f446xx_driver.h"


typedef struct
{
	uint8_t GPIO_PinNumber;			/*possible values from @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;			/*possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/*possible values from @GPIO_SPEED_MODES */
	uint8_t GPIO_PinPuPdControl;	/*possible values from @GPIO_PULLUP_PULLDOWN_MODES */
	uint8_t GPIO_PinOPType;			/*possible values from @GPIO_OUTPUT_TYPe_MODE */
	uint8_t GPIO_PinAltFunMode;		/*@GPIO_PIN_MODES*/
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;  /*Holds the base address of the GPIO Port to which the Pin belongs */
	GPIO_PinConfig_t GPIO_Pinconfig ; /*this holds pin configuration settings*/

}GPIO_Handle_t;

/*********************************************************************************
 * 					Implementing Generic functios for SPI
 *********************************************************************************
 */
void GPIO_ButtonInit(void);
void GPIOLED_Init(void);



/*/
 * @GPIO_PIN_MODES
 * From mode register in addition to 3 additional mode for interrupts
 */
#define GPIO_MOD_INPUT		0
#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_AlTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		//i/p falling edge
#define GPIO_MODE_IT_RT		5		//i/p Rising edge
#define GPIO_MODE_IT_RFT	6		//i/p falling and rising edge

/*
 * @GPIO_OUTPUT_TYPe_MODE
 * GPIO port output type register
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1
/*
 * @GPIO_SPEED_MODES
 * GPIO port output speed register
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3
/*
 * @GPIO_PULLUP_PULLDOWN_MODES
 * GPIO port pull-up/pull-down register
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2
/*
 *@GPIO_PIN_NUMBER
 *Pin numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/**************************************************************************************************
 *  								APIs supported by this Driver
 ***************************************************************************************************/
/*
 * peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
 * Data Reading and Writing
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PIN_Numper);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PIN_Numper,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PIN_Numper);

/*
 * IRQ configuration and  ISR Handling
 */
void GPIO_IRQInterruptConfiq(uint8_t IRQNumber,uint8_t EnorDi);  // for interrupt configuration and configure IRQ pin like enabling it or setting up
void GPIO_IRQPriorityConfiq(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PIN_Numper);

//alf configuration

void GPIO_PinAFConfig(GPIO_RegDef_t* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF);



#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
