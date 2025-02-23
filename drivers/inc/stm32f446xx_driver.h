/*

 * stm32f446xx_driver.h

 *
 *  Created on: Dec 30, 2020
 *      Author: Cubesat58
 *  Developed on : feb 8 ,2022
 *      Developer: MAZEN FAYED
 */

#ifndef INC_STM32F446XX_DRIVER_H_
#define INC_STM32F446XX_DRIVER_H_
#include <stdint.h>     // library for definitios of unit16,32 and so on
#include <stddef.h>
#define __vo volatile
#define __weak __attribute__((weak))   // that macro means that if you use this macro before any function then user can override this function


/**********************************START:Processor Specific Details **********************************/

/* GPIO_Pin_sources
  *
  */
#define GPIO_PinSource0            ((uint8_t)0x00)
#define GPIO_PinSource1            ((uint8_t)0x01)
#define GPIO_PinSource2            ((uint8_t)0x02)
#define GPIO_PinSource3            ((uint8_t)0x03)
#define GPIO_PinSource4            ((uint8_t)0x04)
#define GPIO_PinSource5            ((uint8_t)0x05)
#define GPIO_PinSource6            ((uint8_t)0x06)
#define GPIO_PinSource7            ((uint8_t)0x07)
#define GPIO_PinSource8            ((uint8_t)0x08)
#define GPIO_PinSource9            ((uint8_t)0x09)
#define GPIO_PinSource10           ((uint8_t)0x0A)
#define GPIO_PinSource11           ((uint8_t)0x0B)
#define GPIO_PinSource12           ((uint8_t)0x0C)
#define GPIO_PinSource13           ((uint8_t)0x0D)
#define GPIO_PinSource14           ((uint8_t)0x0E)
#define GPIO_PinSource15           ((uint8_t)0x0F)

/**
  * AF  selection
  */
#define AF0        ((uint8_t)0x00)  /* Alternate Function 0 mapping */
#define AF1        ((uint8_t)0x01)  /* Alternate Function 0 mapping */
#define AF2        ((uint8_t)0x02)  /* Alternate Function 0 mapping */
#define AF3        ((uint8_t)0x03)  /* Alternate Function 0 mapping */
#define AF4        ((uint8_t)0x04)  /* Alternate Function 0 mapping */
#define AF5        ((uint8_t)0x05)  /* Alternate Function 0 mapping */
#define AF6        ((uint8_t)0x06)  /* Alternate Function 0 mapping */
#define AF7        ((uint8_t)0x07)  /* Alternate Function 0 mapping */
#define AF8        ((uint8_t)0x08)  /* Alternate Function 0 mapping */
#define AF9        ((uint8_t)0x09)  /* Alternate Function 0 mapping */
#define AF10        ((uint8_t)0x0A)  /* Alternate Function 0 mapping */
#define AF11       ((uint8_t)0x0B)  /* Alternate Function 0 mapping */
#define AF12       ((uint8_t)0x0C)  /* Alternate Function 0 mapping */
#define AF13       ((uint8_t)0x0D)  /* Alternate Function 0 mapping */
#define AF14       ((uint8_t)0x0E)  /* Alternate Function 0 mapping */
#define AF15       ((uint8_t)0x0F)  /* Alternate Function 0 mapping */


/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses interrupt set register
 */

#define NVIC_ISER0   			        ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          			( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2        				( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          			( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses  interrupt clear register
 */
#define NVIC_ICER0 						((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1						((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  					((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3						((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 				((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register  in ST  may be 3 in TI
 */
#define NO_PR_BITS_IMPLEMENTED  4



#define DWT_BASE            (0xE0001000UL)                            /*!< DWT Base Address                   */


/*
 * define base addresses for peripherals
 */
#define FLASH_BASEADDR 					0x08000000U  // Main memory
#define SRAM1_BASEADDR					0x20000000U
#define SRAM2_BASEADDR					0x2001C000U   //this is calculated by adding 112Kb to SRAM1_BASADDR
#define SRAM 							SRAM1_BASEADDR
#define ROM_BASEADDR					0x1FFF0000U  //System memeory

/*
 * AHBx & APBx Bus peripherals base addresses
 */
#define PERIPH_BASE						0x40000000U
#define APB1_BASEADDR 					PERIPH_BASE
#define APB2_BASEADDR					0x40010000U
#define AHB1_BASEADDR					0x40020000U
#define AHB2_BASEADDR					0x50000000U
#define AHB3_BASEADDR					0xA0001000U
/*
 * define base addresses of peripherals hanging on AHB1 BUS
 */
#define GPIOA_BASEADDR 					(AHB1_BASEADDR+0x0000)
#define GPIOB_BASEADDR					(AHB1_BASEADDR+0x0400)
#define GPIOC_BASEADDR					(AHB1_BASEADDR+0x0800)
#define GPIOD_BASEADDR					(AHB1_BASEADDR+0x0C00)
#define GPIOE_BASEADDR					(AHB1_BASEADDR+0x1000)
#define GPIOF_BASEADDR					(AHB1_BASEADDR+0x1400)
#define GPIOG_BASEADDR					(AHB1_BASEADDR+0x1800)
#define GPIOH_BASEADDR					(AHB1_BASEADDR+0x1C00)
#define CRC_BASEADDR					(AHB1_BASEADDR+0x3000)
#define RCC_BASEADDR					(AHB1_BASEADDR+0x3800)
#define FLASH_INTERFACE_BASEADDR 		(AHB1_BASEADDR+0x3C00)
#define BKPSRAM_BASEADDR				(AHB1_BASEADDR+0x4000)
#define DMA1_BASEADDR					(AHB1_BASEADDR+0x6000)
#define DMA2_BASEADDR					(AHB1_BASEADDR+0x6400)
#define USB_OTG_HS_BASEADDR				(AHB1_BASEADDR+0x0000)

/*
 * define base addresses of peripherals hanging on AHB2 BUS
 */
#define USB_OTG_FS_BASEADDR				(AHB2_BASEADDR+0x0000)
#define DCMI_BASEADDR					(AHB2_BASEADDR+0x0000)

/*
 * define base addresses of peripherals hanging on AHB3 BUS
 */
#define QUADSPI_register_BASEADDR		(AHB3_BASEADDR+0x1000)
#define FMC_control_register_BASEADDR	(AHB3_BASEADDR+0x0000)

/*
 * define base addresses of peripherals hanging on APB1 BUS
 */
#define TIM2_BASEADDR					(APB1_BASEADDR+0x0000)
#define TIM3_BASEADDR					(APB1_BASEADDR+0x4000)
#define TIM4_BASEADDR   				(APB1_BASEADDR+0x8000)
#define TIM5_BASEADDR  					(APB1_BASEADDR+0xC000)
#define TIM6_BASEADDR   				(APB1_BASEADDR+0x1000)
#define TIM7_BASEADDR   				(APB1_BASEADDR+0x1400)
#define TIM12_BASEADDR   				(APB1_BASEADDR+0x1800)
#define TIM13_BASEADDR   				(APB1_BASEADDR+0x1C00)
#define TIM14_BASEADDR  				(APB1_BASEADDR+0x2000)

#define RTC_BKP_Register_BASEADDR 		(APB1_BASEADDR+0x2800)
#define WWDG_BASEADDR 					(APB1_BASEADDR+0x2C00)
#define IWDG_BASEADDR 					(APB1_BASEADDR+0x3000)

#define SPI2_BASEADDR					(APB1_BASEADDR+0x3800)
#define SPI3_BASEADDR					(APB1_BASEADDR+0x3C00)

#define SPDIF_RX_BASEADDR				(APB1_BASEADDR+0x4000)

#define USART2_BASEADDR					(APB1_BASEADDR+0x4400)
#define USART3_BASEADDR					(APB1_BASEADDR+0x4800)
#define UART4_BASEADDR					(APB1_BASEADDR+0x4C00)
#define UART5_BASEADDR					(APB1_BASEADDR+0x5000)

#define I2C1_BASEADDR					(APB1_BASEADDR+0x5400)
#define I2C2_BASEADDR					(APB1_BASEADDR+0x5800)
#define I2C3_BASEADDR					(APB1_BASEADDR+0x5C00)

#define CAN1_BASEADDR					(APB1_BASEADDR+0x6400)
#define CAN2_BASEADDR					(APB1_BASEADDR+0x6800)
#define HDMI_CEC_BASEADDR				(APB1_BASEADDR+0x6C00)
#define PWR_BASEADDR					(APB1_BASEADDR+0x7000)
#define DAC_BASEADDR					(APB1_BASEADDR+0x7400)



/*
 * define base addresses of peripherals hanging on APB2 BUS
 */
#define TIM1_BASEADDR					(APB2_BASEADDR+0x0000)
#define TIM8_BASEADDR					(APB2_BASEADDR+0x0400)

#define USART1_BASEADDR					(APB2_BASEADDR+0x1000)
#define USART6_BASEADDR					(APB2_BASEADDR+0x1400)

#define ADC_BASEADDR					 (APB2_BASEADDR+0x2000) // ADC1 ,ADC2 ADC3  SYSCFG
#define ADC1_BASEADDR                    (ADC_BASEADDR+0x000)
#define ADC2_BASEADDR                    (ADC_BASEADDR+0x100)
#define ADC3_BASEADDR                    (ADC_BASEADDR+0x200)
#define ADC_CR_BASEADDR                  (ADC_BASEADDR+0x300)


#define SDMMC_BASEADDR					(APB2_BASEADDR+0x2C00)

#define SPI1_BASEADDR					(APB2_BASEADDR+0x3000)
#define SPI4_BASEADDR					(APB2_BASEADDR+0x3400)
#define SYSCFG_BASEADDR					(APB2_BASEADDR+0x3800)

#define EXTI_BASEADDR					(APB2_BASEADDR+0x3C00)
#define TIM9_BASEADDR					(APB2_BASEADDR+0x4000)
#define TIM10_BASEADDR					(APB2_BASEADDR+0x4400)
#define TIM11_BASEADDR					(APB2_BASEADDR+0x4800)

#define SAI1_BASEADDR					(APB2_BASEADDR+0x5800)
#define SAI2_BASEADDR					(APB2_BASEADDR+0x5C00)


/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

/** \brief  Structure type to access the Data Watchpoint and Trace Register (DWT).
 */
typedef struct
{
	__vo uint32_t CTRL;                    /*!< Offset: 0x000 (R/W)  Control Register                          */
	__vo uint32_t CYCCNT;                  /*!< Offset: 0x004 (R/W)  Cycle Count Register                      */
	__vo uint32_t CPICNT;                  /*!< Offset: 0x008 (R/W)  CPI Count Register                        */
	__vo uint32_t EXCCNT;                  /*!< Offset: 0x00C (R/W)  Exception Overhead Count Register         */
	__vo uint32_t SLEEPCNT;                /*!< Offset: 0x010 (R/W)  Sleep Count Register                      */
	__vo uint32_t LSUCNT;                  /*!< Offset: 0x014 (R/W)  LSU Count Register                        */
	__vo uint32_t FOLDCNT;                 /*!< Offset: 0x018 (R/W)  Folded-instruction Count Register         */
	__vo  uint32_t PCSR;                    /*!< Offset: 0x01C (R/ )  Program Counter Sample Register           */
	__vo uint32_t COMP0;                   /*!< Offset: 0x020 (R/W)  Comparator Register 0                     */
	__vo uint32_t MASK0;                   /*!< Offset: 0x024 (R/W)  Mask Register 0                           */
	__vo uint32_t FUNCTION0;               /*!< Offset: 0x028 (R/W)  Function Register 0                       */
       uint32_t RESERVED0[1];
       __vo uint32_t COMP1;                   /*!< Offset: 0x030 (R/W)  Comparator Register 1                     */
       __vo uint32_t MASK1;                   /*!< Offset: 0x034 (R/W)  Mask Register 1                           */
       __vo uint32_t FUNCTION1;               /*!< Offset: 0x038 (R/W)  Function Register 1                       */
       uint32_t RESERVED1[1];
       __vo uint32_t COMP2;                   /*!< Offset: 0x040 (R/W)  Comparator Register 2                     */
       __vo uint32_t MASK2;                   /*!< Offset: 0x044 (R/W)  Mask Register 2                           */
       __vo uint32_t FUNCTION2;               /*!< Offset: 0x048 (R/W)  Function Register 2                       */
       uint32_t RESERVED2[1];
       __vo uint32_t COMP3;                   /*!< Offset: 0x050 (R/W)  Comparator Register 3                     */
       __vo uint32_t MASK3;                   /*!< Offset: 0x054 (R/W)  Mask Register 3                           */
       __vo uint32_t FUNCTION3;               /*!< Offset: 0x058 (R/W)  Function Register 3                       */
} DWT_Type;

/*
 * GPIO REGISTERS
 */
typedef struct
{
	__vo uint32_t	MODER ;			/*GPIO port mode register*/ 				/*Address offset 0x00*/
	__vo uint32_t 	OTYPER;				/*GPIO port output type register*/			/*Address offset 0x04*/
	__vo uint32_t 	OSPEEDER;			/*GPIO output Speed*/											/*Address offset 0x08*/
	__vo uint32_t	PUPDR;				/*GPIO port pull-up/pull-down register */	/*Address offset 0x0C*/
	__vo uint32_t	IDR;				/*GPIO port input data register*/			/*Address offset 0x10*/
	__vo uint32_t	ODR ;				/*GPIO port output data register */			/*Address offset 0x14*/
	__vo uint32_t	BSRR;				/*GPIO port bit set/reset register*/		/*Address offset 0x18*/
	__vo uint32_t	LCKR;				/*GPIO port configuration lock register*/	/*Address offset 0x1C*/
	__vo uint32_t	AFR[2];				/*AFR[0]:alternate function low register  AFR[1]:alternate function HIGH register*/


}GPIO_RegDef_t;



/*
 *  GENERAL PURPOSE TIMERS REGISTERS
 */

typedef struct
{
	__vo uint32_t CR1 ;                                        //TIMx control register 1
	__vo uint32_t CR2 ;                                         //TIMx control register 2
	__vo uint32_t SMCR ;                                       //TIMx slave mode control register
	__vo uint32_t DIER ;                                        // TIMx DMA/Interrupt enable register
	__vo uint32_t SR ;                                           //TIMx status register
	__vo uint32_t EGR ;                                              //TIMx event generation register
	__vo uint32_t CCMR1 ;                                                // TIMx capture/compare mode register 1
	__vo uint32_t CCMR2 ;                                                   //TIMx capture/compare mode register 2
	__vo uint32_t CCER ;                                             //TIMx capture/compare enable register
	__vo uint32_t CNT ;                                              //TIMx counter
	__vo uint32_t PSC ;                                               //TIMx prescaler
	__vo uint32_t ARR ;                                              //TIMx auto-reload register
	__vo uint32_t RESERVED1 ;
	__vo uint32_t  CCR1 ;                                              //TIMx capture/compare register 1
	__vo uint32_t  CCR2 ;                                                //TIMx capture/compare register 2
	__vo uint32_t CCR3 ;                                                //TIMx capture/compare register 3
	__vo uint32_t CCR4 ;                                               //TIMx capture/compare register 4
	__vo uint32_t RESERVED2 ;
	__vo uint32_t DCR ;                                                 //TIMx DMA control register
	__vo uint32_t DMAR ;                                                //TIMx DMA address for full transfer
	__vo uint32_t OR ;                                                 //TIM2 option register


}TIM_RegDef_t;

   // TIM_RegDef_t *pTIM2 = ((TIM_RegDef_t*) TIM2_BASEADDR)   or      TIM_RegDef_t *pTIM2 =TIM2


/*
 *  ANALOG TO DIGITAL CONVERTER REGISTERS FOR EACH ADC
 */

typedef struct
{
	__vo uint32_t ADC_SR ;                                                //ADC status register
	__vo uint32_t ADC_CR1 ;                                                //ADC control register 1
	__vo uint32_t  ADC_CR2 ;                                                //ADC control register 2
	__vo uint32_t ADC_SMPR1 ;                                               //ADC sample time register 1
	__vo uint32_t ADC_SMPR2 ;                                               //ADC sample time register 2
	__vo uint32_t ADC_JOFR1 ;                                               //ADC injected channel data offset register 1
	__vo uint32_t ADC_JOFR2 ;                                               //ADC injected channel data offset register 2
	__vo uint32_t ADC_JOFR3 ;                                                //ADC injected channel data offset register 3
	__vo uint32_t ADC_JOFR4 ;                                                  // ADC injected channel data offset register 4
	__vo uint32_t ADC_HTR ;                                                       //ADC watchdog higher threshold register
	__vo uint32_t ADC_LTR ;                                                     // ADC watchdog lower threshold register
	__vo uint32_t ADC_SQR1 ;                                                      //ADC regular sequence register 1
	__vo uint32_t ADC_SQR2 ;                                                            //ADC regular sequence register 2
	__vo uint32_t ADC_SQR3 ;                                                           //ADC regular sequence register 3
	__vo uint32_t ADC_JSQR ;                                                        //ADC injected sequence register
	__vo uint32_t ADC_JDR1 ;                                                       //ADC injected data register 1
	__vo uint32_t ADC_JDR2 ;                                                            //ADC injected data register 2
	__vo uint32_t ADC_JDR3 ;                                                           // ADC injected data register 3
	__vo uint32_t ADC_JDR4 ;                                                          // ADC injected data register 4
	__vo uint32_t ADC_DR ;                                                            //ADC regular data register



}ADC_RegDef_t;


/*
 *  ANALOG TO DIGITAL CONVERTER COMMON RGISTERS FOR ALL ADC
 */

typedef struct
{
	    __vo uint32_t ADC_CSR ;                                                          //ADC Common status register
		__vo uint32_t  ADC_CCR ;                                                         //ADC common control register
		__vo uint32_t  ADC_CDR ;                                                         //ADC common regular data register for dual and triple modes
}ADC_CM_RegDef_t;


/*
 *
 * DIGITAL TO ANALOG CONVERTER REGISTERS
 * */

typedef struct
{

	__vo uint32_t   DAC_CR;                                                          //DAC control register
	__vo uint32_t   DAC_SWTRIGR ;                                                    //DAC software trigger register
	__vo uint32_t   DAC_DHR12R1 ;                                                    //DAC channel1 12-bit right-aligned data holding register
	__vo uint32_t   DAC_DHR12L1 ;                                                     //DAC channel1 12-bit left aligned data holding register
	__vo uint32_t   DAC_DHR8R1 ;                                                      //DAC channel1 8-bit right aligned data holding register
	__vo uint32_t   DAC_DHR12R2 ;                                                     //DAC channel2 12-bit right aligned data holding register
	__vo uint32_t   DAC_DHR12L2 ;                                                        //DAC channel2 12-bit left aligned data holding register
	__vo uint32_t   DAC_DHR8R2 ;                                                       //DAC channel2 8-bit right-aligned data holding register
	__vo uint32_t   DAC_DHR12RD ;                                                      //Dual DAC 12-bit right-aligned data holding register
	__vo uint32_t   DAC_DHR12LD ;                                                       //DUAL DAC 12-bit left aligned data holding register
	__vo uint32_t   DAC_DHR8RD ;                                                       //DUAL DAC 8-bit right aligned data holding register
	__vo uint32_t   DAC_DOR1 ;                                                          //DAC channel1 data output register 1
	__vo uint32_t   DAC_DOR2 ;                                                          //DAC channel1 data output register 2
	__vo uint32_t   DAC_SR ;                                                             //DAC status register


}DAC_RegDef_t;



/*
 * RCC REGISTERS
 */
typedef struct
{
	__vo uint32_t	CR;																/*Address offset 0x00*/
	__vo uint32_t	PLL_CFGR;														/*Address offset 0x04*/
	__vo uint32_t	CFGR;															/*Address offset 0x08*/
	__vo uint32_t	CIR;															/*Address offset 0x0C*/
	__vo uint32_t	AHB1_RSTR;														/*Address offset 0x10*/
	__vo uint32_t	AHB2_RSTR;														/*Address offset 0x14*/
	__vo uint32_t	AHB3_RSTR;														/*Address offset 0x18*/
	__vo uint32_t	Reserved0;														/*Address offset 0x1C*/
	__vo uint32_t	APB1_RSTR;														/*Address offset 0x20*/
	__vo uint32_t	APB2_RSTR;														/*Address offset 0x24*/
	__vo uint32_t	Reserved1[2];													/*Address offset 0x28  0x2C*/
	__vo uint32_t	AHB1ENR;														/*Address offset 0x30*/
	__vo uint32_t	AHB2ENR;														/*Address offset 0x34*/
	__vo uint32_t	AHB3ENR;														/*Address offset 0x38*/
	__vo uint32_t	Reserved2;														/*Address offset 0x3C*/
	__vo uint32_t	APB1ENR;														/*Address offset 0x40*/
	__vo uint32_t	APB2ENR;														/*Address offset 0x44*/
	__vo uint32_t	Reserved3[2];													/*Address offset 0x48 , 0x4C*/
	__vo uint32_t	AHB1_LPENR;														/*Address offset 0x50*/
	__vo uint32_t	AHB2_LPENR;														/*Address offset 0x54*/
	__vo uint32_t	AHB3_LPENR;														/*Address offset 0x58*/
	__vo uint32_t	Reserved4;														/*Address offset 0x5C*/
	__vo uint32_t	APB1_LPENR;														/*Address offset 0x60*/
	__vo uint32_t	APB2_LPENR;														/*Address offset 0x64*/
	__vo uint32_t	Reserved5[2];													/*Address offset 0x68 , 0x6C*/
	__vo uint32_t	BDCR;															/*Address offset 0x70*/
	__vo uint32_t	CSR;															/*Address offset 0x74*/
	__vo uint32_t	Reserved6[2];													/*Address offset 0x78 , 0x7C*/
	__vo uint32_t	SS_CGR;															/*Address offset 0x80*/
	__vo uint32_t	PLLI2_SCFGR;													/*Address offset 0x84*/
	__vo uint32_t	PLL_SAI_CFGR;													/*Address offset 0x88*/
	__vo uint32_t	DCK_CFGR;														/*Address offset 0x8C*/
	__vo uint32_t	CK_GATENR;														/*Address offset 0x90*/
	__vo uint32_t	DCK_CFGR2;														/*Address offset 0x94*/


}RCC_RegDef_t;
//GPIO_RegDef_t *pGPIOA =GPIOA_BASEADDR ; /* a pointer to the base address of GPIOA; hence we can access any register from that base by just
 	 	 	 	 	 	 	 	 	 	 	 /*	typing pGPIOA->REG_NAME ==GPIOA_BASEADDR+REG_NAME*/


/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Interrupt mask register          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< Event mask register							Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!<Rising trigger selection register             Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< Falling trigger selection register			Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!<Software interrupt event register  			Address offset: 0x10 */
	__vo uint32_t PR;     /*!< Pending register                			    Address offset: 0x14 */

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;



/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;


/*
 * peripheral definition (peripheral base addresses type casted to xxx_RegDef_t
 */



#define GPIOA								((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 							   	((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 								((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 								((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 								((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 								((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC 								((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI								((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG								((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1  								((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  								((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  								((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4  								((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1  								((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  								((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  								((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  							((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  							((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  							((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  								((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  								((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  							((USART_RegDef_t*)USART6_BASEADDR)



#define TIM2                             ((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM3                              ((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM4                              ((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM5                              ((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM6                             ((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM7                              ((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM12                              ((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM13                              ((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM14                              ((TIM_RegDef_t*) TIM2_BASEADDR)



#define ADC1                                  ((ADC_RegDef_t*) ADC1_BASEADDR)
#define ADC2                                  ((ADC_RegDef_t*) ADC2_BASEADDR)
#define ADC3                                  ((ADC_RegDef_t*) ADC3_BASEADDR)
#define ADC_CR                                ((ADC_CM_RegDef_t*) ADC_CR_BASEADDR)

#define DAC                                   ((DAC_RegDef_t*) DAC_BASEADDR)


#define DWT                 ((DWT_Type       *)     DWT_BASE      )   /*!< DWT configuration struct           */


/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()						(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()						(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()						(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()						(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()						(RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()						(RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()						(RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()						(RCC->AHB1ENR |=(1<<7))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()						(RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI()						(RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DI()						(RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI()						(RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI()						(RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DI()						(RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DI()						(RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DI()						(RCC->AHB1ENR &=~(1<<7))



/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)    		  ( (x == GPIOA)?0:\
												(x == GPIOB)?1:\
												(x == GPIOC)?2:\
												(x == GPIOD)?3:\
												(x == GPIOE)?4:\
												(x == GPIOF)?5:\
												(x == GPIOG)?6:\
												(x == GPIOH)?7:0 )


/*
 * IRQ(Interrupt Request) Numbers of STM32F446xx MCU
 * NOTE: update these macros with valid values according to your MCU
 * From Vector table get position number
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4			84

#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER  	32
#define IRQ_NO_I2C2_EV      33
#define IRQ_NO_I2C2_ER  	34
#define IRQ_NO_I2C3_EV      72
#define IRQ_NO_I2C3_ER  	73

#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * Clock Enable Macros for CRC_RST peripherals
 */
#define CRC_RST_PCLK_EN()		(RCC->AHB1ENR |=(1<<12))

/*
 * Clock Disable Macros for CRC_RST peripherals
 */
#define CRC_RST_PCLK_DI()		(RCC->AHB1ENR &=~(1<<12))

/*
 * Clock Enable Macros for DMAx peripherals
 */
#define DMA1_RST_PCLK_EN()		(RCC->AHB1ENR |=(1<<21))
#define DMA2_RST_PCLK_EN()		(RCC->AHB1ENR |=(1<<22))

/*
 * Clock Disable Macros for DMAx peripherals
 */
#define DMA1_RST_PCLK_DI()		(RCC->AHB1ENR &=~(1<<21))
#define DMA2_RST_PCLK_DI()		(RCC->AHB1ENR &=~(1<<22))


/*
 * Clock Enable Macros for OTGHS peripherals
 */
#define OTGHS_RST_PCLK_EN()		(RCC->AHB1ENR |=(1<<29))

/*
 * Clock Disable Macros for OTGHS peripherals
 */
#define OTGHS_RST_PCLK_DI()		(RCC->AHB1ENR &=~(1<<29))



/*
 * Clock Enable Macros for SAIx peripherals
 */
#define SAI1_PCLK_EN()			(RCC->APB2_LPENR |=(1<<22))
#define SAI2_PCLK_EN()			(RCC->APB2_LPENR |=(1<<23))

/*
 * Clock Disable Macros for SAIx peripherals
 */
#define SAI1_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<22))
#define SAI2_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<23))


/*
 * Clock Enable Macros for SDIO peripherals
 */
#define SDIO_PCLK_EN()			(RCC->APB2_LPENR |=(1<<11))

/*
 * Clock Disable Macros for SDIO peripherals
 */
#define SDIO_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<11))


/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |=(1<<14))


/*
 * Clock Disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &=~(1<<14))


/*
 * Clock Enable Macros for TIMx peripherals
 */
#define TIM1_PCLK_EN()			(RCC->APB2_LPENR |=(1<<0))
#define TIM2_PCLK_EN()			(RCC->APB1_LPENR |=(1<<0))
#define TIM3_PCLK_EN()			(RCC->APB1_LPENR |=(1<<1))
#define TIM4_PCLK_EN()			(RCC->APB1_LPENR |=(1<<2))
#define TIM5_PCLK_EN()			(RCC->APB1_LPENR |=(1<<3))
#define TIM6_PCLK_EN()			(RCC->APB1_LPENR |=(1<<4))
#define TIM7_PCLK_EN()			(RCC->APB1_LPENR |=(1<<5))
#define TIM8_PCLK_EN()			(RCC->APB2_LPENR |=(1<<1))
#define TIM9_PCLK_EN()			(RCC->APB2_LPENR |=(1<<16))
#define TIM10_PCLK_EN()			(RCC->APB2_LPENR |=(1<<17))
#define TIM11_PCLK_EN()			(RCC->APB2_LPENR |=(1<<18))
#define TIM12_PCLK_EN()			(RCC->APB1_LPENR |=(1<<6))
#define TIM13_PCLK_EN()			(RCC->APB1_LPENR |=(1<<7))
#define TIM14_PCLK_EN()			(RCC->APB1_LPENR |=(1<<8))


/*
 * Clock Disable Macros for TIMx peripherals
 */
#define TIM1_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<0))
#define TIM2_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<0))
#define TIM3_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<1))
#define TIM4_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<2))
#define TIM5_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<3))
#define TIM6_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<4))
#define TIM7_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<5))
#define TIM8_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<1))
#define TIM9_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<16))
#define TIM10_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<17))
#define TIM11_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<18))
#define TIM12_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<6))
#define TIM13_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<7))
#define TIM14_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<8))



/*
 * Clock Enable Macros for WWDG peripherals
 */
#define WWDG_PCLK_EN()		(RCC->APB1_LPENR |=(1<<11))


/*
 * Clock Disable Macros for WWDG peripherals
 */
#define WWDG_PCLK_DI()		(RCC->APB1_LPENR &=~(1<<11))



/*
 * Clock Enable Macros for SPDIFRX peripherals
 */
#define SPDIFRX_PCLK_EN()		(RCC->APB1_LPENR |=(1<<16))


/*
 * Clock Disable Macros for SPDIFRX peripherals
 */
#define SPDIFRX_PCLK_DI()		(RCC->APB1_LPENR &=~(1<<16))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |=(1<<23))


/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()			(RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &=~(1<<23))

/*
 * Clock Enable Macros for  FMPI2C1 peripherals
 */
#define FMPI2C1_PCLK_EN()		(RCC->APB1_LPENR |=(1<<24))

/*
 * Clock Disable Macros for  FMPI2C1 peripherals
 */
#define FMPI2C1_PCLK_DI()		(RCC->APB1_LPENR &=~(1<<24))

/*
 * Clock Enable Macros for  CANx peripherals
 */
#define CAN1_PCLK_EN()			(RCC->APB1_LPENR |=(1<<25))
#define CAN2_PCLK_EN()			(RCC->APB1_LPENR |=(1<<26))


/*
 * Clock Disable Macros for  CANx peripherals
 */
#define CAN1_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<25))
#define CAN2_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<26))


/*
 * Clock Enable Macros for  DAC peripherals
 */
#define DAC_PCLK_EN()			(RCC->APB1_LPENR |=(1<<27))

/*
 * Clock Disable Macros for  DAC peripherals
 */
#define DAC_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<27))



/*
 * Clock Enable Macros for  CECLP peripherals
 */
#define CECLP_PCLK_EN()			(RCC->APB1_LPENR |=(1<<28))

/*
 * Clock Disable Macros for  CECLP peripherals
 */
#define CECLP_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<28))



/*
 * Clock Enable Macros for  PWR peripherals
 */
#define PWR_PCLK_EN()			(RCC->APB1_LPENR |=(1<<29))


/*
 * Clock Disable Macros for  PWR peripherals
 */
#define PWR_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<29))




/*
 * Clock Enable Macros for  ADC peripherals
 */
#define ADC1_PCLK_EN()			(RCC->APB2_LPENR |=(1<<8))
#define ADC2_PCLK_EN()			(RCC->APB2_LPENR |=(1<<9))
#define ADC3_PCLK_EN()			(RCC->APB2_LPENR |=(1<<10))


/*
 * Clock Disable Macros for  ADC peripherals
 */
#define ADC1_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<8))
#define ADC2_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<9))
#define ADC3_PCLK_DI()			(RCC->APB2_LPENR &=~(1<<10))




/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |=(1<<15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |=(1<<13))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &=~(1<<12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &=~(1<<14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &=~(1<<15))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &=~(1<<13))



/*
 *  Macros to reset GPIOx Register
 */
#define GPIOA_REG_RESET()		    do{(RCC->AHB1_RSTR |=(1<<0)); (RCC->AHB1_RSTR &=~(1<<0)); }while(0)
#define GPIOB_REG_RESET()			do{(RCC->AHB1_RSTR |=(1<<1)); (RCC->AHB1_RSTR &=~(1<<1)); }while(0)
#define GPIOC_REG_RESET()			do{(RCC->AHB1_RSTR |=(1<<2)); (RCC->AHB1_RSTR &=~(1<<2)); }while(0)
#define GPIOD_REG_RESET()			do{(RCC->AHB1_RSTR |=(1<<3)); (RCC->AHB1_RSTR &=~(1<<3)); }while(0)
#define GPIOE_REG_RESET()			do{(RCC->AHB1_RSTR |=(1<<4)); (RCC->AHB1_RSTR &=~(1<<4)); }while(0)
#define GPIOF_REG_RESET()			do{(RCC->AHB1_RSTR |=(1<<5)); (RCC->AHB1_RSTR &=~(1<<5)); }while(0)
#define GPIOG_REG_RESET()			do{(RCC->AHB1_RSTR |=(1<<6)); (RCC->AHB1_RSTR &=~(1<<6)); }while(0)
#define GPIOH_REG_RESET()			do{(RCC->AHB1_RSTR |=(1<<7)); (RCC->AHB1_RSTR &=~(1<<7)); }while(0)

/*
 *  Macros to reset SPIx Register
 */
#define SPI1_REG_RESET()		    do{(RCC->APB2_RSTR |=(1<<12)); (RCC->APB2_RSTR &=~(1<<12)); }while(0)
#define SPI2_REG_RESET()			do{(RCC->APB1_RSTR |=(1<<14)); (RCC->APB1_RSTR &=~(1<<14)); }while(0)
#define SPI3_REG_RESET()			do{(RCC->APB1_RSTR |=(1<<15)); (RCC->APB1_RSTR &=~(1<<15)); }while(0)
#define SPI4_REG_RESET()			do{(RCC->APB2_RSTR |=(1<<13)); (RCC->APB2_RSTR &=~(1<<13)); }while(0)



/*
 *  Macros to reset I2Cx Register
 */
#define I2C1_REG_RESET()		    do{(RCC->APB1_RSTR |=(1<<21)); (RCC->APB1_RSTR &=~(1<<21)); }while(0)
#define I2C2_REG_RESET()			do{(RCC->APB1_RSTR |=(1<<22)); (RCC->APB1_RSTR &=~(1<<22)); }while(0)
#define I2C3_REG_RESET()			do{(RCC->APB1_RSTR |=(1<<23)); (RCC->APB1_RSTR &=~(1<<23)); }while(0)


/*
 *  Macros to reset USARTx Register
 */
#define USART2_REG_RESET()		    do{(RCC->APB1_RSTR |=(1<<17)); (RCC->APB1_RSTR &=~(1<<17)); }while(0)
#define USART3_REG_RESET()			do{(RCC->APB1_RSTR |=(1<<18)); (RCC->APB1_RSTR &=~(1<<18)); }while(0)
#define USART4_REG_RESET()			do{(RCC->APB1_RSTR |=(1<<19)); (RCC->APB1_RSTR &=~(1<<19)); }while(0)
#define USART5_REG_RESET()			do{(RCC->APB1_RSTR |=(1<<20)); (RCC->APB1_RSTR &=~(1<<20)); }while(0)



/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()		(RCC->APB2_LPENR |=(1<<4))
#define USART2_PCLK_EN()		(RCC->APB1_LPENR |=(1<<17))
#define USART3_PCLK_EN()		(RCC->APB1_LPENR |=(1<<18))
#define USART6_PCLK_EN()		(RCC->APB2_LPENR |=(1<<5))


/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()		(RCC->APB2_LPENR &=~(1<<4))
#define USART2_PCLK_DI()		(RCC->APB1_LPENR &=~(1<<17))
#define USART3_PCLK_DI()		(RCC->APB1_LPENR &=~(1<<18))
#define USART6_PCLK_DI()		(RCC->APB2_LPENR &=~(1<<5))

/*
 * Clock Enable Macros for UARTx peripherals
 */
#define UART4_PCLK_EN()			(RCC->APB1_LPENR |=(1<<19))
#define UART5_PCLK_EN()			(RCC->APB1_LPENR |=(1<<20))


/*
 * Clock Disable Macros for UARTx peripherals
 */
#define UART4_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<19))
#define UART5_PCLK_DI()			(RCC->APB1_LPENR &=~(1<<20))


/*
 * Some macros
 */
#define Enable			 	1
#define Disable				0
#define set 				Enable
#define Reset				Disable
#define GPIO_PIN_SET		set
#define GPIO_PIN_RESET		Reset
#define FLAG_RESET     	    Reset
#define FLAG_SET 			set


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8



/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9



#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_I2C.h"
#include "stm32f446xx_rcc.h"
#include "stm32f446xx_usart.h"
#include "stm32f446xx_timer.h"
#include "stm32f446xx_dac.h"
#include "stm32f446xx_adc.h"

#endif /* INC_STM32F446XX_DRIVER_H_ */
