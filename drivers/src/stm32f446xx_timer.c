/*
 * stm32f446xx_timer_driver.c

 *
 *  Created on: Feb 8, 2022
 *      Author: MAZEN FAYED
 */

#include "stm32f446xx_timer.h"



#define CR1_CEN         (1U<<0)   // TO ENABLE TIME2

#define OC_TOGGLE        ((1U<<4)  |  (1U<<5))   //TO CONFIGURE TIMER2 CHANNEL ONE AS OUTPUT CAPTURE MODE


#define CCER_CC1E       (1U<<0)      // TO ENABLE TIMER2 IN COMPARE MODE

#define AFR5_TIM          (1U<<20)    //TO ENABLE GPIOA5 AS ALTERNATE FUNCTION 1

#define OPM             (   (1U<<3) |   (1U<<0) | (1U<<7))   //enable the counter & enable one pulse mode & enavle ARPE in cr1


#define PWM1      ((0U<<4) | (1U<<5)  |  (1U<<6)  |(1U<<3))    //enable the Preload Register & enable pwm mode 1 in ccmr1



TIM_RegDef_t *pTIM2 = TIM2 ;

GPIO_RegDef_t *pGPIOA1 = GPIOA ;


void tim2_1hz_init(void)
{
	 TIM1_PCLK_EN() ;                                     //ENABLE CLOCK
	pTIM2->PSC = 1600-1 ;                                   // SET PRESCALER VALUE
	pTIM2->ARR = 10000-1 ;                                  // SET AUTORELOAD VALUE
	pTIM2->CNT = 0 ;                                            //CLEAR COUNTER
	pTIM2->CR1 = CR1_CEN ;                                   //ENABLE TIMER
}


void tim2_pa5_output_compare(void)      // TO TOGGLE A PIN USING TIMER DIRECTLY
{
	GPIOA_PCLK_EN()	;                                         //ENABLE GPIOA CLOCK

	pGPIOA1->MODER &=~ (1U<<10) ;                          //SET PA5 MODE TO ALTERNATE FUNCTION BIT 10 TO ZERO
	pGPIOA1->MODER |=  (1U<<11) ;                           // CONTINUE SET PA5 MODE TO ALTERNATE FUNCTION BIT 11 TO ONE

	pGPIOA1->AFR[0] |=  AFR5_TIM ;                        // SET PA5 ALTERNATE FUNCTION TYPE TO TIM2_CH1  (AF1)

	 TIM1_PCLK_EN() ;                                     //ENABLE TIMER2 CLOCK
	pTIM2->PSC = 1600-1 ;                                   // SET PRESCALER VALUE
	pTIM2->ARR = 10000-1 ;                                  // SET AUTORELOAD VALUE
	pTIM2->CCMR1 = OC_TOGGLE ;                                 //SET OUTPUT COMPARE TOGGLE MODE
	pTIM2->CCER =	CCER_CC1E ;	                               //ENABLE TIMER2 CHANNEL1 IN COMPARE MODE WITH PA5 IN ALTURNATE FUNCTION MODE 0
	pTIM2->CNT = 0 ;                                            //CLEAR COUNTER
	pTIM2->CR1 = CR1_CEN ;                                   //ENABLE TIMER
}


void tim2_pa5_OPM_output_compare_PWM_conf()
{
	GPIOA_PCLK_EN()	;                                         //ENABLE GPIOA CLOCK

		pGPIOA1->MODER &=~ (1U<<10) ;                          //SET PA5 MODE TO ALTERNATE FUNCTION BIT 10 TO ZERO
		pGPIOA1->MODER |=  (1U<<11) ;                           // CONTINUE SET PA5 MODE TO ALTERNATE FUNCTION BIT 11 TO ONE

		pGPIOA1->AFR[0] |=  AFR5_TIM ;                        // SET PA5 ALTERNATE FUNCTION TYPE TO TIM2_CH1  (AF1)

		 TIM2_PCLK_EN() ;                                     //ENABLE TIMER2 CLOCK
		pTIM2->PSC = 1600-1 ;                                   // SET PRESCALER VALUE
		pTIM2->ARR = 10000-1 ;                                  // SET AUTORELOAD VALUE
		pTIM2->CNT = 0 ;                                            //CLEAR COUNTER


		pTIM2->CCMR1 = PWM1  ;                                 //SET OUTPUT COMPARE PWM mode 1
		pTIM2->CCER =	CCER_CC1E ;	                               //ENABLE TIMER2 CHANNEL1 IN COMPARE MODE WITH PA5 IN ALTURNATE FUNCTION MODE 0


		//pTIM2->CR1 =  OPM ;                 //ENABLE TIMER with one pulse mode



}

void tim2_pa5_PWM(float t)
{
	int x=pTIM2->ARR;
	pTIM2->CCR1 = t * x ;


}


