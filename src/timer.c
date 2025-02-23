/*
 * timer.c
 *
 *  Created on: Apr 19, 2022
 *      Author: comp 59
 */
#include "stm32f446xx_driver.h"
#include <stdint.h>
void delay_ms (int delay );

int main(void)
{
	 tim2_pa5_OPM_output_compare_PWM_conf() ;
	 tim2_pa5_PWM( 20);
	 delay_ms(100);
/*/
	uint32_t brightness =0 ;
	 while(1)
	 {
	 while (brightness <9999)
	 {
		 brightness +=100;
		 tim2_pa5_PWM(  brightness);

		 delay_ms(1000);
	 }
	 while (brightness >0)
		 {
			 brightness -=100;
			 tim2_pa5_PWM( brightness);

			 delay_ms(1000);
		 }
	 }
	// delay_ms(10000);

/*/
}
void delay_ms (int delay )
{
	int i,j ;
	for (j=delay; j>0 ; j--)
	{
		for(i=0;i<3195;i++);
	}
}
