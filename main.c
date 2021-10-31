/************************************************************************************************************************
 * Name: Saleem Griggs-Taylor, Kevin Figurski, and Tennison Hoffmann
 * Start Date: 10/21/21
 * Last modified: 10/29/21
 * Description: This program is made to interface a motor with the MSP432 utilizing the SysTick and Registers to make
 * the PWM  q?
 ************************************************************************************************************************/
#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
uint8_t Duty_Cycle=50;

//void SysTick_Init();
//void SysTick_Delay(uint8_t delay);
//void MotorPWM(int DC);
void PWM_Init(void);

uint16_t f_CLK=3000; // Master Clock (3MHz) but converted to msec notation
uint8_t T=25; // in msec

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	P2->SEL0 |=BIT6;    //for timera
	P2->SEL1 &=~BIT6;
	P2->DIR  |= BIT6;
	PWM_Init();

	while(1)
	{

	 //P4->OUT |= BIT4;       // Turning on the Motor's PWM to get it to move
	 //MotorPWM(Duty_Cycle);      // Calling the custom function that utilizes the SysTick function to set the runtime of the motor
	 //P4->OUT &= ~BIT4;      // Turning off the motor at its PWM pin to begin slowing the motor down
	 //MotorPWM(100-Duty_Cycle);

	 /******************************************************** Part 2 *********************************************************/

	    TIMER_A0->CCR[3]=(T*f_CLK/200)*Duty_Cycle;

	}


}
void PWM_Init(void) //Initialize P2.6
{
    TIMER_A0->CTL=TIMER_A_CTL_TASSEL_2|TIMER_A_CTL_ID_1|TIMER_A_CTL_MC_1|TIMER_A_CTL_CLR; // work with 3 MHz, divide your 3MHz by 2, Use Up Mode, Clear
    TIMER_A0->CCR[0]=T*f_CLK/2; // be careful we divided master clock by 2 that is the reason of the 1/2 there
    TIMER_A0->CCR[3]=(T*f_CLK/200)*Duty_Cycle;
    TIMER_A0->CCTL[3]=TIMER_A_CCTLN_OUTMOD_7;
}

/*void MotorPWM(int DC)
{
    SysTick->LOAD=((75000*DC*0.01)-1);      // 75000 is equivalent to 40Hz with proper math
    SysTick->VAL = 5;
    SysTick->CTRL = 1;
    while(!((SysTick->CTRL & 0x00010000) == 0x00010000));
}


void SysTick_Init()
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 32;
    SysTick->CTRL = 0x00000005;
}

void SysTick_Delay(uint8_t delay)
{
  SysTick->LOAD = ((delay*3000)-1);
  SysTick->VAL = 5;
  while((SysTick->CTRL & 0x00010000)== 0x00010000);
}*/

