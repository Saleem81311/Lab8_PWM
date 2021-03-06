/*****************************************************************************************************************
 * Author: Tennison Hoffmann
 * Date: 10/28/2021
 * Group: Figurski, Griggs-Taylor, Hoffmann
 * Description: Program to accept input from a speed entered into the keypad and make the motor run at that 
 * speed.
 *****************************************************************************************************************/
#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
//eyo 
int Read_Keypad (void);                         //function declarations
int arrayToDuty(int array[]);
void Timer_PWM(int duty);
void Timer_Init(void);
void Pin_Init(void);

void main(void)
{
     WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

     int i = 0, num, code[3], newDuty = 0;

     Pin_Init();                                //initialize the pins and the timer
     Timer_Init();

     printf("\nEnter a 3 digit value for speed from 0 to 100 followed by #.\n"); //prompt user

     while (1){
         num = Read_Keypad();                   //get one key press from the keypad
         if ((num != 10) && (num > 0) && (num < 13)){
             if (num != 12){
                 if (i >= 3){
                     printf("Too many digits: Start over!\n");
                     i = 0;
                 }
                 else {
                     if (num == 11)
                         num = 0;
                     code[i] = num;             //put key presses into an array
                     i++;
                 }
             }
             else {
                 if (i < 3){
                     printf("More digits required!\n");
                 }
                 else {
                     newDuty = arrayToDuty(code);
                     if (newDuty > 100){
                         printf("Speed too large: Start over!\n");
                         i = 0;
                     }
                     else {
                         printf("Speed Entered: %d\n", newDuty);    //confirms speed
                         Timer_PWM(newDuty);                        //changes timer to new PWM
                         i = 0;
                     }
                 }
             }
         }
     }
}

int Read_Keypad (void){ //detects and reads key pressed
    int i, j, key = 0;
    for (i = 4; i < 7; i++){
        P4->DIR |= (0x01 << i);
        P4->OUT &= ~(0x01 << i);
        __delay_cycles(5000);
        for (j = 0; j < 4; j++){
            if (!(P4->IN & BIT(j))){
                while (!(P4->IN & BIT(j)));
                if (j == 0)
                    key = i - 3;
                else if (j == 1)
                    key = i;
                else if (j == 2)
                    key = i + 3;
                else
                    key = i + 6;
            }
        }
        P4->DIR &= ~(0x7F);
        P4->OUT |= 0x7F;
    }
    return key;
}

void Timer_Init(void){              //Initializes the timer
    TIMER_A0-> CTL = 0b1001010100;                      //Count up using smclk, clears TAOR register, /2
    TIMER_A0-> CCR[0] = 37500 - 1;                      //TimerA will count up to 37500-1
    TIMER_A0-> CCR[4] = 0;                              //motor off to start
    TIMER_A0-> CCTL[4] = 0b11100000;                    //reset/set mode
}

void Pin_Init(void){
    P2-> SEL0 |= BIT7;
    P2-> SEL1 &=~ BIT7;
    P2 -> DIR |= BIT7;                                 //sets pin 2.7 for GPIO as an output

    P4->SEL0 &= ~(0x7F);                               //initialize all pins to GPIO
    P4->SEL1 &= ~(0x7F);                               //as inputs with pull up resistors
    P4->DIR &= ~(0x7F);
    P4->REN |= 0x7F;
    P4->OUT |= 0x7F;
}

int arrayToDuty(int code[]){            //converts the array of digits into a 3 digit integer
     int newDuty;

     newDuty = (code[0] * 100) + (code[1] * 10) + code[2];
     

     return newDuty;
}

void Timer_PWM(int duty){               //runs timer given the specified duty cycle

    TIMER_A0-> CTL = 0b1001010100;                      //Count up using smclk, clears TAOR register, /2
    TIMER_A0-> CCR[0] = 37500 - 1;                      //TimerA will count up to 37500-1
    if (duty == 0)
        TIMER_A0-> CCR[4] = 0;
    else
        TIMER_A0-> CCR[4] = (37500 * duty / 100) - 1;   //Sets the duty cycle.
    TIMER_A0-> CCTL[4] = 0b11100000;                    //reset/set mode
}
