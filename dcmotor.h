/*
 * dcmotor.h
 *
 *  Created on: Apr 16, 2018
 *      Author: Maria
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

void init_PWM(void) {
    // Pin 2.6
    P2SEL0 |= 0x40 ; // Set bit 5 of P2SEL0 to enable TA0.1 functionality on P2.4
    P2SEL1 &= ~0x40 ; // Clear bit 5 of P2SEL1 to enable TA0.1 functionality on P2.4
    P2DIR |= 0x40 ; // Set pin 2.5 as an output pin
    // Set Timer A period (PWM signal period)
    TA0CCR0 = 3000 ;
    // Set Duty cycle
    TA0CCR3 = 0;
    TA0CCTL3 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
    // Initialize Timer A
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R
    //TA0CCR1 = 3000*dutyCycle;

    // Pin 6.7
    P6SEL0 |= 0x80 ; // Set bit 4 of P2SEL0 to enable TA0.1 functionality on P2.4
    P6SEL1 &= ~0x80 ; // Clear bit 4 of P2SEL1 to enable TA0.1 functionality on P2.4
    P6DIR |= 0x80 ; // Set pin 2.4 as an output pin
    TA2CCR0 = 3000 ;
    // Set Duty cycle
    TA2CCR4 = 0;
    TA2CCTL4 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
    // Initialize Timer A
    TA2CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R

}

void change_duty1(float dutyCycle) {
    TA0CCR3 = dutyCycle;
    TA2CCR4 = 0;
}

void change_duty2(float dutyCycle) {
    TA0CCR3 = 0;
    TA2CCR4 = dutyCycle;
}

#endif /* DCMOTOR_H_ */
