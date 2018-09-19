/*
 * stepper.h
 *
 *  Created on: Apr 16, 2018
 *      Author: Maria
 */

#ifndef STEPPER_H_
#define STEPPER_H_

#include "pins.h"
#include "driverlib.h"

#define FULL_STEPS_RANGE 3000//825
#define HALF_STEPS_RANGE 1500//413

void stepper_setup(void) {
    GPIO_setAsOutputPin(SM_PORT, SM_DIR_PIN);
    GPIO_setAsOutputPin(SM_PORT, SM_STEP_PIN);
    GPIO_setAsInputPin(LIM_PORT, LIM_PIN);
}

void step(int direction, int step_num) {
    // direction -> 0 : cw
    // direction -> 1 : ccw
    // take care of time delay using interrupt in main function
//    int cur_steps = 0;
    if (direction==1) {
        GPIO_setOutputLowOnPin(SM_PORT, SM_DIR_PIN);
    } else if (direction==0) {
        GPIO_setOutputHighOnPin(SM_PORT, SM_DIR_PIN);
    }

    if (step_num % 2 == 0) {
        GPIO_setOutputHighOnPin(SM_PORT, SM_STEP_PIN);
    } else {// if (step_num % 2 == 1) {
        GPIO_setOutputLowOnPin(SM_PORT, SM_STEP_PIN);

    }
}


#endif /* STEPPER_H_ */
