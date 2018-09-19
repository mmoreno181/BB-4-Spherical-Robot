#include "msp.h"
#include "driverlib.h"
#include <stdio.h>
#include <stdbool.h>
#include "msp.h"
#include <math.h>
#include <stdlib.h>
#include "dcmotor.h"


#define SM_PORT GPIO_PORT_P3 // SM -> 'Stepper Motor'
#define SM_DIR_PIN GPIO_PIN7
#define SM_STEP_PIN GPIO_PIN5
#define FULL_STEPS_RANGE 500
#define HALF_STEPS_RANGE 250
#define SOFTWARE_LIMIT 200
#define DELAY_LENGTH 1000

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define ROLLING_LENGTH 7
#define KP 80
#define KD .5

volatile int cur_steps = 0;
volatile char array[4] = "";

volatile int counter = 0;
volatile int counter2 = 0;
volatile float rollingVec[ROLLING_LENGTH] = {0};
volatile float t_curr = 0;
volatile float t_prev = 0;
volatile float dt = 0;
volatile float period = 0;
volatile float total = 0;
volatile float rpm = 0;
volatile int dir = 1;
volatile float last_error = 0;
float rpm_des = 90;

volatile float val_arr [100];
volatile int val_count;

const Timer_A_UpModeConfig upConfig_0 = // Configure counter in Up mode
{
    TIMER_A_CLOCKSOURCE_SMCLK, // Tie Timer A to SMCLK
    TIMER_A_CLOCKSOURCE_DIVIDER_64, // Increment counter every 64 clock cycles
    300, // Period of Timer A (this value placed in TAxCCR0)
    TIMER_A_TAIE_INTERRUPT_DISABLE, // Disable Timer A rollover interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable Capture Compare interrupt
    TIMER_A_DO_CLEAR // Clear counter upon initialization
};

volatile int position;

void step(int num_steps, int delay_length, int direction) {
    // direction -> 0 : cw
    // direction -> 1 : ccw
    int cur_steps = 0;
    if (direction==1) {
        GPIO_setOutputLowOnPin(SM_PORT, SM_DIR_PIN);
    } else if (direction==0) {
        GPIO_setOutputHighOnPin(SM_PORT, SM_DIR_PIN);
    }

//    if (cur_steps < HALF_STEPS_RANGE && cur_steps > -HALF_STEPS_RANGE) {
        while(cur_steps < num_steps) {
               GPIO_setOutputHighOnPin(SM_PORT, SM_STEP_PIN);
               delay_time(delay_length); // dont go lower than 120 delay length ... do we need this?
               GPIO_setOutputLowOnPin(SM_PORT, SM_STEP_PIN);
               delay_time(delay_length);
               cur_steps++;
           }
//    }

}
void main(void)
    {
    /* Disabling the Watchdog  */
    WDT_A_holdTimer();
    FPU_enableModule();

    initializeEnc();

    CS_setDCOFrequency(3E+6); // Set DCO clock source frequency
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // Tie SMCLK to DCO

    // 2 mm/step
    // Limit switch will be to the left (facing motor), so motor rotates CCW to get it
    // Total length is 1651 mm = 825.5 steps, so that halfway is at 825.5 mm = 412.75 steps.
    stepper_setup();

    // Define GPIOs and UART TX/RX. A0 is to the computer. A3 is to the Arduino.
    // 9.7 is TX
    // 3.3 TX -> connect ot arduino RX
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    // Enable Module
    UART_enableModule(EUSCI_A0_BASE);
    UART_enableModule(EUSCI_A2_BASE);

    // Enable UART receive interrupts
        UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Also enable interrupt master...
        Interrupt_enableInterrupt(INT_EUSCIA0);
    Interrupt_enableInterrupt(INT_EUSCIA2);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN3);
    GPIO_setAsInputPin(GPIO_PORT_P8,GPIO_PIN7);
    GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);


//    homing();


    init_PWM();
    change_duty1(2500);


    Interrupt_enableMaster();
    while(1) {
        if (GPIO_getInputPinValue(GPIO_PORT_P8,GPIO_PIN7)==1 && (position < HALF_STEPS_RANGE+SOFTWARE_LIMIT)) {
            step(1,DELAY_LENGTH,1);
            position++;
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN4)==1 && (position > -HALF_STEPS_RANGE-SOFTWARE_LIMIT)) {
            step(1, DELAY_LENGTH, 0);
            position--;
        }
    }
}

//
// Timer ISR


void TA1_0_IRQHandler(void)
{
    t_curr += 0.0001;
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

void PORT5_IRQHandler(void) {
    dt = t_curr - t_prev;
    period += dt;
    int num_ticks = 5;
    if (counter%num_ticks == 0 && counter != 0) {
        int i = counter2 % ROLLING_LENGTH;
        total = total - rollingVec[i] + period*(30/num_ticks);
        rollingVec[i] = period*(30/num_ticks);
        rpm = (1/(total/ROLLING_LENGTH))*60;
        //printf('Average rpm: %f \n', rpm);
        period = 0;
        compute_control();
    }
    t_prev = t_curr;
    counter++;
    if (counter % 90 == 0 && counter != 0) {
        if (dir) {
            dir = 0;
        } else {
            dir = 0;//1;
        }
        total = 0;
        int l = 0;
        for (l = 0; l < ROLLING_LENGTH; l++) {
            rollingVec[l] = 0;
        }
        rpm = 0;
        if (dir) {
            change_duty1(2500);
        } else {
            change_duty2(2500);
        }
        compute_control();
    }
    GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN2) ; // clear IFG flag
}

void initializeEnc(void) {
    // GPIO Interrupt for switch one.
    GPIO_setAsInputPin(GPIO_PORT_P5,GPIO_PIN2);
    GPIO_setAsInputPin(GPIO_PORT_P3,GPIO_PIN6);
    GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_enableInterrupt (GPIO_PORT_P5, GPIO_PIN2);
    GPIO_clearInterruptFlag (GPIO_PORT_P5, GPIO_PIN2);
    GPIO_registerInterrupt (GPIO_PORT_P5, *PORT5_IRQHandler);
    Interrupt_enableInterrupt(INT_PORT5);
}

void compute_control(void) {
    float dutyCycle;
    float error = rpm_des-rpm;
    dutyCycle = KP*(error)+KD*(error-last_error);
    dutyCycle = MIN(dutyCycle, 2900);
    dutyCycle = MAX(dutyCycle, 750);
//    printf("Duty Cycle: %f\n", dutyCycle);
    last_error = error;
    if (dir) {
        change_duty1(dutyCycle);
    } else {
        change_duty2(dutyCycle);
    }
}

void delay_time(int delay_length) {
    int i = 0;
    for (i=0; i<delay_length; i++);
}

void homing(void) {
    GPIO_setOutputHighOnPin(SM_PORT, SM_DIR_PIN); // CCW
    while (GPIO_getInputPinValue(GPIO_PORT_P2,GPIO_PIN3)!=0) {
        step(1,DELAY_LENGTH, 0);
    }
    delay_time(DELAY_LENGTH);
    GPIO_setOutputLowOnPin(SM_PORT, SM_DIR_PIN); // CW
    step(HALF_STEPS_RANGE,DELAY_LENGTH, 1);
    position  = 0;
}



void stepper_setup(void) {
    GPIO_setAsOutputPin(SM_PORT, SM_DIR_PIN | SM_STEP_PIN);
}
