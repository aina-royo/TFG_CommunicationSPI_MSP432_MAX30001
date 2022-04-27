 /*******************************************************************************
 * delay.c
 *
 *  Created on: 25 d’abr. 2022
 *      Author: ainar
 *
 * MSP432 Timer_A - VLO Period Capture
 *
 * Description: Capture a number of periods of the VLO clock and store them
 * in an array. When the set number of periods is captured the program is
 * trapped. At this point halt the program execution read out the values
 * using the debugger.
 *
 * ACLK = VLOCLK = 14kHz (typ.), MCLK = SMCLK = default DCO  = 3MHz
 *******************************************************************************/

/* DriverLib Includes */
#include <driverlib.h>

/* Standard Includes */
#include "msp.h"
#include <stdint.h>
#include <stdbool.h>

#define NUMBER_TIMER_CAPTURES

/* Timer_A Continuous Mode Configuration Parameter */
const Timer_A_ContinuousModeConfig continuousModeConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // SMCLK/1 = 3MHz
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
        TIMER_A_SKIP_CLEAR                   // Skip Clear Counter
};

/* Timer_A Capture Mode Configuration Parameter */
const Timer_A_CaptureModeConfig captureModeConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,        // CC Register 2
        TIMER_A_CAPTUREMODE_RISING_EDGE,          // Rising Edge
        TIMER_A_CAPTURE_INPUTSELECT_CCIxB,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};

/* Statics */
static volatile uint_fast16_t timerAcaptureValues[NUMBER_TIMER_CAPTURES];

void init_clock(void)
{
    /* Setting ACLK = VLO = 14kHz */
    CS_initClockSignal(CS_ACLK, CS_VLOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Configuring Capture Mode */
    Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);

    /* Configuring Continuous Mode */
    Timer_A_configureContinuousMode(TIMER_A0_BASE, &continuousModeConfig);

    /* Enabling interrupts and going to sleep */
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableInterrupt(INT_TA0_N);
    Interrupt_enableMaster();

    /* Starting the Timer_A0 in continuous mode */
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
}

//******************************************************************************
//
//This is the TIMERA interrupt vector service routine.
//
//******************************************************************************
void timer_a_ccr_isr(void)
{
    uint32_t jj;
    static volatile uint32_t timerAcapturePointer = 0;

    timerAcaptureValues[timerAcapturePointer++] = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    /*if(timerAcapturePointer >= NUMBER_TIMER_CAPTURES)
    {
        while(1)
        {
            for(jj=0;jj<10000;jj++);
        }
    }*/
}
