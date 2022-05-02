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

uint32_t valueSMCLK;

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upModeConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source 24MHz
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 24 MHz-> TIMER_A_CLOCKSOURCE_DIVIDER_8 -> 3MHz
        8,                                      // 5000 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,     // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};


void init_clock(void)
{
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24); //sets the DCOCLK to 24MHz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    valueSMCLK = CS_getSMCLK();

    /* Configuring Timer_A1 for Up Mode */
    Timer_A_configureUpMode(TIMER_A1_BASE, &upModeConfig);

    /* Enabling interrupts and starting the timer */
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableInterrupt(INT_TA1_0);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}

