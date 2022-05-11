/*
 * delay.c
 *
 *  Created on: 2 de maig 2022
 *      Author: ainar
 */


/* DriverLib Includes */
#include <driverlib.h>

/* Standard Includes */
#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include "delay.h"


void init_delay(void)
{
    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);

    Timer32_disableInterrupt(TIMER32_0_BASE);
}

void delay(uint32_t duration_us)
{
    Timer32_haltTimer(TIMER32_0_BASE);
    Timer32_setCount(TIMER32_0_BASE, 24 * duration_us);
    Timer32_startTimer(TIMER32_0_BASE, true);

    while (Timer32_getValue(TIMER32_0_BASE) > 0);
}
