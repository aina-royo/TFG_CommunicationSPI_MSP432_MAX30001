
/* DriverLib Includes */
#include <driverlib.h>

/* Standard Includes */
#include "msp.h"
#include <stdint.h>
#include <stdbool.h>


void delay_init(void)
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

int main(void)
{
    WDT_A_holdTimer();

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);                   // 24000000 Hz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // 24000000 Hz

    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3);

    delay_init();

    while(1)
    {
        delay(1 * 1000); // 1s (* 1000)x2 --> 1ms * 1000 --> 1us

        GPIO_toggleOutputOnPin(GPIO_PORT_P3, GPIO_PIN3);
    }
}
