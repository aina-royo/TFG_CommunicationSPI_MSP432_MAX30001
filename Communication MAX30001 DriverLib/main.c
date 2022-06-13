 /******************************************************************************
 * -------------------------------------------
 *    MSP432 DriverLib - v3_21_00_05
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Author: ainar
 *
*******************************************************************************/

/* DriverLib Includes */
#include <driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include "msp.h"
#include <stdbool.h>
#include "UART_COM_PORT.h"
#include "SPI_communication.h"
#include "delay.h"

uint8_t DataReceivedCom[3];

uint32_t DataBioZ;
uint32_t DataEcg;

uint8_t Flag_ecgbioz;
uint8_t value;


bool info;

int main(void)
{
    /* Halting the watchdog */
    WDT_A_holdTimer();

    /* Set the clock frecuency */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);                    // 24000000 Hz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // 24000000 Hz

    init_delay();
    init_uart();
    init_spi();

    DataEcg = 0;
    DataBioZ = 0;

    /* Enabling recieve UART interrupt always */
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);

    info = max30001_ReadInfo();

    if(info==1)
    {
        /* READ INFO IS CORRECT */
    }
    else
    {
        while(info==0)
        {
            /* Stay here untill the issue is fixed */
            info = max30001_ReadInfo();
            delay(10000); // 10 ms
        }
    }

    value = 0;
    Flag_ecgbioz = 0;


    while (1)
    {
        switch(value)
        {
        case 1:
            if(Flag_ecgbioz == 0)
            {
                max30001_CalibrationECG();
                delay(100);
                Flag_ecgbioz = 1;
            }

            DataEcg = max30001_getEcgValue();
            UART_transmitData(EUSCI_A0_BASE, DataEcg >> 8);

            delay(10);
            break;
        case 2:
            if(Flag_ecgbioz == 1)
            {
                max30001_CalibrationBioZ();
                delay(100);
                Flag_ecgbioz = 0;
            }

            DataBioZ = max30001_getBioZValue();
            UART_transmitData(EUSCI_A0_BASE, DataBioZ >> 8);

            delay(15);
            break;
        default:

            break;
        }
    }
}

//******************************************************************************
//
//This is the EUSCI_B0 interrupt vector service routine.
//
//******************************************************************************
void EUSCIB2_IRQHandler(void)
{
    uint32_t status = SPI_getEnabledInterruptStatus(EUSCI_B2_BASE);

    SPI_clearInterruptFlag(EUSCI_B2_BASE, status);
}

//******************************************************************************
//
//EUSCI A0 UART ISR interrupt vector service routine. Echoes data back to PC host
//
//******************************************************************************
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    UART_clearInterruptFlag(EUSCI_A0_BASE, status);


    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
        value = UART_receiveData(EUSCI_A0_BASE);
        UART_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // disable the interrupt
    }

}
