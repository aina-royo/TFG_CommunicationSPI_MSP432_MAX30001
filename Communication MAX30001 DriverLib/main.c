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

/* DIFFERENTS ECG OR BIOZ DATA TAGS */
#define VALID       0x00
#define FAST        0x01
#define VALID_EOF   0x02
#define FAST_EOF    0x03
#define UNUSED_1    0x04
#define UNUSED_2    0x05
#define EMPTY       0x06
#define OVERFLOW    0x07

/* Statics */
uint8_t DataEcg[3];         // buffer on emmagatzemem les dades de lectura de ECG 3 byte data word
uint8_t DataBioZ[3];        // buffer on emmagatzemem les dades de lectura del BioZ 3 byte data word
uint8_t DataReceived[3];    // buffer where we keep the data we receive from SPI communication

uint8_t fifo_ECG = 0x1F;    // 0 1 0   0 0 0 0 | 1 ultim 1 per indicar lectura seria 0x43 0x20+read 1 BURST MODE
uint8_t fifo_BioZ = 0x45;   // 0 1 0   0 0 1 0 | 1 ultim 1 per indicar lectura seria 0x47 BURST MODE

uint8_t Flag_ecgbioz;

int main(void)
{
    /* Halting the watchdog */
    WDT_A_holdTimer();

    init_uart();
    init_spi();

    Flag_ecgbioz = 0;

    while (1)
    {
        if(Flag_ecgbioz == 0)
        {
            /* Enabling interrupts */
            UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
            Interrupt_enableInterrupt(INT_EUSCIA0);
        }
        else if(Flag_ecgbioz == 1 && Flag_ecgbioz == 2)
        {
            /* Enabling interrupts SPI */
            SPI_enableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);
            SPI_enableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
            Interrupt_enableInterrupt(INT_EUSCIB2);
        }
        else if(Flag_ecgbioz == 3)
        {
            /* Enabling interrupts */
            UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
            Interrupt_enableInterrupt(INT_EUSCIA0);
        }
        else
        {
            /* Sleeping when not in use */
            PCM_gotoLPM0();
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
    uint32_t jj;

    SPI_clearInterruptFlag(EUSCI_B2_BASE, status);

    while ((SPI_isBusy(EUSCI_B2_BASE)==EUSCI_SPI_BUSY));

    if(Flag_ecgbioz == 1)
    {
        /**********************FIRST COMMUNICATION WITH ECG SIGNAL************************************/
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer

        /* Send the next data packet */
        SPI_transmitData(EUSCI_B2_BASE, fifo_ECG);     // send the register to read the raw data of ECG

        while (!(SPI_getInterruptStatus(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        DataReceived[0] = SPI_receiveData(EUSCI_B2_BASE);   // we receive the data

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        DataReceived[1] = SPI_receiveData(EUSCI_B2_BASE);   // we receive the data

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        DataReceived[2] = SPI_receiveData(EUSCI_B2_BASE);   // we receive the data

        GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // chip select high

        SPI_disableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);  // we disable the interrupt
        SPI_disableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT); // we disable the interrupt

        Flag_ecgbioz = 3;                                                    // reset the flag
        /*******************************END FIRST COMMUNICATION***************************************/
    }
    else if(Flag_ecgbioz == 2)
    {
        /**********************SECOND COMMUNICATION WITH BIOZ SIGNAL**********************************/

        GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer

        /* Send the next data packet */
        SPI_transmitData(EUSCI_B2_BASE, fifo_BioZ);    // send the register to read the raw data of BioZ

        while (!(SPI_getInterruptStatus(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        DataReceived[0] = SPI_receiveData(EUSCI_B2_BASE);  // we receive the data

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        DataReceived[1] = SPI_receiveData(EUSCI_B2_BASE);  // we receive the data

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        DataReceived[2] = SPI_receiveData(EUSCI_B2_BASE);  // we receive the data

        GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // chip select high

        SPI_disableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);  // we disable the interrupt
        SPI_disableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT); // we disable the interrupt

        Flag_ecgbioz = 3;                                                    // reset the flag
        /*******************************END FIRST COMMUNICATION***************************************/
    }
    else
    {
        while(1);         /*NOT IDEA CUELGATE*/
    }
}

//******************************************************************************
//
//EUSCI A0 UART ISR interrupt vector service routine. Echoes data back to PC host
//
//******************************************************************************
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    uint32_t jj;

    UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    //UART_transmitData(EUSCI_A0_BASE, 0x70);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
        // if(status & EUSCI_A_UART_RECEIVE_INTERRUPT)
        // UART_transmitData(EUSCI_A0_BASE, UART_receiveData(EUSCI_A0_BASE));

        Flag_ecgbioz = UART_receiveData(EUSCI_A0_BASE);
        UART_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // disable the interrupt
    }

    if(status & EUSCI_A_UART_TRANSMIT_INTERRUPT)
    {
        UART_transmitData(EUSCI_A0_BASE, DataReceived[0]);
        UART_transmitData(EUSCI_A0_BASE, DataReceived[1]);
        UART_transmitData(EUSCI_A0_BASE, DataReceived[2]);

        Flag_ecgbioz = 0;                                                      // we reset ALL the communication

        UART_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT); // disable the interrupt
    }
}
