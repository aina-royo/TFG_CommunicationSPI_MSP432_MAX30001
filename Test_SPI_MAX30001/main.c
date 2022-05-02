/******************************************************************************
 * SPI_communication.c
 *
 *  Created on: 24 d’abr. 2022
 *      Author: ainar
 *
 * MSP432 SPI - 3-wire Master Incremented Data
 *
 * This example shows how SPI master talks to SPI slave using 3-wire mode.
 * Incrementing data is sent by the master starting at 0x01. Received data is
 * expected to be same as the previous transmission.  eUSCI RX ISR is used to
 * handle communication with the CPU, normally in LPM0. Because all execution
 * after LPM0 is in ISRs, initialization waits for DCO to stabilize against
 * ACLK.
 *
 * Note that in this example, EUSCIB0 is used for the SPI port. If the user
 * wants to use EUSCIA for SPI operation, they are able to with the same APIs
 * with the EUSCI_AX parameters.
 *
 * ACLK = ~32.768kHz, MCLK = SMCLK = DCO 3MHz
 *
 * Use with SPI Slave Data Echo code example.
 *
 *                MSP432P401
 *              -----------------
 *             |                 |
 *             |                 |
 *             |                 |
 *             |             P3.6|-> Data Out (UCB0SIMO) (MOSI) Slave uses SDI 3.6
 *             |                 |
 *             |             P3.7|<- Data In (UCB0SOMI) (MISO) Slave uses SDO 3.7
 *             |                 |
 *             |             P3.5|-> Serial Clock Out (UCB0CLK) 3.5
 *             |                 |
 *             |             P2.5|-> ChipSelect of SPI slave
 *             |                 |
 * Author: Timothy Logan
 *******************************************************************************/

/* DriverLib Includes */
#include <driverlib.h>

/* Standard Includes */
#include "msp.h"
#include <stdint.h>
#include <stdbool.h>


/* Statics */
uint8_t DataEcg[3];         // buffer on emmagatzemem les dades de lectura de ECG 3 byte data word
uint8_t DataBioZ[3];        // buffer on emmagatzemem les dades de lectura del BioZ 3 byte data word

uint8_t fifo_ECG = 0x1F;    // 0 1 0   0 0 0 0 | 1 ultim 1 per indicar lectura seria 0x43 0x20+read 1 BURST MODE
uint8_t fifo_BioZ = 0x45;   // 0 1 0   0 0 1 0 | 1 ultim 1 per indicar lectura seria 0x47 BURST MODE

uint8_t Flag_ecgbioz;

/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig spiMasterConfig =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,                              // SMCLK Clock Source
        24000000,                                                   // SMCLK = DCO = 3MHZ (24MHz)
        3000000,                                                    // SPICLK = 500khz 500000 (3MHz)
        EUSCI_B_SPI_MSB_FIRST,                                      // MSB First
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,    // Phase CPHA = 0
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,                   // Low polarity CPOL = 0
        EUSCI_B_SPI_3PIN                                            // 3Wire SPI Mode
};

int main(void)
{
    /* Halting the watchdog */
    WDT_A_holdTimer();

    Flag_ecgbioz = 1;
	
	init_spi();

	return 0;
}

void init_spi(void)
{
    /* initialize SPI */
    /* Selecting P1.5 P1.6 and P1.7 in SPI mode */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                                               SCLK | MOSI | MISO, GPIO_PRIMARY_MODULE_FUNCTION);

    //Configure pin CS1
    GPIO_setAsOutputPin(GPIO_PORT_P2, CS1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // initialize chip select as high

    /* Configuring SPI in 3wire master mode */
    SPI_initMaster(EUSCI_B2_BASE, &spiMasterConfig);

    /* Enable SPI module */
    SPI_enableModule(EUSCI_B2_BASE);
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
