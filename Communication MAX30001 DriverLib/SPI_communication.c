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
#include "SPI_communication.h"


/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig spiMasterConfig =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,                              // SMCLK Clock Source
        24000000,                                                   // SMCLK = DCO = 3MHZ
        3000000,                                                    // SPICLK = 500khz 500000
        EUSCI_B_SPI_MSB_FIRST,                                      // MSB First
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,    // Phase CPHA = 0
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,                   // Low polarity CPOL = 0
        EUSCI_B_SPI_3PIN                                            // 3Wire SPI Mode
};

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
