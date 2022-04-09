/*
 * -------------------------------------------
 *    MSP432 DriverLib - v3_21_00_05
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
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
 *             |             P1.6|-> Data Out (UCB0SIMO) (MOSI) Slave uses SDI 3.6
 *             |                 |
 *             |             P1.7|<- Data In (UCB0SOMI) (MISO) Slave uses SDO 3.7
 *             |                 |
 *             |             P1.5|-> Serial Clock Out (UCB0CLK) 3.5
 *             |                 |
 *             |             P2.5|-> ChipSelect of SPI slave
 *             |                 |
 * Author: Timothy Logan
*******************************************************************************/
/* DriverLib Includes */
#include <driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include "msp.h"
#include <stdbool.h>

/* SPI COMUNICATION DEFINES */
#define MOSI        GPIO_PIN6
#define MISO        GPIO_PIN7
#define SCLK        GPIO_PIN5
#define CS1         GPIO_PIN5

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
uint8_t DataEcg[3]; // buffer on emmagatzemem les dades de lectura de ECG 3 byte data word
/* al tenir 32 words de 24 bits putse hauria de ser un uint32_t DataEcg[32] */
uint8_t DataBioZ[3]; // buffer on emmagatzemem les dades de lectura del BioZ 3 byte data word
/* al tenir 8 words de 24 bits putse hauria de ser un uint32_t DataEcg[8] */
uint8_t fifo_ECG = 0x1F; // 0 1 0   0 0 0 0 | 1 ultim 1 per indicar lectura seria 0x43 0x20+read 1 BURST MODE
uint8_t fifo_BioZ = 0x45; // 0 1 0   0 0 1 0 | 1 ultim 1 per indicar lectura seria 0x47 BURST MODE

uint8_t ECG_etag = 0; // ECG data tag [2:0]
uint8_t ECG_ptag = 0; // The PACE FIFO data content is closely linked to ECG FIFO content [2:0]

uint32_t ECG_SVD = 0; // ECG Sample Voltage Data [17:0] (tenim 32 words de 24 bits ?)
uint32_t BioZ_SVD = 0; // BioZ Sample Voltage Data [19:0] (tenim 8 words de 24 bits ?)


/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig spiMasterConfig =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,                              // SMCLK Clock Source
        24000000,                                                    // SMCLK = DCO = 3MHZ
        3000000,                                                     // SPICLK = 500khz 500000
        EUSCI_B_SPI_MSB_FIRST,                                      // MSB First
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,    // Phase CPHA = 0
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,                   // Low polarity CPOL = 0
        EUSCI_B_SPI_3PIN                                            // 3Wire SPI Mode
};


int main(void)
{

    //EUSCI_B_CTLW0_STEM
    /* Halting the watchdog */
    WDT_A_holdTimer();

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

    /* Enabling interrupts */
    SPI_enableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);
    SPI_enableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIB2);
    Interrupt_enableSleepOnIsrExit();

    Interrupt_enableMaster();

    /* Sleeping when not in use */
    while (1)
    {
        MAP_PCM_gotoLPM0();
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

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer
    /* Send the next data packet */
    SPI_transmitData(EUSCI_B2_BASE, fifo_ECG); // send the register where the raw data of ECG

    while (!(SPI_getInterruptStatus(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

    SPI_transmitData(EUSCI_B2_BASE, 0xFF); // send the register where the raw data of ECG
    DataEcg[0] = SPI_receiveData(EUSCI_B2_BASE);

    SPI_transmitData(EUSCI_B2_BASE, 0xFF); // send the register where the raw data of ECG
    DataEcg[1] = SPI_receiveData(EUSCI_B2_BASE);

    SPI_transmitData(EUSCI_B2_BASE, 0xFF); // send the register where the raw data of ECG
    DataEcg[2] = SPI_receiveData(EUSCI_B2_BASE);


    /* Send the next data packet */
    //SPI_transmitData(EUSCI_B2_BASE, fifo_BioZ);

    GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // chip select high



    /*if(status & EUSCI_B_SPI_TRANSMIT_INTERRUPT)
    {
        while ((SPI_isBusy(EUSCI_B2_BASE)==EUSCI_SPI_BUSY));

        GPIO_setOutputLowOnPin(GPIO_PORT_P4, CS1);     // chip select low to start the transfer
        /* Send the next data packet
        SPI_transmitData(EUSCI_B2_BASE, fifo_ECG); // send the register where the raw data of ECG

        for(jj=50;jj<50;jj++);
        //SPI_disableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT); //solo hace una vez

        while ((SPI_isBusy(EUSCI_B2_BASE)==EUSCI_SPI_BUSY));

        GPIO_setOutputLowOnPin(GPIO_PORT_P4, CS1);     // chip select low to start the transfer
        /* Send the next data packet
        SPI_transmitData(EUSCI_B2_BASE, fifo_BioZ); // send the register where the raw data of ECG
    }

    if(status & EUSCI_B_SPI_RECEIVE_INTERRUPT)
    {

       /* USCI_B0 TX buffer ready?
       while (!(SPI_getInterruptStatus(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

       DataEcg[0] = SPI_receiveData(EUSCI_B2_BASE);
       DataEcg[1] = SPI_receiveData(EUSCI_B2_BASE);
       DataEcg[2] = SPI_receiveData(EUSCI_B2_BASE);

       /* Send the next data packet
       //SPI_transmitData(EUSCI_B2_BASE, fifo_BioZ);

       GPIO_setOutputHighOnPin (GPIO_PORT_P4, CS1);    // chip select high

       /* Delay between transmissions for slave to process information*
       for(jj=50;jj<50;jj++);

       /* USCI_B0 TX buffer ready?
       while (!(SPI_getInterruptStatus(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

       /* Send the next data packet
       //SPI_transmitData(EUSCI_B2_BASE, fifo_BioZ);

       DataBioZ[0] = SPI_receiveData(EUSCI_B2_BASE);
       DataBioZ[1] = SPI_receiveData(EUSCI_B2_BASE);
       DataBioZ[2] = SPI_receiveData(EUSCI_B2_BASE);

       GPIO_setOutputHighOnPin (GPIO_PORT_P4, CS1);    // chip select high

       //SPI_disableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
    }*/

    ECG_etag = (DataEcg[0]>>3)&0x0F;
    ECG_ptag = DataEcg[0]&0x0F;

    switch(ECG_etag)
    {
        case (VALID):
                /* DATA VALID AND TIME VALID */
                ECG_SVD = (((DataEcg[0]>>6)&0x000F) |
                            ((DataEcg[1]>>6)&0x00FF)|
                            ((DataEcg[2]>>6)&0xFF00));
                break;

        case (FAST):
                /* NO DATA VALID BUT VALID TIME */
                /* continue to get data  how ?? */
                break;

        case (VALID_EOF):
                /* DATA VALID AND TIME VALID BUT LAST SAMPLE */
                ECG_SVD = (((DataEcg[0]>>6)&0x000F) |
                            ((DataEcg[1]>>6)&0x00FF)|
                            ((DataEcg[2]>>6)&0xFF00));
                break;

        case (FAST_EOF):
                /* NO DATA VALID BUT VALID TIME */
                /* continue to get data  how ?? */
                break;

        case ((UNUSED_1)|(UNUSED_2)):
                /*unused what to do ?? */
                break;

        case (EMPTY):
                /* NO DATA VALID AND TIME VALID
                 * Suspend read back operations on this FIFO until more samples are available.
                 * */

                break;

        case (OVERFLOW):
                /* NO DATA VALID AND TIME VALID
                * Issue a FIFO_RST command to clear the FIFOs or re-SYNCH if necessary.
                * Note the corresponding halt and resumption in ECG/BioZ time/voltage records.
                * */
                break;

        default:
                /* what to do in default */
            break;
    }

}
