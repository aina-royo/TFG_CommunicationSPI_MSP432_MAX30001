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

/* al tenir 32 words de 24 bits putse hauria de ser un uint32_t DataEcg[32] */
uint8_t DataBioZ[3];        // buffer on emmagatzemem les dades de lectura del BioZ 3 byte data word

/* al tenir 8 words de 24 bits putse hauria de ser un uint32_t DataEcg[8] */
uint8_t fifo_ECG = 0x1F;    // 0 1 0   0 0 0 0 | 1 ultim 1 per indicar lectura seria 0x43 0x20+read 1 BURST MODE
uint8_t fifo_BioZ = 0x45;   // 0 1 0   0 0 1 0 | 1 ultim 1 per indicar lectura seria 0x47 BURST MODE

uint8_t ECG_etag = 0;       // ECG data tag [2:0]
uint8_t ECG_ptag = 0;       // The PACE FIFO data content is closely linked to ECG FIFO content [2:0]

uint32_t ECG_SVD = 0;       // ECG Sample Voltage Data [17:0] (tenim 32 words de 24 bits ?)
uint32_t BioZ_SVD = 0;      // BioZ Sample Voltage Data [19:0] (tenim 8 words de 24 bits ?)

int main(void)
{
    //EUSCI_B_CTLW0_STEM

    /* Halting the watchdog */
    WDT_A_holdTimer();

    init_spi();
    init_uart();

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

    /**********************FIRST COMMUNICATION WITH ECG SIGNAL************************************/

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer

    /* Send the next data packet */
    SPI_transmitData(EUSCI_B2_BASE, fifo_ECG);     // send the register to read the raw data of ECG

    while (!(SPI_getInterruptStatus(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

    SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
    DataEcg[0] = SPI_receiveData(EUSCI_B2_BASE);   // we receive the data

    SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
    DataEcg[1] = SPI_receiveData(EUSCI_B2_BASE);   // we receive the data

    SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
    DataEcg[2] = SPI_receiveData(EUSCI_B2_BASE);   // we receive the data

    GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // chip select high

    /*******************************END FIRST COMMUNICATION***************************************/

    /**********************SECOND COMMUNICATION WITH BIOZ SIGNAL**********************************/

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer

    /* Send the next data packet */
    SPI_transmitData(EUSCI_B2_BASE, fifo_BioZ);    // send the register to read the raw data of BioZ

    while (!(SPI_getInterruptStatus(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

    SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
    DataBioZ[0] = SPI_receiveData(EUSCI_B2_BASE);  // we receive the data

    SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
    DataBioZ[1] = SPI_receiveData(EUSCI_B2_BASE);  // we receive the data

    SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
    DataBioZ[2] = SPI_receiveData(EUSCI_B2_BASE);  // we receive the data

    GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // chip select high

    /*******************************END FIRST COMMUNICATION***************************************/

    /***********************COOKING RAW DATA WE RECIVED FROM MAX30001*****************************/

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

    UART_transmitData(EUSCI_A0_BASE, 0x70);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
        // UART_transmitData(EUSCI_A0_BASE, UART_receiveData(EUSCI_A0_BASE));

    }
}
