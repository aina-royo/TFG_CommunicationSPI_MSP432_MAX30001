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
#include "delay.h"

/* SPI COMUNICATION DEFINES */
#define MOSI        GPIO_PIN6
#define MISO        GPIO_PIN7
#define SCLK        GPIO_PIN5
#define CS1         GPIO_PIN5

/* REGISTERS VALUES */
#define   NO_OP           0x00
#define   STATUS          0x03
#define   SW_RST          0x10
#define   SYNCH           0x09
#define   INFO            0x1F
#define   CNFG_GEN        0x21
#define   CNFG_CAL        0x25
#define   CNFG_EMUX       0x29
#define   CNFG_ECG        0x2B
#define   CNFG_BMUX       0x2F
#define   CNFG_BIOZ       0x31
#define   ECG_FIFO_BURST  0x41
#define   ECG_FIFO        0x43
#define   BIOZ_FIFO_BURST 0x45
#define   BIOZ_FIFO       0x47

#define   DELAY_1s        1000000
#define   DELAY_1ms       1000
#define   DELAY_100ms     100000


/* Statics */
//uint8_t DataEcg[3];         // buffer on emmagatzemem les dades de lectura de ECG 3 byte data word
//uint8_t DataBioZ[3];        // buffer on emmagatzemem les dades de lectura del BioZ 3 byte data word
uint8_t DataRecieved[3];

uint32_t DataBioZ;
uint32_t DataEcg;

uint8_t Flag_ecgbioz;

/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig spiMasterConfig =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,                              // SMCLK Clock Source
        24000000,                                                   // SMCLK = DCO = 24MHz
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

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);                    // 24000000 Hz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // 24000000 Hz

    init_delay();

    DataEcg = 0;
    DataBioZ = 0;

    Flag_ecgbioz = 1;

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

    /* Enabling interrupts SPI */
    SPI_enableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);
    SPI_enableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIB2);

    while(1)
    {

    }
}

void max30001_RegWrite(uint8_t write_addr, uint32_t data_send)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer

    /* Send the register we desire to communicate with */
    delay(2*DELAY_1ms);
    SPI_transmitData(EUSCI_B2_BASE, write_addr);

    /* Send the value desired */
    SPI_transmitData(EUSCI_B2_BASE, data_send >> 16);
    SPI_transmitData(EUSCI_B2_BASE, data_send >> 8);
    SPI_transmitData(EUSCI_B2_BASE, data_send);
    delay(2*DELAY_1ms);

    GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // chip select high
}

void max30001_RegRead(uint8_t reg_addr, uint8_t *data_recieved)
{
    uint8_t i;

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer

    SPI_transmitData(EUSCI_B2_BASE, reg_addr);     // Send register location

    for(i = 0; i < 3; i++)
    {
       SPI_transmitData(EUSCI_B2_BASE, 0xFF);
       data_recieved[i] = SPI_receiveData(EUSCI_B2_BASE);
    }

    GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // chip select high
}

void max30001_SwReset(void)
{
    max30001_RegWrite(SW_RST, 0x000000);
    delay(DELAY_100ms);                            // Delay post reset communication
}


void max30001_ReadInfo(void)
{
    uint8_t InfoRecieved[3];

    max30001_RegRead(INFO, InfoRecieved);

    /*InfoRecieved = InfoRecieved >> 0x01;

    if(InfoRecieved )
    {

    }*/
}

void max30001_Calibration(void)
{
    /***************************WE START CALIBRATING THE DEVICE**********************************/
    max30001_SwReset();
    delay(DELAY_100ms);

    max30001_RegWrite(CNFG_GEN, 0x0E0017);
    delay(DELAY_100ms);

    max30001_RegWrite(CNFG_CAL, 0x004800);
    delay(DELAY_100ms);

    max30001_RegWrite(CNFG_EMUX, 0x000000);
    delay(DELAY_100ms);

    max30001_RegWrite(CNFG_ECG, 0x805000);
    delay(DELAY_100ms);

    max30001_RegWrite(CNFG_BMUX, 0x001040);
    delay(DELAY_100ms);

    max30001_RegWrite(CNFG_BIOZ, 0x201130);
    delay(DELAY_100ms);
}

uint32_t max30001_getEcgValue(void)
{
    uint32_t data0, data1, data2;

    max30001_RegRead(ECG_FIFO, DataRecieved);

    data0 = (DataRecieved[0] << 24) & 0xF00;
    data1 = (DataRecieved[1] << 16) & 0x0F0;
    data2 = (DataRecieved[2] >> 6) & 0x00F;

    return DataEcg = data0 | data1 | data2;            // real value of the ECG data
}

uint32_t max30001_getBioZValue(void)
{
    uint32_t data0, data1, data2;

    max30001_RegRead(BIOZ_FIFO, DataRecieved);

    data0 = (DataRecieved[0] << 24) & 0xF00;
    data1 = (DataRecieved[1] << 16) & 0x0F0;
    data2 = (DataRecieved[2] >> 4) & 0x00F;

    return DataBioZ = data0 | data1 | data2;            // real value of the ECG data
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

    if(Flag_ecgbioz == 1)
    {
        /**********************FIRST COMMUNICATION WITH ECG SIGNAL************************************/
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer

        delay(2);                                // 2us || 11.5 us de delay
        /* Send the next data packet */
        SPI_transmitData(EUSCI_B2_BASE, 0x43);     // send the register to read the raw data of ECG
        //delay(6);
        //while (!(SPI_getInterruptStatus(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);              // we don't care, we just need clock signal
        //DataRecieved[0] = SPI_receiveData(EUSCI_B2_BASE);   // we receive the data
        //delay(6);

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        //DataRecieved[1] = SPI_receiveData(EUSCI_B2_BASE);   // we receive the data
        //delay(6);
        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        //DataRecieved[2] = SPI_receiveData(EUSCI_B2_BASE);   // we receive the data

        delay(2);                                  // 2us

        GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // chip select high

        /*******************************END FIRST COMMUNICATION***************************************/
    }
    else if(Flag_ecgbioz == 2)
    {
        /**********************SECOND COMMUNICATION WITH BIOZ SIGNAL**********************************/

        GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer

        /* Send the next data packet */
        SPI_transmitData(EUSCI_B2_BASE, INFO);    // send the register to read the raw data of BioZ

        while (!(SPI_getInterruptStatus(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        DataRecieved[0] = SPI_receiveData(EUSCI_B2_BASE);  // we receive the data

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        DataRecieved[1] = SPI_receiveData(EUSCI_B2_BASE);  // we receive the data

        SPI_transmitData(EUSCI_B2_BASE, 0xFF);         // we don't care, we just need clock signal
        DataRecieved[2] = SPI_receiveData(EUSCI_B2_BASE);  // we receive the data

        GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // chip select high

        /*******************************END FIRST COMMUNICATION***************************************/
    }
    else
    {
        while(1);         /*NOT IDEA CUELGATE*/
    }
}
