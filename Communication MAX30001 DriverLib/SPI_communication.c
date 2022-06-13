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
 * ACLK = ~32.768kHz, MCLK = SMCLK = DCO 24MHz
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
#include "delay.h"

/* Statics */
uint8_t DataRecieved[3];    // buffer where we keep the data we receive from SPI communication

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

void max30001_RegWrite(uint8_t write_addr, uint32_t data_send)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer

    /* Send the register we desire to communicate with */
    delay(2); // 2us of delay
    SPI_transmitData(EUSCI_B2_BASE, write_addr);

    /* Send the value desired */
    SPI_transmitData(EUSCI_B2_BASE, data_send >> 16);
    SPI_transmitData(EUSCI_B2_BASE, data_send >> 8);
    SPI_transmitData(EUSCI_B2_BASE, data_send);
    delay(2); // 2us of delay

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
    delay(100);                            // Delay post reset communication 100 us
}

void max30001_Synch(void)
{
    max30001_RegWrite(SYNCH, 0x000000);
}

bool max30001_ReadInfo(void)
{
    uint8_t InfoRecieved[4];
    uint8_t i;

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, CS1);     // chip select low to start the transfer

    SPI_transmitData(EUSCI_B2_BASE, INFO);         // Send register location

    for(i = 0; i < 3; i++)
    {
       SPI_transmitData(EUSCI_B2_BASE, 0xFF);
       InfoRecieved[i] = SPI_receiveData(EUSCI_B2_BASE);
    }

    GPIO_setOutputHighOnPin(GPIO_PORT_P2, CS1);    // chip select high

    if((InfoRecieved[0] & 0xF0) == 0x50)
    {
        /* max30001 is ready */
        return true;
    }
    else
    {
        /* max30001 read info error */
        return false;
    }
}

void max30001_CalibrationECG(void)
{
    /***************************WE START CALIBRATING THE DEVICE**********************************/
    max30001_SwReset();
    delay(100);         // delay of 100 us

    max30001_RegWrite(CNFG_GEN, 0x080017); // ECG channel enable. ECG Resistive Bias enabled. 100Mohms. ECGP/ECGN connected to Vmid through a resistor.
    delay(100);

    max30001_RegWrite(CNFG_CAL, 0x004800); // FCAL = Fmstr/2^15. THIGH = 50% CAL_THIGH are ignored.
    delay(100);

    max30001_RegWrite(CNFG_EMUX, 0x000000); // ECGP/ECGN is internally connected to the ECG AFE.
    delay(100);

    max30001_RegWrite(CNFG_ECG, 0x805000); // 128 sps. ECG_DHPF a 0.50 Hz. ECG_DLPF 28.35 Hz.
    delay(100);

    max30001_RegWrite(CNFG_RTOR1, 0x36A300);
    max30001_Synch();
    delay(100);
}

void max30001_CalibrationBioZ(void)
{
    /***************************WE START CALIBRATING THE DEVICE**********************************/
    max30001_SwReset();
    delay(100);         // delay of 100 us

    max30001_RegWrite(CNFG_GEN, 0x040027);
    delay(100);

    max30001_RegWrite(CNFG_CAL, 0x004800);
    delay(100);

    max30001_RegWrite(CNFG_BMUX, 0x001040);
    delay(100);

    max30001_RegWrite(CNFG_BIOZ, 0x201130);
    delay(100);

    max30001_RegWrite(CNFG_RTOR1, 0x3F2300);
    max30001_Synch();
    delay(100);
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
