/*
 * SPI_communication.h
 *
 *  Created on: 24 d’abr. 2022
 *      Author: ainar
 */

#ifndef SPI_COMMUNICATION_H_
#define SPI_COMMUNICATION_H_


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

/* Initialization of the SPI module B2 */
void init_spi(void);

/* Function that writes into the internal register of MAX30001 */
void max30001_RegWrite(uint8_t write_addr, uint32_t data_send);

/* Function that reads from the internal register of MAX30001 */
void max30001_RegRead(uint8_t reg_addr, uint8_t *data_recieved)

#endif /* SPI_COMMUNICATION_H_ */
