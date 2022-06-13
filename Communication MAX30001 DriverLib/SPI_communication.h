/*
 * SPI_communication.h
 *
 *  Created on: 24 d’abr. 2022
 *      Author: ainar
 */

#ifndef SPI_COMMUNICATION_H_
#define SPI_COMMUNICATION_H_

uint32_t DataBioZ;
uint32_t DataEcg;

/* SPI COMUNICATION DEFINES */
#define   MOSI            GPIO_PIN6
#define   MISO            GPIO_PIN7
#define   SCLK            GPIO_PIN5
#define   CS1             GPIO_PIN5

/* REGISTERS VALUES */
#define   NO_OP           0x00
#define   STATUS          0x03              //R
#define   SW_RST          0x10              //W
#define   SYNCH           0x12              //W
#define   FIFO_RST        0x14              //W
#define   INFO            0x1F              //R
#define   CNFG_GEN        0x20              //W
#define   CNFG_CAL        0x24              //W
#define   CNFG_EMUX       0x28              //W
#define   CNFG_ECG        0x2A              //W
#define   CNFG_BMUX       0x2E              //W
#define   CNFG_BIOZ       0x30              //W
#define   ECG_FIFO_BURST  0x41              //R
#define   ECG_FIFO        0x43              //R
#define   BIOZ_FIFO_BURST 0x45              //R
#define   BIOZ_FIFO       0x47              //R
#define   CNFG_RTOR1      0x88

/* DIFFERENTS ECG OR BIOZ DATA TAGS */
#define   VALID           0x00
#define   FAST            0x01
#define   VALID_EOF       0x02
#define   FAST_EOF        0x03
#define   UNUSED_1        0x04
#define   UNUSED_2        0x05
#define   EMPTY           0x06
#define   OVERFLOW        0x07

/* Initialization of the SPI module B2
 * INPUTS:
 * - None
 * OUTPUTS:
 * - None
 * */
void init_spi(void);

/* Function that writes into the internal register of MAX30001
 * INPUTS:
 * - write_addr: enter the register where we want to write
 * - data_send: data we want to write in the register we specified
 * OUTPUTS:
 * - None
 * */
void max30001_RegWrite(uint8_t write_addr, uint32_t data_send);

/* Function that reads from the internal register of MAX30001
 * INPUTS:
 * - reg_addr: enter the register where we want to read
 * - data_recieved: pointer that poitns the register where the read data is
 * OUTPUTS:
 * - None
 * */
void max30001_RegRead(uint8_t reg_addr, uint8_t *data_recieved);

/* Writes a total Reset to the Max30001
 * INPUTS:
 * - None
 * OUTPUTS:
 * - None
 * */
void max30001_SwReset(void);

/* Reads the Info register from the Max30001
 * INPUTS:
 * - None
 * OUTPUTS:
 * - True: The info register was read successfully
 * - False: The info register was not read successfully
 * */
bool max30001_ReadInfo(void);

/* Writes in the registers needed the calibration for the Max30001
 * INPUTS:
 * - None
 * OUTPUTS:
 * - None
 * */
void max30001_CalibrationECG(void);

/* Writes in the registers needed the calibration for the Max30001
 * INPUTS:
 * - None
 * OUTPUTS:
 * - None
 * */
void max30001_CalibrationBioZ(void);

/* Gets the values from the ECG FIFO from the Max30001
 * INPUTS:
 * - None
 * OUTPUTS:
 * - DataEcg: uint32_t that keeps the value we are reading from the FIFO ECG register
 * */
uint32_t max30001_getEcgValue(void);

/* Gets the values from the BIOZ FIFO from the Max30001
 * INPUTS:
 * - None
 * OUTPUTS:
 * - DataBioZ: uint32_t that keeps the value we are reading from the FIFO BIOZ register
 * */
uint32_t max30001_getBioZValue(void);

#endif /* SPI_COMMUNICATION_H_ */
