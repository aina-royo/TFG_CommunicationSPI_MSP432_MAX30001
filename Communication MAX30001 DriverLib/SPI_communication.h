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

/* Initialization of the SPI module B2 */
void init_spi(void);

#endif /* SPI_COMMUNICATION_H_ */
