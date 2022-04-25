/*
 * UART_COM_PORT.h
 *
 *  Created on: 21 d’abr. 2022
 *      Author: ainar
 */

//#include "UART_COM_PORT.c"

#ifndef UART_COM_PORT_H_
#define UART_COM_PORT_H_

/* UART COMUNICATION DEFINES */
#define UART_RX        GPIO_PIN2
#define UART_TX        GPIO_PIN3

/* Initialization of the UART module A0 */
void init_uart(void);

#endif /* UART_COM_PORT_H_ */
